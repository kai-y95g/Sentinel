/**
 * @file dht22_driver.c
 * @brief High-level DHT22 sensor driver with trend analysis and threat evaluation
 *
 * This module wraps a low-level DHT22 driver and provides:
 *  - Validated temperature & humidity readings
 *  - Temperature rate-of-change calculation (°C/min)
 *  - Early fire-risk threat evaluation based on thresholds and trends
 *
 * Designed for ESP-IDF (v5.x) and safety/monitoring systems such as Sentinel.
 *
 * Key design goals:
 *  - Deterministic memory usage (no dynamic allocation)
 *  - Robustness against sensor glitches
 *  - Early detection of rapid temperature rise
 *  - Clear separation between data acquisition and policy logic
 */

#include "dht22_driver.h"
#include "dht.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "system_config.h"

/* Logging tag for ESP-IDF log system */
static const char *TAG = "DHT22_DRIVER";

/* Number of historical temperature samples retained for trend analysis */
#define DHT_HISTORY_LEN 8

/* Valid operating bounds for DHT22 (used to reject glitch readings) */
#define DHT_MIN_VALID_TEMP_C (-20.0f)
#define DHT_MAX_VALID_TEMP_C (80.0f)

/**
 * @brief One historical temperature sample
 *
 * Stores temperature along with a precise timestamp so that
 * rate-of-change can be calculated reliably.
 */
typedef struct
{
    int64_t ts_us;     /**< Timestamp in microseconds since boot */
    float temperature; /**< Temperature in degrees Celsius */
} dht_hist_sample_t;

/* Ring buffer storage for historical samples */
static dht_hist_sample_t s_hist[DHT_HISTORY_LEN];

/* Index of next write position in ring buffer */
static size_t s_hist_head = 0;

/* Number of valid samples currently stored */
static size_t s_hist_count = 0;

/* -------------------------------------------------------------------------- */
/*                         History buffer helpers                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Push a new temperature sample into the history buffer
 *
 * Implements a fixed-size ring buffer. When full, the oldest
 * sample is automatically overwritten.
 *
 * @param ts_us  Timestamp in microseconds
 * @param temp_c Temperature in Celsius
 */
static void dht_hist_push(int64_t ts_us, float temp_c)
{
    s_hist[s_hist_head].ts_us = ts_us;
    s_hist[s_hist_head].temperature = temp_c;

    /* Advance write index with wrap-around */
    s_hist_head = (s_hist_head + 1) % DHT_HISTORY_LEN;

    /* Increase count until buffer becomes full */
    if (s_hist_count < DHT_HISTORY_LEN)
    {
        s_hist_count++;
    }
}

/**
 * @brief Retrieve the oldest valid sample from history
 *
 * @param[out] out Pointer to receive the oldest sample
 * @return true if a sample was returned, false if history is empty
 */
static bool dht_hist_oldest(dht_hist_sample_t *out)
{
    if (s_hist_count == 0)
    {
        return false;
    }

    /*
     * Oldest sample index is calculated relative to the current head.
     * This avoids shifting data and keeps operations O(1).
     */
    size_t tail =
        (s_hist_head + DHT_HISTORY_LEN - s_hist_count) % DHT_HISTORY_LEN;

    *out = s_hist[tail];
    return true;
}

/* -------------------------------------------------------------------------- */
/*                          Public sensor interface                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Read DHT22 sensor and compute temperature trend
 *
 * Performs:
 *  - Hardware read from DHT22
 *  - Sanity validation of values
 *  - Temperature rate-of-change calculation (°C/min)
 *  - Timestamping
 *
 * @param[out] data Pointer to output reading structure
 * @return ESP_OK on success, error code otherwise
 *
 * @note Must not be called faster than once every 2 seconds
 *       (DHT22 hardware limitation)
 */
esp_err_t dht_read(dht_reading_t *data)
{
    if (!data)
    {
        return ESP_ERR_INVALID_ARG;
    }

    float humidity = 0.0f;
    float temperature = 0.0f;

    /* Read raw sensor data via low-level driver */
    esp_err_t res = dht_read_float_data(
        DHT_TYPE_AM2301,
        DHT22_PIN,
        &humidity,
        &temperature);

    if (res != ESP_OK)
    {
        ESP_LOGW(TAG, "DHT read failed (%d)", res);
        return res;
    }

    /* Reject clearly invalid or glitch values */
    if (humidity < 0.0f || humidity > 100.0f ||
        temperature < DHT_MIN_VALID_TEMP_C ||
        temperature > DHT_MAX_VALID_TEMP_C)
    {

        ESP_LOGW(TAG,
                 "Discarding invalid reading T=%.2f H=%.2f",
                 temperature, humidity);

        return ESP_FAIL;
    }

    int64_t now_us = esp_timer_get_time();
    float rate_per_min = 0.0f;

    /*
     * Compute rate-of-change only once enough history is available.
     * This avoids unstable values during startup.
     */
    if (s_hist_count >= (DHT_HISTORY_LEN / 2))
    {

        dht_hist_sample_t oldest;
        if (dht_hist_oldest(&oldest))
        {

            float delta_c = temperature - oldest.temperature;

            /* Convert microseconds to minutes */
            float delta_min =
                (float)(now_us - oldest.ts_us) / 1000000.0f / 60.0f;

            if (delta_min > 0.01f)
            {
                rate_per_min = delta_c / delta_min;
            }
        }
    }

    /* Store current sample for future trend calculations */
    dht_hist_push(now_us, temperature);

    /* Populate output structure */
    data->temperature = temperature;
    data->humidity = humidity;
    data->temp_rate_per_min = rate_per_min;
    data->timestamp = now_us;

    ESP_LOGD(TAG,
             "T=%.2fC H=%.2f%% Rate=%.2f C/min",
             temperature, humidity, rate_per_min);

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*                           Threat evaluation logic                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Evaluate environmental threat level from DHT22 data
 *
 * Uses:
 *  - Absolute temperature thresholds
 *  - Humidity conditions
 *  - Temperature rate-of-change
 *
 * Rate-based escalation can override static thresholds to
 * provide early fire detection.
 *
 * @param r Pointer to a valid DHT reading
 * @return Evaluated threat level
 */
dht_threat_level_t dht_evaluate_threat(const dht_reading_t *r)
{
    if (!r)
    {
        return DHT_THREAT_NONE;
    }

    float temp = r->temperature;
    float hum = r->humidity;
    float rate = r->temp_rate_per_min;

    bool very_dry = hum <= SENTINEL_HUMIDITY_LOW_THRESHOLD_PCT;
    bool near_fire = temp >= (SENTINEL_TEMP_FIRE_THRESHOLD_C - 5.0f);
    bool above_fire = temp >= SENTINEL_TEMP_FIRE_THRESHOLD_C;

    dht_threat_level_t level = DHT_THREAT_NONE;

    /* Static temperature-based assessment */
    if (above_fire && very_dry)
    {
        level = DHT_THREAT_HIGH;
    }
    else if (above_fire)
    {
        level = DHT_THREAT_MEDIUM;
    }
    else if (near_fire)
    {
        level = DHT_THREAT_LOW;
    }

    /*
     * Rate-of-rise escalation
     * Rapid temperature increases are strong indicators of fire
     * even before absolute thresholds are crossed.
     */
    if (rate >= SENTINEL_TEMP_RATE_CRITICAL)
    {
        level = DHT_THREAT_CRITICAL;
    }
    else if (rate >= SENTINEL_TEMP_RATE_HIGH)
    {
        if (level < DHT_THREAT_HIGH)
            level = DHT_THREAT_HIGH;
    }
    else if (rate >= SENTINEL_TEMP_RATE_MEDIUM)
    {
        if (level < DHT_THREAT_MEDIUM)
            level = DHT_THREAT_MEDIUM;
    }
    else if (rate >= SENTINEL_TEMP_RATE_LOW)
    {
        if (level < DHT_THREAT_LOW)
            level = DHT_THREAT_LOW;
    }

    /* Extra caution when environment is dry */
    if (very_dry && rate >= SENTINEL_TEMP_RATE_MEDIUM)
    {
        if (level < DHT_THREAT_HIGH)
            level = DHT_THREAT_HIGH;
    }

    return level;
}