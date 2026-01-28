#include "dht22_driver.h"
#include "dht.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "system_config.h"

static const char *TAG = "DHT22_DRIVER";

#define DHT_HISTORY_LEN 8

typedef struct {
    int64_t ts_us;
    float temperature;
} dht_hist_sample_t;

static dht_hist_sample_t s_hist[DHT_HISTORY_LEN];
static size_t s_hist_head = 0;
static size_t s_hist_count = 0;

static void dht_hist_push(int64_t ts_us, float temp_c)
{
    s_hist[s_hist_head].ts_us = ts_us;
    s_hist[s_hist_head].temperature = temp_c;
    s_hist_head = (s_hist_head + 1) % DHT_HISTORY_LEN;
    if (s_hist_count < DHT_HISTORY_LEN) {
        s_hist_count++;
    }
}

static bool dht_hist_oldest(dht_hist_sample_t *out)
{
    if (s_hist_count == 0) return false;
    size_t tail = (s_hist_head + DHT_HISTORY_LEN - s_hist_count) % DHT_HISTORY_LEN;
    *out = s_hist[tail];
    return true;
}


esp_err_t dht_read(dht_reading_t *data) 
{
    float humidity = 0.0f, temperature = 0.0f;

    esp_err_t result = dht_read_float_data(
        DHT_TYPE_AM2301,
        DHT22_PIN,
        &humidity,
        &temperature
    );

    if (result != ESP_OK) return result;

    int64_t now_us = esp_timer_get_time();
    dht_hist_sample_t oldest;
    float rate_per_min = 0.0f;

    if (dht_hist_oldest(&oldest)) {
        float delta_c = temperature - oldest.temperature;
        float delta_min = (float)(now_us - oldest.ts_us) / 1000000.0f / 60.0f;
        if (delta_min > 0.01f) {
            rate_per_min = delta_c / delta_min;
        }
    }

    dht_hist_push(now_us, temperature);

    data->temperature = temperature;
    data->humidity = humidity;
    data->temp_rate_per_min = rate_per_min;
    data->timestamp = now_us;

    return ESP_OK;
}

dht_threat_level_t dht_evaluate_threat(const dht_reading_t *reading) 
{
    if (reading == NULL) {
        return DHT_THREAT_NONE;
    }

    float temp = reading->temperature;
    float hum = reading->humidity;
    float rate = reading->temp_rate_per_min;

    bool very_dry = hum <= SENTINEL_HUMIDITY_LOW_THRESHOLD_PCT;
    bool above_fire = temp >= SENTINEL_TEMP_FIRE_THRESHOLD_C;

    dht_threat_level_t level = DHT_THREAT_NONE;

    if (above_fire && very_dry) {
        level = DHT_THREAT_HIGH;
    } else if (above_fire) {
        level = DHT_THREAT_MEDIUM;
    } else if (temp >= (SENTINEL_TEMP_FIRE_THRESHOLD_C - 5.0f)) {
        level = DHT_THREAT_LOW;
    }

    if (rate >= 10.0f) {
        if (level < DHT_THREAT_CRITICAL) level = DHT_THREAT_CRITICAL ;
    } else if (rate >= 5.0f) {
        if (level < DHT_THREAT_HIGH) level = DHT_THREAT_HIGH;
    } else if (rate >= 2.0f) {
        if (level < DHT_THREAT_MEDIUM) level = DHT_THREAT_MEDIUM;
    } else if (rate >= 0.5f) {
        if (level < DHT_THREAT_LOW) level = DHT_THREAT_LOW;
    }

    return level;
}