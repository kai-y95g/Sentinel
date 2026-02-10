#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file dht22_driver.h
 * @brief High-level DHT22 (AM2301) driver with trend analysis and threat evaluation
 *
 * This module provides a safe, filtered interface for reading temperature
 * and humidity from a DHT22 sensor. In addition to raw readings, it computes
 * the temperature rate-of-change and evaluates environmental threat levels
 * for the Sentinel system.
 *
 * Features:
 *  - Input validation and sanity checks
 *  - Temperature history buffering
 *  - Temperature rate calculation (°C per minute)
 *  - Threat level evaluation using static and dynamic thresholds
 *
 * The driver is designed to be called periodically (e.g., every few seconds)
 * from an application task.
 */

/* -------------------------------------------------------------------------- */
/*                              Data Structures                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief Single DHT22 sensor reading with trend data
 *
 * All values are validated before being returned to the caller.
 */
typedef struct {
    float    temperature;        /**< Temperature in degrees Celsius */
    float    humidity;           /**< Relative humidity in percent (0–100) */
    float    temp_rate_per_min;  /**< Temperature change rate (°C/min) */
    int64_t  timestamp;          /**< Timestamp of reading (microseconds since boot) */
} dht_reading_t;

/**
 * @brief Environmental threat levels derived from sensor data
 *
 * These levels are used by Sentinel to classify environmental risk,
 * especially for early fire detection.
 */
typedef enum {
    DHT_THREAT_NONE = 0,      /**< Normal environmental conditions */
    DHT_THREAT_LOW,           /**< Slightly elevated temperature or rising trend */
    DHT_THREAT_MEDIUM,        /**< Sustained high temperature or moderate rise */
    DHT_THREAT_HIGH,          /**< Dangerous conditions detected */
    DHT_THREAT_CRITICAL       /**< Rapid temperature increase indicating immediate threat */
} dht_threat_level_t;

/* -------------------------------------------------------------------------- */
/*                               Public API                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief Read temperature and humidity from the DHT22 sensor
 *
 * This function performs a sensor read, validates the returned values,
 * updates the internal temperature history buffer, and calculates the
 * temperature rate-of-change when sufficient history is available.
 *
 * On success, the provided @p data structure is fully populated.
 *
 * @param[out] data Pointer to a dht_reading_t structure to receive results
 *
 * @return
 *  - ESP_OK on successful read and validation
 *  - ESP_ERR_INVALID_ARG if @p data is NULL
 *  - ESP_FAIL if the reading is invalid or fails sanity checks
 *  - Any error code returned by the underlying DHT driver
 *
 * @note
 *  - Temperature rate will be zero until enough history samples exist
 *  - This function is not thread-safe; call from a single task
 */
esp_err_t dht_read(dht_reading_t *data);

/**
 * @brief Evaluate environmental threat level from a DHT22 reading
 *
 * This function applies Sentinel-specific logic to determine the threat
 * level based on:
 *  - Absolute temperature
 *  - Relative humidity
 *  - Temperature rate-of-change
 *
 * Rate-based escalation can override static thresholds, allowing early
 * detection of rapidly developing hazards.
 *
 * @param[in] r Pointer to a previously obtained dht_reading_t
 *
 * @return A dht_threat_level_t classification
 *
 * @note
 *  - Passing NULL returns DHT_THREAT_NONE
 *  - This function performs no I/O and is deterministic
 */
dht_threat_level_t dht_evaluate_threat(const dht_reading_t *r);

#ifdef __cplusplus
}
#endif
