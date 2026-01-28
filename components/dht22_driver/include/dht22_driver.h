#ifndef _DHT22_DRIVER_H_
#define _DHT22_DRIVER_H_

#include "esp_err.h"
#include <stdint.h>


typedef struct {
    float temperature; // In Celsius
    float humidity; // In percentage
    float temp_rate_per_min; // Rate of increase of temperature per minute
    int64_t timestamp;
} dht_reading_t;


typedef enum {
    DHT_THREAT_NONE = 0,
    DHT_THREAT_LOW,
    DHT_THREAT_MEDIUM,
    DHT_THREAT_HIGH,
    DHT_THREAT_CRITICAL,
} dht_threat_level_t;


/**
 * @brief Perform a DHT22 read.
 *
 * Includes checksum verification. On success, updates internal
 * history for temperature rate-of-rise calculation.
 */
esp_err_t dht_read(dht_reading_t *data);


/**
 * @brief Classify fire-related threat based on temperature and rate-of-rise.
 *
 * Uses SENTINEL_TEMP_FIRE_THRESHOLD_C & SENTINEL_HUMIDITY_LOW_THRESHOLD_PCT
 * from system_config.h plus temperature rate-of-rise.
 */
dht_threat_level_t dht_evaluate_threat(const dht_reading_t *reading);

#endif