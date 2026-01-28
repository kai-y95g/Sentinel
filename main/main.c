#include <stdio.h>
#include "dht22_driver.h"
#include "esp_log.h"
#include "inttypes.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "Test";

void app_main(void)
{
    dht_reading_t data = {
        .temperature = 46.2,
        .humidity = 25.2,
        .temp_rate_per_min = 5.5,
        .timestamp = 2448425,
    };
    
    dht_threat_level_t level = dht_evaluate_threat(&data);
    ESP_LOGI(TAG, "Temperature :- %.2f\tHumidity :- %.2f%%\tRate :- %.2f\tTimestamp :- %" PRId64,
    data.temperature, data.humidity, data.temp_rate_per_min, data.timestamp);
    ESP_LOGI(TAG, "THREAT LEVEL :- %d",level);
}