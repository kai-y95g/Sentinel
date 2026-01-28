idf_component_register(
    SRCS "dht22_driver.c"
    INCLUDE_DIRS "include"
    REQUIRES dht driver config esp_timer
)