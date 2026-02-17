idf_component_register(
    SRCS "mpu6050_driver.c"
    INCLUDE_DIRS "include"
    REQUIRES mpu6050 config driver esp-dsp
)