#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FS          100.0f
#define STA_SEC     0.5f
#define LTA_SEC     10.0f
#define GRAV_TAU    1.0f
#define TRIG_ON     3.5f
#define TRIG_OFF    1.2f
#define MIN_EVENT_SAMPLES 30   // 300 ms at 100 Hz


typedef struct {
    float ax;
    float ay;
    float az;
    float magnitude;
} mpu6050_data_t;

typedef struct {
    float b0,b1,b2,a1,a2;
    float z1,z2;
} biquad_t;

typedef struct {
    float gx, gy, gz;
} gravity_t;

void mpu6050_driver_init();

esp_err_t mpu6050_driver_read_data(mpu6050_data_t *data);

void imu_task(void *arg);

#ifdef __cplusplus
}
#endif