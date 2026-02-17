#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define N       256
#define STEP    128
#define FS      100.0f
#define F_LOW   1.0f
#define F_HIGH  10.0f

#define BIN_F1 ((int)(F_LOW  * N / FS))
#define BIN_F2 ((int)(F_HIGH * N / FS))

#define B0  0.056448462f
#define B1  0.0f
#define B2 -0.056448462f
#define A1 -1.777737f
#define A2  0.887103f

typedef enum {
    EQ_THREAT_NONE = 0,
    EQ_THREAT_LOW,
    EQ_THREAT_MEDIUM,
    EQ_THREAT_HIGH,
} eq_threat_level;


void mpu6050_driver_init();

esp_err_t mpu6050_driver_read_data(float *ax, float *ay, float *az);

void mpu_dsp_task(void *arg);

eq_threat_level evaluate_threat_eq(float E);

#ifdef __cplusplus
}
#endif