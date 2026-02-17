#pragma once

/**
 * @file mpu6050_driver.h
 * @brief High-level MPU6050 accelerometer driver API with earthquake P-wave detection
 *
 * This header provides:
 *  - MPU6050 initialization and data acquisition
 *  - Ring-buffered accelerometer storage
 *  - DSP task prototype for FFT-based P-wave energy computation
 *  - Earthquake threat evaluation based on energy thresholds
 *
 * Designed for ESP-IDF (v5.x) embedded monitoring systems.
 */

#include "esp_err.h"
#include "i2cdev.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/*                                I2C configuration                            */
/* -------------------------------------------------------------------------- */

/** I2C peripheral number used for MPU6050 */
#define I2C_NUM I2C_NUM_0

/* -------------------------------------------------------------------------- */
/*                              DSP / Sampling constants                      */
/* -------------------------------------------------------------------------- */

/** Number of samples in circular buffer and FFT */
#define N       256

/** DSP hop size (number of samples between processing windows) */
#define STEP    128

/** Sampling frequency in Hz */
#define FS      100.0f

/** Bandpass filter lower frequency (Hz) */
#define F_LOW   1.0f

/** Bandpass filter higher frequency (Hz) */
#define F_HIGH  10.0f

/** FFT bin index corresponding to F_LOW */
#define BIN_F1 ((int)(F_LOW  * N / FS))

/** FFT bin index corresponding to F_HIGH */
#define BIN_F2 ((int)(F_HIGH * N / FS))

/* -------------------------------------------------------------------------- */
/*                              IIR filter coefficients                        */
/* -------------------------------------------------------------------------- */

/** Bandpass filter numerator and denominator coefficients */
#define B0  0.056448462f
#define B1  0.0f
#define B2 -0.056448462f
#define A1 -1.777737f
#define A2  0.887103f

/* -------------------------------------------------------------------------- */
/*                             Earthquake threat levels                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief Earthquake P-wave threat levels based on FFT energy
 */
typedef enum {
    EQ_THREAT_NONE = 0,   /**< No threat detected */
    EQ_THREAT_LOW,        /**< Low probability of earthquake */
    EQ_THREAT_MEDIUM,     /**< Medium probability of earthquake */
    EQ_THREAT_HIGH,       /**< High probability of earthquake detected */
} eq_threat_level;

/* -------------------------------------------------------------------------- */
/*                               Public API                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize MPU6050 sensor and DSP resources
 *
 * - Configures I2C
 * - Initializes MPU6050 driver
 * - Prepares FFT and Hann window for energy computation
 */
void mpu6050_driver_init(void);

/**
 * @brief Read raw acceleration from MPU6050
 *
 * @param ax Pointer to float to receive X-axis acceleration
 * @param ay Pointer to float to receive Y-axis acceleration
 * @param az Pointer to float to receive Z-axis acceleration
 * @return ESP_OK on success, otherwise ESP_ERR_*
 */
esp_err_t mpu6050_driver_read_data(float *ax, float *ay, float *az);

/**
 * @brief Combined DSP task for MPU6050
 *
 * - Periodically samples accelerometer
 * - Computes magnitude and applies bandpass filter
 * - Calculates FFT and energy
 * - Evaluates earthquake threat level
 *
 * @param arg Task argument (unused)
 */
void mpu_dsp_task(void *arg);

/**
 * @brief Evaluate earthquake threat based on energy
 *
 * Converts FFT energy to dB, smooths, and thresholds:
 *  - EQ_THREAT_NONE
 *  - EQ_THREAT_LOW
 *  - EQ_THREAT_MEDIUM
 *  - EQ_THREAT_HIGH
 *
 * @param E FFT band energy
 * @return Evaluated threat level
 */
eq_threat_level evaluate_threat_eq(float E);

#ifdef __cplusplus
}
#endif
