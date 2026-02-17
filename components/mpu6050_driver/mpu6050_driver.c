/**
 * @file mpu6050_driver.c
 * @brief High-level MPU6050 accelerometer driver with earthquake P-wave detection
 *
 * This module provides:
 *  - Ring-buffered accelerometer data acquisition
 *  - Magnitude computation and mean removal
 *  - IIR bandpass filtering
 *  - FFT-based energy computation for P-wave detection
 *  - Earthquake threat evaluation based on energy thresholds
 *
 * Designed for ESP-IDF (v5.x) and embedded safety/monitoring systems.
 *
 * Key design goals:
 *  - Deterministic memory usage (no dynamic allocation)
 *  - Real-time P-wave detection with minimal latency
 *  - Clear separation between data acquisition, DSP, and policy logic
 */

#include <stdio.h>
#include <math.h>
#include "esp_dsp.h"
#include "i2cdev.h"
#include "mpu6050.h"
#include "mpu6050_driver.h"
#include "esp_log.h"
#include "esp_err.h"
#include "system_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Logging tag for ESP-IDF log system */
static const char *TAG = "MPU6050_DRIVER";

/* -------------------------------------------------------------------------- */
/*                               Ring buffers                                  */
/* -------------------------------------------------------------------------- */

/* Circular buffers for raw acceleration samples */
float ax[N], ay[N], az[N];
float mag[N];         /**< Magnitude of acceleration */
float filtered[N];    /**< Filtered magnitude after bandpass */
float fft_buf[2 * N]; /**< Complex FFT buffer */
float hann[N];        /**< Hann window coefficients */

/* Write index for circular buffer */
int write_idx = 0;

/* IIR filter state variables (static across samples) */
static float iir_x1 = 0, iir_x2 = 0, iir_y1 = 0, iir_y2 = 0;

/* MPU6050 device handle */
static mpu6050_dev_t mpu = {0};
static mpu6050_acceleration_t accel;

/* -------------------------------------------------------------------------- */
/*                          Helper DSP functions                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief Convert circular buffer to linear array for DSP
 *
 * @param circ Input circular buffer
 * @param lin  Output linear array
 */
void unwrap(float *circ, float *lin)
{
    int idx = write_idx;
    for (int i = 0; i < N; i++)
        lin[i] = circ[(idx + i) % N];
}

/**
 * @brief Compute vector magnitude of 3-axis acceleration
 *
 * @param x Input X-axis samples
 * @param y Input Y-axis samples
 * @param z Input Z-axis samples
 * @param m Output magnitude
 */
void compute_magnitude(float *x, float *y, float *z, float *m)
{
    for (int i = 0; i < N; i++)
        m[i] = sqrtf(x[i] * x[i] + y[i] * y[i] + z[i] * z[i]);
}

/**
 * @brief Remove mean from input array (DC offset removal)
 *
 * @param x Input/output array
 */
void remove_mean(float *x)
{
    float sum = 0;
    for (int i = 0; i < N; i++)
        sum += x[i];
    float mean = sum / N;
    for (int i = 0; i < N; i++)
        x[i] -= mean;
}

/**
 * @brief Apply 2nd-order IIR bandpass filter
 *
 * @param x Input array
 * @param y Output filtered array
 */
void bandpass(float *x, float *y)
{
    for (int i = 0; i < N; i++)
    {
        y[i] = B0 * x[i] + B1 * iir_x1 + B2 * iir_x2 - A1 * iir_y1 - A2 * iir_y2;
        iir_x2 = iir_x1;
        iir_x1 = x[i];
        iir_y2 = iir_y1;
        iir_y1 = y[i];
    }
}

/**
 * @brief Apply Hann window for FFT
 *
 * @param x Input/output array
 */
void apply_hann(float *x)
{
    for (int i = 0; i < N; i++)
        x[i] *= hann[i];
}

/**
 * @brief Compute FFT of real-valued input
 *
 * @param x Input array
 */
void compute_fft(float *x)
{
    for (int i = 0; i < N; i++)
    {
        fft_buf[2 * i] = x[i];  // real part
        fft_buf[2 * i + 1] = 0; // imaginary part
    }

    dsps_fft2r_fc32(fft_buf, N);
    dsps_bit_rev_fc32(fft_buf, N);
    dsps_cplx2reC_fc32(fft_buf, N);
}

/**
 * @brief Compute energy of FFT within band of interest
 *
 * Energy is used for earthquake P-wave detection.
 *
 * @return Energy value
 */
float compute_energy()
{
    float E = 0;
    for (int k = BIN_F1; k <= BIN_F2; k++)
    {
        float re = fft_buf[2 * k];
        float im = fft_buf[2 * k + 1];
        E += re * re + im * im;
    }
    return E / N;
}

/* -------------------------------------------------------------------------- */
/*                            Sensor + DSP task                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief Combined MPU6050 sampling and DSP task
 *
 * Performs:
 *  - Periodic acceleration sampling
 *  - Magnitude computation
 *  - Bandpass filtering
 *  - FFT and energy calculation
 *  - Earthquake threat evaluation
 *
 * @param arg Task argument (unused)
 */
void mpu_dsp_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    int hop = 0;
    static float ax_lin[N], ay_lin[N], az_lin[N];

    while (1)
    {
        /* ---- SAMPLE ---- */
        float x, y, z;
        mpu6050_driver_read_data(&x, &y, &z);

        ax[write_idx] = x;
        ay[write_idx] = y;
        az[write_idx] = z;
        write_idx = (write_idx + 1) % N;

        /* ---- DSP every STEP samples ---- */
        if (++hop >= STEP)
        {
            hop = 0;

            /* Reset filter state for each DSP window */
            iir_x1 = iir_x2 = iir_y1 = iir_y2 = 0;

            unwrap(ax, ax_lin);
            unwrap(ay, ay_lin);
            unwrap(az, az_lin);

            compute_magnitude(ax_lin, ay_lin, az_lin, mag);
            remove_mean(mag);
            bandpass(mag, filtered);
            remove_mean(filtered);
            apply_hann(filtered);

            compute_fft(filtered);
            float E = compute_energy();

            /* Evaluate earthquake threat */
            eq_threat_level lvl = evaluate_threat_eq(E);

            // Optional debug print
            //printf("E=%.3f  LEVEL=%d\n", E, lvl);
        }

        /* Maintain sampling rate */
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / FS));
    }
}

/* -------------------------------------------------------------------------- */
/*                           Driver initialization                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize MPU6050 and DSP resources
 *
 * - Sets up I2C device
 * - Initializes FFT and Hann window
 */
void mpu6050_driver_init()
{
    ESP_ERROR_CHECK(i2cdev_init());
    ESP_ERROR_CHECK(
        mpu6050_init_desc(&mpu, MPU6050_ADDR, I2C_NUM, MPU6050_SDA_PIN, MPU6050_SCL_PIN));
    ESP_ERROR_CHECK(mpu6050_init(&mpu));

    ESP_LOGI(TAG, "initialized device.");

    dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    dsps_wind_hann_f32(hann, N);
}

/* -------------------------------------------------------------------------- */
/*                            Sensor reading                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief Read acceleration from MPU6050
 *
 * @param ax Pointer to X-axis output
 * @param ay Pointer to Y-axis output
 * @param az Pointer to Z-axis output
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mpu6050_driver_read_data(float *ax, float *ay, float *az)
{
    esp_err_t res = mpu6050_get_acceleration(&mpu, &accel);
    if (res != ESP_OK)
        return res;
    *ax = accel.x;
    *ay = accel.y;
    *az = accel.z;
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*                          Earthquake threat evaluation                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief Evaluate earthquake threat based on P-wave energy
 *
 * Converts energy to dB, applies smoothing, and thresholds for:
 *  - EQ_THREAT_NONE
 *  - EQ_THREAT_LOW
 *  - EQ_THREAT_MEDIUM
 *  - EQ_THREAT_HIGH
 *
 * @param E Energy computed from FFT
 * @return Evaluated earthquake threat level
 */
eq_threat_level evaluate_threat_eq(float E)
{
    eq_threat_level level;
    float db = 10 * log10f(E + 1e-9f);
    static float y = 0;
    
    y = 0.75f * y + 0.25f * db;
    /* ----------------------------------------------------------------------
    * Exponential Moving Average (EMA) smoothing of P-wave energy in dB
    *
    * y = 0.75*y + 0.25*db
    *
    * - Purpose: Smooths out short-term fluctuations in P-wave energy readings
    *   from the MPU6050 to prevent false alarm triggering.
    * - α = 0.25 (new sample weight): Provides a balance between responsiveness
    *   and noise suppression.
    * - Previous value weight = 0.75: Retains historical trend to detect
    *   sustained energy increases.
    *
    * Note: Adjust α to tune sensitivity:
    *   Higher α (e.g., 0.3–0.5) → more responsive but more noise-prone
    *   Lower α (e.g., 0.1–0.2) → smoother but slower to react
    * ------------------------------------------------------------------- */

    if (y > 5)
        level = EQ_THREAT_HIGH;
    else if (y > 2)
        level = EQ_THREAT_MEDIUM;
    else if (y > -5)
        level = EQ_THREAT_LOW;
    else
        level = EQ_THREAT_NONE;

    // Optional debug print
    // printf("E=%.3f  db=%.2f  LEVEL=%d\n", E, y, level);

    return level;
}