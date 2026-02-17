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

#define I2C_NUM I2C_NUM_0

static const char *TAG = "MPU6050_driver";

float ax[N], ay[N], az[N];   // ring buffers
float mag[N];
float filtered[N];
float fft_buf[2*N];
float hann[N];

int write_idx = 0;
static float iir_x1 = 0, iir_x2 = 0, iir_y1 = 0, iir_y2 = 0;

static mpu6050_dev_t mpu = {0};
static mpu6050_acceleration_t accel;

void unwrap(float *circ, float *lin)
{
    int idx = write_idx;
    for(int i=0;i<N;i++)
        lin[i] = circ[(idx + i) % N];
}


void compute_magnitude(float *x, float *y, float *z, float *m)
{
    for(int i=0;i<N;i++)
        m[i] = sqrtf(x[i]*x[i] + y[i]*y[i] + z[i]*z[i]);
}


void remove_mean(float *x)
{
    float sum = 0;
    for(int i=0;i<N;i++) sum += x[i];
    float mean = sum / N;
    for(int i=0;i<N;i++) x[i] -= mean;
}


void bandpass(float *x, float *y)
{
    for(int i=0;i<N;i++)
    {
        y[i] = B0*x[i] + B1*iir_x1 + B2*iir_x2 - A1*iir_y1 - A2*iir_y2;
        iir_x2 = iir_x1; iir_x1 = x[i];
        iir_y2 = iir_y1; iir_y1 = y[i];
    }
}



void apply_hann(float *x)
{
    for(int i=0;i<N;i++)
        x[i] *= hann[i];
}

void compute_fft(float *x)
{
    for(int i=0;i<N;i++){
        fft_buf[2*i]   = x[i];
        fft_buf[2*i+1] = 0;
    }

    dsps_fft2r_fc32(fft_buf, N);
    dsps_bit_rev_fc32(fft_buf, N);
    dsps_cplx2reC_fc32(fft_buf, N);
}


float compute_energy()
{
    float E = 0;

    for(int k = BIN_F1; k <= BIN_F2; k++)
    {
        float re = fft_buf[2*k];
        float im = fft_buf[2*k+1];
        E += re*re + im*im;
    }

    return E / N;
}


void mpu_dsp_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    int hop = 0;

    static float ax_lin[N], ay_lin[N], az_lin[N];

    while (1)
    {
        /* ---- SAMPLE ---- */
        float x,y,z;
        mpu6050_driver_read_data(&x,&y,&z);

        ax[write_idx] = x;
        ay[write_idx] = y;
        az[write_idx] = z;
        write_idx = (write_idx + 1) % N;

        /* ---- DSP every STEP samples ---- */
        if (++hop >= STEP)
        {
            hop = 0;
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
            eq_threat_level lvl = evaluate_threat_eq(E);

//            printf("E=%.3f  LEVEL=%d\n", E, lvl);
        }

        /* ---- TIMING ---- */
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000/FS));
    }
}

void mpu6050_driver_init() {

    ESP_ERROR_CHECK(i2cdev_init());
    ESP_ERROR_CHECK(
        mpu6050_init_desc(&mpu, MPU6050_ADDR, I2C_NUM, MPU6050_SDA_PIN, MPU6050_SCL_PIN)
    );
    ESP_ERROR_CHECK(mpu6050_init(&mpu));

    ESP_LOGI(TAG, "initialized device.");

    dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    dsps_wind_hann_f32(hann, N);
}

esp_err_t mpu6050_driver_read_data(float *ax, float *ay, float *az) {
    esp_err_t res = mpu6050_get_acceleration(&mpu, &accel);
    if (res != ESP_OK) return res;
    *ax = accel.x;
    *ay = accel.y;
    *az = accel.z;
    return ESP_OK;
}

eq_threat_level evaluate_threat_eq(float E)
{
    eq_threat_level level;
    float db = 10*log10f(E+1e-9f);
    static float y=0;
    y = 0.9f*y + 0.1f*db;

    if (y > 5)       level = EQ_THREAT_HIGH;
    else if (y > 2)  level = EQ_THREAT_MEDIUM;
    else if (y > -5) level = EQ_THREAT_LOW;
    else             level = EQ_THREAT_NONE;

//    printf("E=%.3f  db=%.2f  LEVEL=%d\n", E, y, level);
    return level;
}
