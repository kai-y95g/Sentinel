#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"
#include "mpu6050_driver.h"
#include "i2cdev.h"
#include "esp_log.h"
#include "esp_dsp.h"
#include "system_config.h"

#define I2C_NUM I2C_NUM_0

static const char *TAG = "MPU6050_driver";

static gravity_t grav;
static float grav_alpha;
static float alpha_sta;
static float alpha_lta;
static float sta;
static float lta;
static bool triggered;
static int event_count = 0;
mpu6050_dev_t mpu = {0};
mpu6050_acceleration_t accel;

static biquad_t bp = {
    .b0 =  0.067455,
    .b1 =  0.000000,
    .b2 = -0.067455,
    .a1 = -1.14298,
    .a2 =  0.86509,
};


float biquad(biquad_t *f, float x)
{
    float y = f->b0*x + f->z1;
    f->z1 = f->b1*x - f->a1*y + f->z2;
    f->z2 = f->b2*x - f->a2*y;
    return y;
}


void sta_lta_init(float fs)
{
    grav_alpha = expf(-1.0f / (GRAV_TAU * fs));
    alpha_sta  = expf(-1.0f / (STA_SEC * fs));
    alpha_lta  = expf(-1.0f / (LTA_SEC * fs));

    // reset all state
    grav.gx = grav.gy = grav.gz = 0;
    sta = 0;
    lta = 1e-6;
    bp.z1 = bp.z2 = 0;
    triggered = false;
}


void mpu6050_driver_init() 
{
    ESP_ERROR_CHECK(i2cdev_init());
    ESP_ERROR_CHECK(
        mpu6050_init_desc(&mpu, MPU6050_ADDR, I2C_NUM, MPU6050_SDA_PIN, MPU6050_SCL_PIN)
    );
    ESP_ERROR_CHECK(mpu6050_init(&mpu));

    ESP_LOGI(TAG, "initialized device.");
    mpu6050_set_full_scale_accel_range(&mpu, MPU6050_ACCEL_RANGE_2);
    
    sta_lta_init(FS);
}


void remove_gravity(mpu6050_acceleration_t *accel)
{
    grav.gx = grav_alpha * grav.gx + (1 - grav_alpha) * (accel->x);
    grav.gy = grav_alpha * grav.gy + (1 - grav_alpha) * (accel->y);
    grav.gz = grav_alpha * grav.gz + (1 - grav_alpha) * (accel->z);

    accel->x -= grav.gx;
    accel->y -= grav.gy;
    accel->z -= grav.gz;
}


float update_sta_lta(float x)
{
    float e = x * x;   // energy
    //printf("Eneregy :- %.2f\n",e);

    sta = alpha_sta * sta + (1 - alpha_sta) * e;
    lta = alpha_lta * lta + (1 - alpha_lta) * e;

    //return sta / lta;
    float res = sta/lta;
    //printf("STA/LTA %.2f\n",res);
    return res;
}

esp_err_t mpu6050_driver_read_data(mpu6050_data_t *data)
{
    esp_err_t res = mpu6050_get_acceleration(&mpu, &accel);
    if (res != ESP_OK) return res;
    remove_gravity(&accel);
    data->ax = accel.x;
    data->ay = accel.y;
    data->az = accel.z;
    data->magnitude = sqrtf(accel.x * accel.x + 
                        accel.y * accel.y + 
                        accel.z * accel.z);
    return ESP_OK;
}


void imu_task(void *arg)
{
    mpu6050_data_t data;
    static int warmup = 0;

    while (1) {
        mpu6050_driver_read_data(&data);

        if (warmup < FS) {
            warmup++;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        float y = biquad(&bp, data.magnitude);
        float r = update_sta_lta(y);
        if (r > TRIG_ON) {
            event_count++;
            printf("STA/LTA %.2f\n",r);
            if (!triggered && event_count > MIN_EVENT_SAMPLES) {
                triggered = true;
                printf("EVENT ON\n");
            }
        } else {
            event_count = 0;
        }

        if (triggered && (r < TRIG_OFF)) {
            printf("Reached here 2\n");
            printf("STA/LTA %.2f\n",r);
            triggered = false;
            printf("EVENT OFF\n");
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
