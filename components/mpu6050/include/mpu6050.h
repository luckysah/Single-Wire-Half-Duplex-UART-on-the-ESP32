/*
 * Minimal MPU6050 (GY-521) driver on the ESP-IDF i2c_master API.
 * Configured for balancing: +/-2 g, +/-250 deg/s, DLPF 44 Hz, 1 kHz sample rate.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_ADDR_AD0_LOW    0x68
#define MPU6050_ADDR_AD0_HIGH   0x69
#define MPU6050_WHO_AM_I_VALUE  0x68

typedef struct {
    i2c_master_dev_handle_t dev;
} mpu6050_t;

typedef struct {
    float ax, ay, az;   /* g */
    float gx, gy, gz;   /* deg/s */
    float temp_c;
} mpu6050_sample_t;

esp_err_t mpu6050_attach(i2c_master_bus_handle_t bus, uint8_t addr, uint32_t scl_hz, mpu6050_t *out);
esp_err_t mpu6050_detach(mpu6050_t *m);
esp_err_t mpu6050_who_am_i(mpu6050_t *m, uint8_t *id);
/* Wake the sensor and apply the balancing configuration above. */
esp_err_t mpu6050_configure(mpu6050_t *m);
esp_err_t mpu6050_read(mpu6050_t *m, mpu6050_sample_t *s);
esp_err_t mpu6050_read_reg(mpu6050_t *m, uint8_t reg, uint8_t *val);

#ifdef __cplusplus
}
#endif
