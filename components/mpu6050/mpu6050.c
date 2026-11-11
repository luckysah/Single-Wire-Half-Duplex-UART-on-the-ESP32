#include "mpu6050.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75

#define ACCEL_LSB_PER_G     16384.0f    /* +/-2 g */
#define GYRO_LSB_PER_DPS    131.0f      /* +/-250 deg/s */
#define XFER_TIMEOUT_MS     20

static esp_err_t write_reg(mpu6050_t *m, uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return i2c_master_transmit(m->dev, b, sizeof(b), XFER_TIMEOUT_MS);
}

esp_err_t mpu6050_read_reg(mpu6050_t *m, uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(m->dev, &reg, 1, val, 1, XFER_TIMEOUT_MS);
}

esp_err_t mpu6050_attach(i2c_master_bus_handle_t bus, uint8_t addr, uint32_t scl_hz, mpu6050_t *out)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = scl_hz,
    };
    return i2c_master_bus_add_device(bus, &cfg, &out->dev);
}

esp_err_t mpu6050_detach(mpu6050_t *m)
{
    esp_err_t e = i2c_master_bus_rm_device(m->dev);
    m->dev = NULL;
    return e;
}

esp_err_t mpu6050_who_am_i(mpu6050_t *m, uint8_t *id)
{
    return mpu6050_read_reg(m, REG_WHO_AM_I, id);
}

esp_err_t mpu6050_configure(mpu6050_t *m)
{
    esp_err_t e;
    if ((e = write_reg(m, REG_PWR_MGMT_1, 0x80)) != ESP_OK) {   /* device reset */
        return e;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    if ((e = write_reg(m, REG_PWR_MGMT_1, 0x01)) != ESP_OK) {   /* wake, clock = gyro X PLL */
        return e;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    if ((e = write_reg(m, REG_CONFIG, 0x03)) != ESP_OK) {       /* DLPF 44 Hz -> 1 kHz internal rate */
        return e;
    }
    if ((e = write_reg(m, REG_SMPLRT_DIV, 0x00)) != ESP_OK) {   /* 1 kHz output */
        return e;
    }
    if ((e = write_reg(m, REG_GYRO_CONFIG, 0x00)) != ESP_OK) {  /* +/-250 deg/s */
        return e;
    }
    e = write_reg(m, REG_ACCEL_CONFIG, 0x00);                    /* +/-2 g */
    vTaskDelay(pdMS_TO_TICKS(50));
    return e;
}

esp_err_t mpu6050_read(mpu6050_t *m, mpu6050_sample_t *s)
{
    uint8_t reg = REG_ACCEL_XOUT_H, b[14];
    esp_err_t e = i2c_master_transmit_receive(m->dev, &reg, 1, b, sizeof(b), XFER_TIMEOUT_MS);
    if (e != ESP_OK) {
        return e;
    }
    int16_t raw[7];
    for (int i = 0; i < 7; i++) {
        raw[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
    }
    s->ax = raw[0] / ACCEL_LSB_PER_G;
    s->ay = raw[1] / ACCEL_LSB_PER_G;
    s->az = raw[2] / ACCEL_LSB_PER_G;
    s->temp_c = raw[3] / 340.0f + 36.53f;
    s->gx = raw[4] / GYRO_LSB_PER_DPS;
    s->gy = raw[5] / GYRO_LSB_PER_DPS;
    s->gz = raw[6] / GYRO_LSB_PER_DPS;
    return ESP_OK;
}
