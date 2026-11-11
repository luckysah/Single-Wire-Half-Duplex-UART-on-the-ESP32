/*
 * Minimal DYNAMIXEL Protocol 2.0 master for ESP-IDF, single-wire bus.
 *
 * Hardware (see Circuit/TwoWheelRobot_Schematic_Simple.png):
 *   ESP32 TX and RX pins are jumpered together and go through a BSS138 level
 *   shifter to the DYNAMIXEL DATA line. TX is switched to open-drain so the
 *   ESP32 releases the wire when idle; every byte we send is therefore echoed
 *   back on RX, and the driver checks and discards that echo.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DXL_BROADCAST_ID    0xFE

/* XL430-W250 control table (address, size in bytes) */
#define XL430_MODEL_NUMBER          0   /* 2, R  */
#define XL430_FIRMWARE_VERSION      6   /* 1, R  */
#define XL430_ID                    7   /* 1, RW */
#define XL430_BAUD_RATE             8   /* 1, RW 0=9600 1=57600 2=115200 3=1M 4=2M 5=3M 6=4M 7=4.5M */
#define XL430_RETURN_DELAY_TIME     9   /* 1, RW 2 us units */
#define XL430_DRIVE_MODE            10  /* 1, RW */
#define XL430_OPERATING_MODE        11  /* 1, RW 1=velocity 3=position 4=ext. position 16=PWM */
#define XL430_PROTOCOL_TYPE         13  /* 1, RW */
#define XL430_TEMPERATURE_LIMIT     31  /* 1, RW degC */
#define XL430_MAX_VOLTAGE_LIMIT     32  /* 2, RW 0.1 V */
#define XL430_MIN_VOLTAGE_LIMIT     34  /* 2, RW 0.1 V */
#define XL430_VELOCITY_LIMIT        44  /* 4, RW 0.229 rpm */
#define XL430_SHUTDOWN              63  /* 1, RW */
#define XL430_TORQUE_ENABLE         64  /* 1, RW */
#define XL430_LED                   65  /* 1, RW */
#define XL430_STATUS_RETURN_LEVEL   68  /* 1, RW */
#define XL430_HARDWARE_ERROR_STATUS 70  /* 1, R  */
#define XL430_VELOCITY_I_GAIN       76  /* 2, RW default 1920 */
#define XL430_VELOCITY_P_GAIN       78  /* 2, RW default 100 */
#define XL430_PROFILE_ACCELERATION  108 /* 4, RW 214.577 rev/min^2, 0 = no ramp */
#define XL430_GOAL_PWM              100 /* 2, RW +/-885 = +/-100 % (operating mode 16) */
#define XL430_GOAL_VELOCITY         104 /* 4, RW 0.229 rpm */
#define XL430_PRESENT_LOAD          126 /* 2, R  0.1 % */
#define XL430_PRESENT_VELOCITY      128 /* 4, R  0.229 rpm */
#define XL430_PRESENT_POSITION      132 /* 4, R  pulse (4096 / rev) */
#define XL430_PRESENT_INPUT_VOLTAGE 144 /* 2, R  0.1 V */
#define XL430_PRESENT_TEMPERATURE   146 /* 1, R  degC */

#define XL430_MODEL_NUMBER_VALUE    1060
#define XL430_RPM_PER_UNIT          0.229f

typedef struct {
    uart_port_t port;
    int tx_gpio;
    int rx_gpio;
    uint32_t baud;
    int reply_timeout_ms;   /* how long to wait for a status packet */
} dxl_config_t;

typedef struct {
    uint32_t packets;       /* instruction packets sent */
    uint32_t echo_errors;   /* our own bytes did not come back intact */
    uint32_t timeouts;      /* no (complete) status packet */
    uint32_t crc_errors;
    uint32_t bad_packets;   /* wrong ID / instruction / length */
} dxl_stats_t;

/*
 * Return codes used by all calls:
 *   ESP_OK                    status packet received (check *dxl_err for the motor's error byte)
 *   ESP_ERR_INVALID_RESPONSE  echo of our own packet missing or corrupted (wiring / TX-RX jumper)
 *   ESP_ERR_TIMEOUT           echo OK, but no reply from the motor
 *   ESP_ERR_INVALID_CRC       reply arrived with a bad checksum
 *   ESP_ERR_INVALID_SIZE      reply malformed or longer than the buffer
 */
esp_err_t dxl_init(const dxl_config_t *cfg);
esp_err_t dxl_deinit(void);
esp_err_t dxl_set_baud(uint32_t baud);
void dxl_set_reply_timeout(int ms);
/* Finer timeout for fast loops: at 1 Mbps a reply normally starts within ~100 us. */
void dxl_set_reply_timeout_us(int us);

esp_err_t dxl_ping(uint8_t id, uint16_t *model, uint8_t *fw, uint8_t *dxl_err);
esp_err_t dxl_read(uint8_t id, uint16_t addr, void *data, uint16_t len, uint8_t *dxl_err);
esp_err_t dxl_write(uint8_t id, uint16_t addr, const void *data, uint16_t len, uint8_t *dxl_err);
/* Restart the motor (clears a latched Hardware Error Status). Allow ~1 s before talking to it again. */
esp_err_t dxl_reboot(uint8_t id, uint8_t *dxl_err);

/* One packet for several motors: `data` holds n blocks of `len` bytes, in the order of `ids`. */
esp_err_t dxl_sync_write(uint16_t addr, uint16_t len, const uint8_t *ids, int n, const void *data);
/* One request, n replies; `data` receives n blocks of `len` bytes, `dxl_errs` (optional) n error bytes. */
esp_err_t dxl_sync_read(uint16_t addr, uint16_t len, const uint8_t *ids, int n, void *data, uint8_t *dxl_errs);

/* Little-endian convenience wrappers */
esp_err_t dxl_read_u8(uint8_t id, uint16_t addr, uint8_t *v);
esp_err_t dxl_read_u16(uint8_t id, uint16_t addr, uint16_t *v);
esp_err_t dxl_read_u32(uint8_t id, uint16_t addr, uint32_t *v);
esp_err_t dxl_write_u8(uint8_t id, uint16_t addr, uint8_t v);
esp_err_t dxl_write_u16(uint8_t id, uint16_t addr, uint16_t v);
esp_err_t dxl_write_u32(uint8_t id, uint16_t addr, uint32_t v);

/* Debug: ping `id`, then report the raw bytes received (no parsing) and how many level changes
 * the RX pin saw during the following 2 ms. Returns the byte count, or -1 if our echo failed. */
int dxl_debug_ping_raw(uint8_t id, uint8_t *raw, size_t cap, uint32_t *edges);

const dxl_stats_t *dxl_get_stats(void);
void dxl_reset_stats(void);

/* Map the XL430 Baud Rate register value (0..7) to bps; 0 if unknown. */
uint32_t dxl_baud_from_reg(uint8_t reg);

#ifdef __cplusplus
}
#endif
