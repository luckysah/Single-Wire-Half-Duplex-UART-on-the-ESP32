/*
 * Minimal DYNAMIXEL Protocol 2.0 master, single-wire (open-drain TX + echo cancel).
 * Packet: FF FF FD 00 | ID | LEN_L LEN_H | INST | PARAMS... | CRC_L CRC_H
 *   LEN = bytes after the length field (INST + PARAMS + CRC), after byte stuffing.
 *   Byte stuffing: FF FF FD inside INST..PARAMS is sent as FF FF FD FD.
 *   CRC-16 (poly 0x8005, init 0, not reflected) over everything before the CRC.
 */
#include <string.h>
#include "dxl.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define INST_PING       0x01
#define INST_READ       0x02
#define INST_WRITE      0x03
#define INST_REBOOT     0x08
#define INST_SYNC_READ  0x82
#define INST_SYNC_WRITE 0x83
#define INST_STATUS     0x55

#define DXL_MAX_PACKET  256
#define ECHO_TIMEOUT_MS 5

static const char *TAG = "dxl";
static dxl_config_t s_cfg;
static dxl_stats_t s_stats;
static bool s_ready;
static int64_t s_timeout_us;

static uint16_t crc16(uint16_t crc, const uint8_t *d, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x8005) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static TickType_t ticks_until(int64_t deadline_us)
{
    int64_t rem = deadline_us - esp_timer_get_time();
    if (rem <= 0) {
        return 0;
    }
    TickType_t t = pdMS_TO_TICKS((rem + 999) / 1000);
    return t ? t : 1;
}

/* Short waits (< 2 ms) spin instead of sleeping a whole RTOS tick, so a missing reply costs
 * only the configured timeout. */
static bool read_exact(uint8_t *buf, size_t n, int64_t deadline_us)
{
    size_t got = 0;
    while (got < n) {
        size_t avail = 0;
        uart_get_buffered_data_len(s_cfg.port, &avail);
        if (avail) {
            int r = uart_read_bytes(s_cfg.port, buf + got, avail < n - got ? avail : n - got, 0);
            if (r > 0) {
                got += (size_t)r;
            }
            continue;
        }
        int64_t rem = deadline_us - esp_timer_get_time();
        if (rem <= 0) {
            return false;
        }
        if (rem > 2000) {
            int r = uart_read_bytes(s_cfg.port, buf + got, n - got, ticks_until(deadline_us - 1000));
            if (r > 0) {
                got += (size_t)r;
            }
        } else {
            esp_rom_delay_us(10);
        }
    }
    return true;
}

/* Build an instruction packet with byte stuffing; returns its length (0 if it does not fit). */
static size_t build_packet(uint8_t *out, size_t cap, uint8_t id, uint8_t inst,
                           const uint8_t *params, size_t plen)
{
    if (cap < 10 + plen * 2) {
        return 0;
    }
    size_t n = 0;
    out[n++] = 0xFF; out[n++] = 0xFF; out[n++] = 0xFD; out[n++] = 0x00;
    out[n++] = id;
    n += 2;                                 /* length, filled below */
    out[n++] = inst;
    for (size_t i = 0; i < plen; i++) {
        out[n++] = params[i];
        if (n - 3 >= 7 && out[n - 3] == 0xFF && out[n - 2] == 0xFF && out[n - 1] == 0xFD) {
            out[n++] = 0xFD;
        }
    }
    uint16_t len = (uint16_t)(n - 7 + 2);   /* INST..PARAMS + CRC */
    out[5] = len & 0xFF;
    out[6] = len >> 8;
    uint16_t crc = crc16(0, out, n);
    out[n++] = crc & 0xFF;
    out[n++] = crc >> 8;
    return n;
}

/* Wait for a status packet from `id`; copies its parameters (max `cap` bytes) to `params`. */
static esp_err_t rx_status(uint8_t id, uint8_t *params, size_t cap, size_t *plen, uint8_t *dxl_err)
{
    int64_t deadline = esp_timer_get_time() + s_timeout_us;
    uint8_t pkt[DXL_MAX_PACKET];
    uint32_t hdr = 0;

    while (hdr != 0xFFFFFD00) {
        uint8_t b;
        if (!read_exact(&b, 1, deadline)) {
            s_stats.timeouts++;
            return ESP_ERR_TIMEOUT;
        }
        hdr = (hdr << 8) | b;
    }
    pkt[0] = 0xFF; pkt[1] = 0xFF; pkt[2] = 0xFD; pkt[3] = 0x00;
    if (!read_exact(&pkt[4], 3, deadline)) {
        s_stats.timeouts++;
        return ESP_ERR_TIMEOUT;
    }
    uint16_t len = pkt[5] | (pkt[6] << 8);
    if (len < 4 || 7 + (size_t)len > sizeof(pkt)) {
        s_stats.bad_packets++;
        return ESP_ERR_INVALID_SIZE;
    }
    if (!read_exact(&pkt[7], len, deadline)) {
        s_stats.timeouts++;
        return ESP_ERR_TIMEOUT;
    }
    size_t total = 7 + len;
    uint16_t crc_rx = pkt[total - 2] | (pkt[total - 1] << 8);
    if (crc16(0, pkt, total - 2) != crc_rx) {
        s_stats.crc_errors++;
        return ESP_ERR_INVALID_CRC;
    }

    /* Remove byte stuffing from INST..PARAMS */
    uint8_t *body = &pkt[7];
    size_t blen = len - 2, o = 0;
    for (size_t i = 0; i < blen; i++) {
        body[o++] = body[i];
        if (o >= 3 && body[o - 3] == 0xFF && body[o - 2] == 0xFF && body[o - 1] == 0xFD
            && i + 1 < blen && body[i + 1] == 0xFD) {
            i++;
        }
    }
    if (o < 2 || body[0] != INST_STATUS || (id != DXL_BROADCAST_ID && pkt[4] != id)) {
        s_stats.bad_packets++;
        return ESP_ERR_INVALID_SIZE;
    }
    size_t n = o - 2;
    if (n > cap) {
        s_stats.bad_packets++;
        return ESP_ERR_INVALID_SIZE;
    }
    if (params && n) {
        memcpy(params, &body[2], n);
    }
    if (plen) {
        *plen = n;
    }
    if (dxl_err) {
        *dxl_err = body[1];
    }
    return ESP_OK;
}

/* Send one instruction and verify and drop its echo. */
static esp_err_t send_inst(uint8_t id, uint8_t inst, const uint8_t *params, size_t plen)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t tx[DXL_MAX_PACKET], echo[DXL_MAX_PACKET];
    size_t n = build_packet(tx, sizeof(tx), id, inst, params, plen);
    if (n == 0) {
        return ESP_ERR_INVALID_SIZE;
    }

    uart_flush_input(s_cfg.port);
    s_stats.packets++;
    uart_write_bytes(s_cfg.port, tx, n);
    uart_wait_tx_done(s_cfg.port, pdMS_TO_TICKS(100));

    int got = uart_read_bytes(s_cfg.port, echo, n, pdMS_TO_TICKS(ECHO_TIMEOUT_MS));
    if (got != (int)n || memcmp(echo, tx, n) != 0) {
        s_stats.echo_errors++;
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

/* Send one instruction, then (unless it is a broadcast write) receive the reply. */
static esp_err_t transact(uint8_t id, uint8_t inst, const uint8_t *params, size_t plen,
                          uint8_t *rparams, size_t rcap, size_t *rlen, uint8_t *dxl_err)
{
    esp_err_t e = send_inst(id, inst, params, plen);
    if (e != ESP_OK) {
        return e;
    }
    if (id == DXL_BROADCAST_ID && inst != INST_PING) {
        return ESP_OK;  /* broadcast write: motors do not reply */
    }
    return rx_status(id, rparams, rcap, rlen, dxl_err);
}

esp_err_t dxl_init(const dxl_config_t *cfg)
{
    if (s_ready) {
        dxl_deinit();
    }
    s_cfg = *cfg;
    s_timeout_us = (int64_t)cfg->reply_timeout_ms * 1000;
    uart_config_t uc = {
        .baud_rate = (int)cfg->baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(cfg->port, 1024, 0, 0, NULL, 0), TAG, "driver install");
    ESP_RETURN_ON_ERROR(uart_param_config(cfg->port, &uc), TAG, "param config");
    ESP_RETURN_ON_ERROR(uart_set_pin(cfg->port, cfg->tx_gpio, cfg->rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "set pin");
    /* Release the wire when idle; weak internal pull-ups back up the level shifter's. */
    gpio_od_enable(cfg->tx_gpio);
    gpio_pullup_en(cfg->tx_gpio);
    gpio_pullup_en(cfg->rx_gpio);
    /* Hand received bytes to the driver after 2 idle symbols instead of the default 10. */
    uart_set_rx_timeout(cfg->port, 2);
    s_ready = true;
    return ESP_OK;
}

esp_err_t dxl_deinit(void)
{
    if (!s_ready) {
        return ESP_OK;
    }
    s_ready = false;
    return uart_driver_delete(s_cfg.port);
}

esp_err_t dxl_set_baud(uint32_t baud)
{
    s_cfg.baud = baud;
    return uart_set_baudrate(s_cfg.port, baud);
}

void dxl_set_reply_timeout(int ms)
{
    s_cfg.reply_timeout_ms = ms;
    s_timeout_us = (int64_t)ms * 1000;
}

void dxl_set_reply_timeout_us(int us)
{
    s_timeout_us = us;
}

esp_err_t dxl_ping(uint8_t id, uint16_t *model, uint8_t *fw, uint8_t *dxl_err)
{
    uint8_t r[3];
    size_t rlen = 0;
    esp_err_t e = transact(id, INST_PING, NULL, 0, r, sizeof(r), &rlen, dxl_err);
    if (e == ESP_OK && rlen == 3) {
        if (model) {
            *model = r[0] | (r[1] << 8);
        }
        if (fw) {
            *fw = r[2];
        }
    } else if (e == ESP_OK) {
        e = ESP_ERR_INVALID_SIZE;
    }
    return e;
}

esp_err_t dxl_read(uint8_t id, uint16_t addr, void *data, uint16_t len, uint8_t *dxl_err)
{
    uint8_t p[4] = { addr & 0xFF, addr >> 8, len & 0xFF, len >> 8 };
    size_t rlen = 0;
    esp_err_t e = transact(id, INST_READ, p, sizeof(p), data, len, &rlen, dxl_err);
    if (e == ESP_OK && rlen != len) {
        e = ESP_ERR_INVALID_SIZE;
    }
    return e;
}

esp_err_t dxl_write(uint8_t id, uint16_t addr, const void *data, uint16_t len, uint8_t *dxl_err)
{
    uint8_t p[2 + 64];
    if (len > 64) {
        return ESP_ERR_INVALID_SIZE;
    }
    p[0] = addr & 0xFF;
    p[1] = addr >> 8;
    memcpy(&p[2], data, len);
    return transact(id, INST_WRITE, p, 2 + len, NULL, 0, NULL, dxl_err);
}

esp_err_t dxl_reboot(uint8_t id, uint8_t *dxl_err)
{
    return transact(id, INST_REBOOT, NULL, 0, NULL, 0, NULL, dxl_err);
}

int dxl_debug_ping_raw(uint8_t id, uint8_t *raw, size_t cap, uint32_t *edges)
{
    if (send_inst(id, INST_PING, NULL, 0) != ESP_OK) {
        return -1;
    }
    /* Watch the RX pin for 2 ms: every level change means someone is driving the wire. */
    int pin = s_cfg.rx_gpio;
    uint32_t n = 0;
    int last = gpio_get_level(pin);
    int64_t end = esp_timer_get_time() + 2000;
    while (esp_timer_get_time() < end) {
        int l = gpio_get_level(pin);
        if (l != last) {
            n++;
            last = l;
        }
    }
    *edges = n;
    vTaskDelay(1);
    int got = uart_read_bytes(s_cfg.port, raw, cap, 0);
    return got < 0 ? 0 : got;
}

esp_err_t dxl_sync_write(uint16_t addr, uint16_t len, const uint8_t *ids, int n, const void *data)
{
    uint8_t p[DXL_MAX_PACKET / 2];
    size_t plen = 4 + (size_t)n * (1 + len);
    if (plen > sizeof(p)) {
        return ESP_ERR_INVALID_SIZE;
    }
    p[0] = addr & 0xFF; p[1] = addr >> 8; p[2] = len & 0xFF; p[3] = len >> 8;
    uint8_t *o = &p[4];
    for (int i = 0; i < n; i++) {
        *o++ = ids[i];
        memcpy(o, (const uint8_t *)data + (size_t)i * len, len);
        o += len;
    }
    return send_inst(DXL_BROADCAST_ID, INST_SYNC_WRITE, p, plen);
}

esp_err_t dxl_sync_read(uint16_t addr, uint16_t len, const uint8_t *ids, int n, void *data, uint8_t *dxl_errs)
{
    uint8_t p[4 + 32];
    if (n > 32) {
        return ESP_ERR_INVALID_SIZE;
    }
    p[0] = addr & 0xFF; p[1] = addr >> 8; p[2] = len & 0xFF; p[3] = len >> 8;
    memcpy(&p[4], ids, n);
    esp_err_t e = send_inst(DXL_BROADCAST_ID, INST_SYNC_READ, p, 4 + n);
    /* Each motor answers with its own status packet, in the order of `ids`. */
    for (int i = 0; e == ESP_OK && i < n; i++) {
        size_t rlen = 0;
        uint8_t err = 0;
        e = rx_status(ids[i], (uint8_t *)data + (size_t)i * len, len, &rlen, &err);
        if (e == ESP_OK && rlen != len) {
            e = ESP_ERR_INVALID_SIZE;
        }
        if (dxl_errs) {
            dxl_errs[i] = err;
        }
    }
    return e;
}

static esp_err_t check_status(esp_err_t e, uint8_t dxl_err)
{
    return (e == ESP_OK && (dxl_err & 0x7F)) ? ESP_FAIL : e;
}

esp_err_t dxl_read_u8(uint8_t id, uint16_t addr, uint8_t *v)
{
    uint8_t err = 0;
    return check_status(dxl_read(id, addr, v, 1, &err), err);
}

esp_err_t dxl_read_u16(uint8_t id, uint16_t addr, uint16_t *v)
{
    uint8_t b[2], err = 0;
    esp_err_t e = check_status(dxl_read(id, addr, b, 2, &err), err);
    if (e == ESP_OK) {
        *v = b[0] | (b[1] << 8);
    }
    return e;
}

esp_err_t dxl_read_u32(uint8_t id, uint16_t addr, uint32_t *v)
{
    uint8_t b[4], err = 0;
    esp_err_t e = check_status(dxl_read(id, addr, b, 4, &err), err);
    if (e == ESP_OK) {
        *v = b[0] | (b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
    }
    return e;
}

esp_err_t dxl_write_u8(uint8_t id, uint16_t addr, uint8_t v)
{
    uint8_t err = 0;
    return check_status(dxl_write(id, addr, &v, 1, &err), err);
}

esp_err_t dxl_write_u16(uint8_t id, uint16_t addr, uint16_t v)
{
    uint8_t b[2] = { v & 0xFF, v >> 8 }, err = 0;
    return check_status(dxl_write(id, addr, b, 2, &err), err);
}

esp_err_t dxl_write_u32(uint8_t id, uint16_t addr, uint32_t v)
{
    uint8_t b[4] = { v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, v >> 24 }, err = 0;
    return check_status(dxl_write(id, addr, b, 4, &err), err);
}

const dxl_stats_t *dxl_get_stats(void)
{
    return &s_stats;
}

void dxl_reset_stats(void)
{
    memset(&s_stats, 0, sizeof(s_stats));
}

uint32_t dxl_baud_from_reg(uint8_t reg)
{
    static const uint32_t map[] = { 9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000, 4500000 };
    return reg < sizeof(map) / sizeof(map[0]) ? map[reg] : 0;
}
