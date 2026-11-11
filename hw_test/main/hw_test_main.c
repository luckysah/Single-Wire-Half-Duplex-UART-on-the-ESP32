/*
 * Two-wheel balancing robot: hardware bring-up test.
 *
 * Runs once after reset and prints a report on the USB console:
 *   1. ESP32-S3 board details
 *   2. I2C bus scan (finds the MPU6050 even if SDA/SCL are on other pins)
 *   3. MPU6050: identity, noise, gyro bias, orientation, live tilt readout
 *   4. DYNAMIXEL bus: idle level, TX->RX echo (jumper + open-drain)
 *   5. Motor search (configured baud first, then all bauds / IDs 0-20)
 *   6. Motor details (control table) and battery voltage
 *   7. Bus reliability (repeated ping + read)
 *   8. LED and slow spin test (optional, wheels must be free)
 *   9. Summary
 */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_idf_version.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "dxl.h"
#include "mpu6050.h"

#define PIN_SDA     CONFIG_HWT_I2C_SDA
#define PIN_SCL     CONFIG_HWT_I2C_SCL
#define PIN_DXL_TX  CONFIG_HWT_DXL_TX
#define PIN_DXL_RX  CONFIG_HWT_DXL_RX
#define DXL_UART    UART_NUM_1
#define RAD2DEG     57.29578f

/* ------------------------------------------------------------------ report helpers */
typedef enum { PASS, WARN, FAIL, SKIP } verdict_t;
static const char *const VSTR[] = { "PASS", "WARN", "FAIL", "SKIP" };

typedef struct {
    char name[28];
    verdict_t v;
    char note[110];
} result_t;

static result_t s_res[32];
static int s_nres;

static void result(const char *name, verdict_t v, const char *fmt, ...)
{
    if (s_nres >= (int)(sizeof(s_res) / sizeof(s_res[0]))) {
        return;
    }
    result_t *r = &s_res[s_nres++];
    snprintf(r->name, sizeof(r->name), "%s", name);
    r->v = v;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(r->note, sizeof(r->note), fmt, ap);
    va_end(ap);
    printf("  -> [%s] %s: %s\n", VSTR[v], name, r->note);
}

static void section(const char *title)
{
    printf("\n===== %s =====\n", title);
}

/* ------------------------------------------------------------------ 1. board */
static const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON:  return "power-on";
    case ESP_RST_EXT:      return "external pin";
    case ESP_RST_SW:       return "software";
    case ESP_RST_PANIC:    return "panic/crash";
    case ESP_RST_INT_WDT:  return "interrupt watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_WDT:      return "other watchdog";
    case ESP_RST_BROWNOUT: return "BROWNOUT (supply voltage dropped)";
    case ESP_RST_USB:      return "USB";
    case ESP_RST_JTAG:     return "JTAG";
    default:               return "other";
    }
}

static void test_board(void)
{
    section("1. ESP32-S3 board");
    esp_chip_info_t ci;
    esp_chip_info(&ci);
    uint32_t flash = 0;
    esp_flash_get_physical_size(NULL, &flash);
    uint8_t mac[6] = { 0 };
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    esp_reset_reason_t rr = esp_reset_reason();

    printf("  Chip          : %s, revision v%d.%d, %d CPU cores @ %d MHz\n", CONFIG_IDF_TARGET,
           ci.revision / 100, ci.revision % 100, ci.cores, CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
    printf("  Features      : %s%s%s%s\n",
           (ci.features & CHIP_FEATURE_WIFI_BGN) ? "Wi-Fi " : "",
           (ci.features & CHIP_FEATURE_BLE) ? "BLE " : "",
           (ci.features & CHIP_FEATURE_EMB_FLASH) ? "embedded-flash " : "",
           (ci.features & CHIP_FEATURE_EMB_PSRAM) ? "embedded-PSRAM" : "");
    printf("  Flash (real)  : %" PRIu32 " MB\n", flash / (1024 * 1024));
    printf("  MAC (Wi-Fi)   : %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    printf("  ESP-IDF       : %s\n", esp_get_idf_version());
    printf("  Reset reason  : %s\n", reset_reason_str(rr));
    printf("  Free heap     : %" PRIu32 " bytes\n", esp_get_free_heap_size());

    if (rr == ESP_RST_BROWNOUT) {
        result("ESP32 board", WARN, "last reset was a brownout - check the 5 V / USB supply");
    } else {
        result("ESP32 board", PASS, "%s rev v%d.%d, %" PRIu32 " MB flash", CONFIG_IDF_TARGET,
               ci.revision / 100, ci.revision % 100, flash / (1024 * 1024));
    }
}

/* ------------------------------------------------------------------ 2. I2C */
static esp_err_t i2c_open(int sda, int scl, i2c_master_bus_handle_t *bus)
{
    i2c_master_bus_config_t bc = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&bc, bus);
}

static int read_idle_level(int gpio, bool pullup)
{
    gpio_reset_pin(gpio);
    gpio_set_direction(gpio, GPIO_MODE_INPUT);
    if (pullup) {
        gpio_pullup_en(gpio);
    } else {
        gpio_pullup_dis(gpio);
    }
    esp_rom_delay_us(200);
    return gpio_get_level(gpio);
}

/* Returns the bus with the MPU6050 on it (or NULL) and its address. */
static i2c_master_bus_handle_t test_i2c(uint8_t *mpu_addr)
{
    section("2. I2C bus (MPU6050)");
    *mpu_addr = 0;
    printf("  Pins          : SDA=GPIO%d  SCL=GPIO%d\n", PIN_SDA, PIN_SCL);
    int sda_lvl = read_idle_level(PIN_SDA, true), scl_lvl = read_idle_level(PIN_SCL, true);
    printf("  Idle levels   : SDA=%d SCL=%d (both should be 1)\n", sda_lvl, scl_lvl);
    if (!sda_lvl || !scl_lvl) {
        result("I2C lines", FAIL, "SDA/SCL held low - check for a short or a swapped wire");
    }

    esp_log_level_set("i2c.master", ESP_LOG_NONE);
    i2c_master_bus_handle_t bus = NULL;
    if (i2c_open(PIN_SDA, PIN_SCL, &bus) != ESP_OK) {
        result("I2C bus", FAIL, "could not open I2C on GPIO%d/%d", PIN_SDA, PIN_SCL);
        return NULL;
    }
    printf("  Devices found :");
    int n = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (i2c_master_probe(bus, a, 20) == ESP_OK) {
            printf(" 0x%02X", a);
            n++;
            if (!*mpu_addr && (a == MPU6050_ADDR_AD0_LOW || a == MPU6050_ADDR_AD0_HIGH)) {
                *mpu_addr = a;
            }
        }
    }
    printf("%s\n", n ? "" : " none");

    if (*mpu_addr) {
        result("I2C bus", PASS, "%d device(s); MPU6050 at 0x%02X", n, *mpu_addr);
        return bus;
    }
    i2c_del_master_bus(bus);

    /* Not on the expected pins: look for 0x68/0x69 on every pair of header GPIOs. */
    static const int cand[] = { 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 17, 18, 21, 38, 47, 48 };
    const int nc = sizeof(cand) / sizeof(cand[0]);
    printf("  MPU6050 not found on GPIO%d/%d - searching other header pins...\n", PIN_SDA, PIN_SCL);
    for (int i = 0; i < nc; i++) {
        for (int j = 0; j < nc; j++) {
            if (i == j || i2c_open(cand[i], cand[j], &bus) != ESP_OK) {
                continue;
            }
            bool hit = i2c_master_probe(bus, 0x68, 5) == ESP_OK || i2c_master_probe(bus, 0x69, 5) == ESP_OK;
            i2c_del_master_bus(bus);
            if (hit) {
                result("I2C bus", FAIL, "MPU6050 answers on SDA=GPIO%d SCL=GPIO%d, not GPIO%d/%d - fix wiring or Kconfig",
                       cand[i], cand[j], PIN_SDA, PIN_SCL);
                return NULL;
            }
        }
    }
    result("I2C bus", FAIL, "no MPU6050 on any pin pair - check VCC=3V3, GND, SDA, SCL");
    return NULL;
}

/* ------------------------------------------------------------------ 3. MPU6050 */
static void test_mpu(i2c_master_bus_handle_t bus, uint8_t addr)
{
    section("3. MPU6050");
    if (!bus) {
        result("MPU6050", SKIP, "not found on I2C");
        return;
    }
    mpu6050_t mpu;
    if (mpu6050_attach(bus, addr, 400000, &mpu) != ESP_OK) {
        result("MPU6050", FAIL, "could not attach I2C device");
        return;
    }
    uint8_t who = 0;
    mpu6050_who_am_i(&mpu, &who);
    printf("  WHO_AM_I      : 0x%02X (genuine MPU6050 = 0x68)\n", who);
    if (who == MPU6050_WHO_AM_I_VALUE) {
        result("MPU6050 identity", PASS, "WHO_AM_I = 0x68");
    } else {
        result("MPU6050 identity", WARN, "WHO_AM_I = 0x%02X - clone or different chip; registers may differ", who);
    }
    if (mpu6050_configure(&mpu) != ESP_OK) {
        result("MPU6050 config", FAIL, "register writes failed");
        mpu6050_detach(&mpu);
        return;
    }

    /* 1000 samples with the sensor still */
    printf("  Keep the board STILL for 2 seconds...\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    const int N = 1000;
    double sum[7] = { 0 }, sq[7] = { 0 };
    int ok = 0, errs = 0;
    int64_t read_us = 0, t0 = esp_timer_get_time();
    for (int i = 0; i < N; i++) {
        mpu6050_sample_t s;
        int64_t a = esp_timer_get_time();
        esp_err_t e = mpu6050_read(&mpu, &s);
        read_us += esp_timer_get_time() - a;
        if (e == ESP_OK) {
            float v[7] = { s.ax, s.ay, s.az, s.gx, s.gy, s.gz, s.temp_c };
            for (int k = 0; k < 7; k++) {
                sum[k] += v[k];
                sq[k] += (double)v[k] * v[k];
            }
            ok++;
        } else {
            errs++;
        }
        vTaskDelay(1);
    }
    double secs = (esp_timer_get_time() - t0) / 1e6;
    if (ok == 0) {
        result("MPU6050 data", FAIL, "all %d reads failed", N);
        mpu6050_detach(&mpu);
        return;
    }
    double mean[7], sd[7];
    for (int k = 0; k < 7; k++) {
        mean[k] = sum[k] / ok;
        sd[k] = sqrt(fmax(0.0, sq[k] / ok - mean[k] * mean[k]));
    }
    double g = sqrt(mean[0] * mean[0] + mean[1] * mean[1] + mean[2] * mean[2]);
    printf("  Reads         : %d ok, %d failed, %.0f reads/s, %.0f us per read (400 kHz I2C)\n",
           ok, errs, ok / secs, (double)read_us / N);
    printf("  Accel  mean g : X %+.3f  Y %+.3f  Z %+.3f   |a| = %.3f g\n", mean[0], mean[1], mean[2], g);
    printf("  Accel  noise  : X %.4f  Y %.4f  Z %.4f g (std dev)\n", sd[0], sd[1], sd[2]);
    printf("  Gyro   bias   : X %+.2f  Y %+.2f  Z %+.2f deg/s (removed by calibration later)\n", mean[3], mean[4], mean[5]);
    printf("  Gyro   noise  : X %.3f  Y %.3f  Z %.3f deg/s (std dev)\n", sd[3], sd[4], sd[5]);
    printf("  Temperature   : %.1f C\n", mean[6]);

    const char *axis[] = { "X", "Y", "Z" };
    int dom = 0;
    for (int k = 1; k < 3; k++) {
        if (fabs(mean[k]) > fabs(mean[dom])) {
            dom = k;
        }
    }
    printf("  Orientation   : gravity is along %s%s - that axis points %s\n",
           mean[dom] > 0 ? "+" : "-", axis[dom], mean[dom] > 0 ? "UP" : "DOWN");
    printf("  Tilt          : about X %+.1f deg, about Y %+.1f deg\n",
           atan2(mean[1], mean[2]) * RAD2DEG, atan2(-mean[0], sqrt(mean[1] * mean[1] + mean[2] * mean[2])) * RAD2DEG);

    result("MPU6050 I2C reads", errs ? WARN : PASS, "%d/%d ok, %.0f us per read", ok, N, (double)read_us / N);
    result("MPU6050 accel", (g > 0.9 && g < 1.1) ? PASS : WARN, "|a| = %.3f g (expect ~1.000), noise %.4f g",
           g, fmax(sd[0], fmax(sd[1], sd[2])));
    double maxbias = fmax(fabs(mean[3]), fmax(fabs(mean[4]), fabs(mean[5])));
    double maxnoise = fmax(sd[3], fmax(sd[4], sd[5]));
    result("MPU6050 gyro", (maxbias < 20 && maxnoise < 0.5) ? PASS : WARN,
           "bias up to %.2f deg/s, noise %.3f deg/s%s", maxbias, maxnoise,
           maxnoise >= 0.5 ? " (board moved or vibrating?)" : "");

    printf("\n  LIVE: tilt the board / robot now (5 s)\n");
    printf("     angle X   angle Y   gyro X   gyro Y   gyro Z\n");
    for (int i = 0; i < 20; i++) {
        mpu6050_sample_t s;
        if (mpu6050_read(&mpu, &s) == ESP_OK) {
            printf("   %+7.1f   %+7.1f  %+7.1f  %+7.1f  %+7.1f\n",
                   atan2f(s.ay, s.az) * RAD2DEG, atan2f(-s.ax, sqrtf(s.ay * s.ay + s.az * s.az)) * RAD2DEG,
                   s.gx - mean[3], s.gy - mean[4], s.gz - mean[5]);
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    mpu6050_detach(&mpu);
}

/* ------------------------------------------------------------------ 4-8. DYNAMIXEL */
typedef struct {
    uint8_t id;
    const char *name;
    bool found;
    uint8_t op_mode, hw_err, torque;
} motor_t;

static const char *op_mode_str(uint8_t m)
{
    switch (m) {
    case 0:  return "current";
    case 1:  return "velocity";
    case 3:  return "position";
    case 4:  return "extended position";
    case 16: return "PWM";
    default: return "?";
    }
}

static void print_hw_error(uint8_t e)
{
    if (!e) {
        printf("none");
        return;
    }
    if (e & 0x01) printf("input-voltage ");
    if (e & 0x04) printf("overheating ");
    if (e & 0x08) printf("motor-encoder ");
    if (e & 0x10) printf("electrical-shock ");
    if (e & 0x20) printf("overload ");
}

static bool ping_retry(uint8_t id, uint16_t *model, uint8_t *fw)
{
    for (int i = 0; i < 3; i++) {
        uint8_t err;
        if (dxl_ping(id, model, fw, &err) == ESP_OK) {
            return true;
        }
    }
    return false;
}

static bool test_dxl_bus(void)
{
    section("4. DYNAMIXEL bus wiring");
    printf("  Pins          : TX=GPIO%d (open-drain) + RX=GPIO%d jumpered, UART1, %d bps\n",
           PIN_DXL_TX, PIN_DXL_RX, CONFIG_HWT_DXL_BAUD);
    int idle = read_idle_level(PIN_DXL_RX, false);
    printf("  Idle level    : DATA (3.3 V side) = %d (should be 1, pulled up by the level shifter)\n", idle);
    if (!idle) {
        result("DXL idle level", FAIL, "DATA held low - check level shifter LV/HV power and GND/DATA wiring");
    } else {
        result("DXL idle level", PASS, "DATA line idles high");
    }

    dxl_config_t cfg = {
        .port = DXL_UART,
        .tx_gpio = PIN_DXL_TX,
        .rx_gpio = PIN_DXL_RX,
        .baud = CONFIG_HWT_DXL_BAUD,
        .reply_timeout_ms = 20,
    };
    if (dxl_init(&cfg) != ESP_OK) {
        result("DXL UART", FAIL, "UART init failed");
        return false;
    }
    /* Ping an ID nobody uses: the echo must come back, the reply must not. */
    uint8_t err;
    esp_err_t e = dxl_ping(200, NULL, NULL, &err);
    if (e == ESP_ERR_INVALID_RESPONSE) {
        result("DXL echo", FAIL, "our packet did not come back on RX - TX1-RX0 jumper missing or wrong pins");
        return false;
    }
    result("DXL echo", PASS, "TX->RX echo intact (jumper and open-drain OK)");
    return true;
}

static void find_motors(motor_t *m, int nm)
{
    section("5. Find motors");
    for (int i = 0; i < nm; i++) {
        uint16_t model = 0;
        uint8_t fw = 0;
        m[i].found = ping_retry(m[i].id, &model, &fw);
        printf("  %-6s ID %d at %d bps : %s", m[i].name, m[i].id, CONFIG_HWT_DXL_BAUD, m[i].found ? "FOUND" : "no reply");
        if (m[i].found) {
            printf(" (model %u%s, firmware v%u)", model, model == XL430_MODEL_NUMBER_VALUE ? " = XL430-W250" : "", fw);
        }
        printf("\n");
    }
    if (m[0].found && m[1].found) {
        result("Motors found", PASS, "IDs %d and %d at %d bps", m[0].id, m[1].id, CONFIG_HWT_DXL_BAUD);
        return;
    }

    printf("  Scanning all baud rates, IDs 0-20 (motors need battery power)...\n");
    static const uint32_t bauds[] = { 57600, 115200, 1000000, 2000000, 3000000, 4000000, 9600 };
    int total = 0;
    for (size_t b = 0; b < sizeof(bauds) / sizeof(bauds[0]); b++) {
        dxl_set_baud(bauds[b]);
        dxl_set_reply_timeout(bauds[b] <= 9600 ? 40 : 6);
        for (int id = 0; id <= 20; id++) {
            uint16_t model;
            uint8_t fw, err;
            if (dxl_ping(id, &model, &fw, &err) == ESP_OK) {
                printf("    found ID %d at %" PRIu32 " bps (model %u, fw v%u)\n", id, bauds[b], model, fw);
                total++;
            }
        }
    }
    dxl_set_baud(CONFIG_HWT_DXL_BAUD);
    dxl_set_reply_timeout(20);
    if (total) {
        result("Motors found", FAIL, "motor(s) answer, but not as ID %d/%d at %d bps - see scan above",
               m[0].id, m[1].id, CONFIG_HWT_DXL_BAUD);
    } else {
        result("Motors found", FAIL, "no motor answers - battery on? HV1->motor pin 3? common GND? HV=5 V?");
    }
}

static void motor_details(motor_t *m)
{
    uint16_t model = 0, vmax = 0, vmin = 0, volt = 0, load = 0;
    uint8_t fw = 0, id = 0, baud = 0, rdt = 0, drive = 0, proto = 0, tlim = 0, shut = 0, temp = 0, led = 0;
    uint32_t vlim = 0, pos = 0, vel = 0;
    int bad = 0;
    bad += dxl_read_u16(m->id, XL430_MODEL_NUMBER, &model) != ESP_OK;
    bad += dxl_read_u8(m->id, XL430_FIRMWARE_VERSION, &fw) != ESP_OK;
    bad += dxl_read_u8(m->id, XL430_ID, &id) != ESP_OK;
    bad += dxl_read_u8(m->id, XL430_BAUD_RATE, &baud) != ESP_OK;
    bad += dxl_read_u8(m->id, XL430_RETURN_DELAY_TIME, &rdt) != ESP_OK;
    bad += dxl_read_u8(m->id, XL430_DRIVE_MODE, &drive) != ESP_OK;
    bad += dxl_read_u8(m->id, XL430_OPERATING_MODE, &m->op_mode) != ESP_OK;
    bad += dxl_read_u8(m->id, XL430_PROTOCOL_TYPE, &proto) != ESP_OK;
    bad += dxl_read_u8(m->id, XL430_TEMPERATURE_LIMIT, &tlim) != ESP_OK;
    bad += dxl_read_u16(m->id, XL430_MAX_VOLTAGE_LIMIT, &vmax) != ESP_OK;
    bad += dxl_read_u16(m->id, XL430_MIN_VOLTAGE_LIMIT, &vmin) != ESP_OK;
    bad += dxl_read_u32(m->id, XL430_VELOCITY_LIMIT, &vlim) != ESP_OK;
    bad += dxl_read_u8(m->id, XL430_SHUTDOWN, &shut) != ESP_OK;
    bad += dxl_read_u8(m->id, XL430_TORQUE_ENABLE, &m->torque) != ESP_OK;
    bad += dxl_read_u8(m->id, XL430_LED, &led) != ESP_OK;
    bad += dxl_read_u8(m->id, XL430_HARDWARE_ERROR_STATUS, &m->hw_err) != ESP_OK;
    bad += dxl_read_u16(m->id, XL430_PRESENT_LOAD, &load) != ESP_OK;
    bad += dxl_read_u32(m->id, XL430_PRESENT_VELOCITY, &vel) != ESP_OK;
    bad += dxl_read_u32(m->id, XL430_PRESENT_POSITION, &pos) != ESP_OK;
    bad += dxl_read_u16(m->id, XL430_PRESENT_INPUT_VOLTAGE, &volt) != ESP_OK;
    bad += dxl_read_u8(m->id, XL430_PRESENT_TEMPERATURE, &temp) != ESP_OK;

    /* A latched error (e.g. low voltage while USB back-powered the motor) only clears on reboot. */
    uint8_t first_err = m->hw_err;
    if (first_err && dxl_reboot(m->id, NULL) == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        dxl_read_u8(m->id, XL430_HARDWARE_ERROR_STATUS, &m->hw_err);
        dxl_read_u8(m->id, XL430_TORQUE_ENABLE, &m->torque);
        dxl_read_u16(m->id, XL430_PRESENT_INPUT_VOLTAGE, &volt);
    }

    printf("\n  --- %s motor (ID %d) ---\n", m->name, m->id);
    if (bad) {
        printf("  %d of 21 register reads failed\n", bad);
    }
    printf("  Model / firmware   : %u%s / v%u\n", model, model == XL430_MODEL_NUMBER_VALUE ? " (XL430-W250)" : "", fw);
    printf("  ID / baud          : %u / reg %u = %" PRIu32 " bps\n", id, baud, dxl_baud_from_reg(baud));
    printf("  Return delay       : %u (= %u us)\n", rdt, rdt * 2);
    printf("  Operating mode     : %u (%s), drive mode %u, protocol %u.0\n", m->op_mode, op_mode_str(m->op_mode), drive, proto);
    printf("  Limits             : temp %u C, voltage %.1f-%.1f V, velocity %" PRIu32 " (%.1f rpm)\n",
           tlim, vmin / 10.0, vmax / 10.0, vlim, vlim * XL430_RPM_PER_UNIT);
    printf("  Shutdown mask      : 0x%02X, torque %s, LED %s\n", shut, m->torque ? "ON" : "off", led ? "on" : "off");
    printf("  Hardware errors    : ");
    print_hw_error(m->hw_err);
    printf("\n");
    if (first_err) {
        printf("  (was 0x%02X at start; motor rebooted to clear it)\n", first_err);
    }
    printf("  Input voltage      : %.1f V   (battery)\n", volt / 10.0);
    printf("  Temperature        : %u C\n", temp);
    printf("  Position           : %" PRIu32 " (%.1f deg), velocity %.1f rpm, load %.1f %%\n",
           pos, (pos % 4096) * 360.0 / 4096, (int32_t)vel * XL430_RPM_PER_UNIT, (int16_t)load / 10.0);

    char name[28];
    snprintf(name, sizeof(name), "%s motor config", m->name);
    if (bad) {
        result(name, FAIL, "%d register reads failed", bad);
    } else if (m->op_mode != 1 || rdt != 0 || dxl_baud_from_reg(baud) != CONFIG_HWT_DXL_BAUD) {
        result(name, WARN, "mode %s, return delay %u, baud %" PRIu32 " (want velocity, 0, %d)",
               op_mode_str(m->op_mode), rdt, dxl_baud_from_reg(baud), CONFIG_HWT_DXL_BAUD);
    } else {
        result(name, PASS, "velocity mode, return delay 0, %d bps", CONFIG_HWT_DXL_BAUD);
    }
    snprintf(name, sizeof(name), "%s motor health", m->name);
    if (m->hw_err) {
        result(name, FAIL, "hardware error 0x%02X - power-cycle the motor", m->hw_err);
    } else if (first_err) {
        result(name, WARN, "error 0x%02X at start, cleared by reboot (switch battery on before USB)", first_err);
    } else if (volt < 100) {
        result(name, WARN, "battery %.1f V is low (3S: charge below ~10 V)", volt / 10.0);
    } else if (temp > 60) {
        result(name, WARN, "temperature %u C is high", temp);
    } else {
        result(name, PASS, "%.1f V, %u C, no hardware errors", volt / 10.0, temp);
    }
}

static void bus_reliability(motor_t *m)
{
    const int N = CONFIG_HWT_PING_COUNT;
    dxl_reset_stats();
    int ok = 0;
    int64_t sum = 0, worst = 0;
    for (int i = 0; i < N; i++) {
        uint8_t err;
        uint32_t pos;
        int64_t t = esp_timer_get_time();
        bool good = dxl_ping(m->id, NULL, NULL, &err) == ESP_OK
                    && dxl_read_u32(m->id, XL430_PRESENT_POSITION, &pos) == ESP_OK;
        int64_t dt = esp_timer_get_time() - t;
        if (good) {
            ok++;
            sum += dt;
            worst = dt > worst ? dt : worst;
        }
    }
    const dxl_stats_t *s = dxl_get_stats();
    printf("  %-6s ID %d: %d/%d ping+read OK, avg %.2f ms, worst %.2f ms | echo err %" PRIu32
           ", timeouts %" PRIu32 ", CRC err %" PRIu32 ", bad %" PRIu32 "\n",
           m->name, m->id, ok, N, ok ? sum / 1000.0 / ok : 0, worst / 1000.0,
           s->echo_errors, s->timeouts, s->crc_errors, s->bad_packets);
    char name[28];
    snprintf(name, sizeof(name), "%s bus reliability", m->name);
    result(name, ok == N ? PASS : (ok >= N * 99 / 100 ? WARN : FAIL), "%d/%d at %d bps, avg %.2f ms per ping+read",
           ok, N, CONFIG_HWT_DXL_BAUD, ok ? sum / 1000.0 / ok : 0);
}

static void spin_test(motor_t *m)
{
    char name[28];
    snprintf(name, sizeof(name), "%s motor spin", m->name);
    if (m->op_mode != 1) {
        result(name, SKIP, "not in velocity mode");
        return;
    }
    if (m->hw_err) {
        result(name, SKIP, "hardware error present");
        return;
    }
    const int32_t v = CONFIG_HWT_SPIN_VELOCITY;
    uint32_t p0 = 0, p1 = 0, vel;
    float meas[2] = { 0 };
    dxl_write_u8(m->id, XL430_TORQUE_ENABLE, 1);
    dxl_read_u32(m->id, XL430_PRESENT_POSITION, &p0);
    for (int dir = 0; dir < 2; dir++) {
        int32_t goal = dir ? -v : v;
        dxl_write_u32(m->id, XL430_GOAL_VELOCITY, (uint32_t)goal);
        vTaskDelay(pdMS_TO_TICKS(700));
        float acc = 0;
        int n = 0;
        for (int i = 0; i < 8; i++) {
            if (dxl_read_u32(m->id, XL430_PRESENT_VELOCITY, &vel) == ESP_OK) {
                acc += (int32_t)vel;
                n++;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        meas[dir] = n ? acc / n : 0;
        if (dir == 0) {
            dxl_read_u32(m->id, XL430_PRESENT_POSITION, &p1);
        }
    }
    dxl_write_u32(m->id, XL430_GOAL_VELOCITY, 0);
    vTaskDelay(pdMS_TO_TICKS(300));
    dxl_write_u8(m->id, XL430_TORQUE_ENABLE, 0);

    int32_t moved = (int32_t)(p1 - p0);
    printf("  %-6s ID %d: goal %+.1f rpm -> measured %+.1f rpm, reverse -> %+.1f rpm, encoder moved %+" PRId32 " counts\n",
           m->name, m->id, v * XL430_RPM_PER_UNIT, meas[0] * XL430_RPM_PER_UNIT, meas[1] * XL430_RPM_PER_UNIT, moved);
    bool fwd_ok = meas[0] > 0.5f * v && meas[0] < 1.5f * v;
    bool rev_ok = meas[1] < -0.5f * v && meas[1] > -1.5f * v;
    result(name, (fwd_ok && rev_ok && moved > 0) ? PASS : FAIL, "fwd %+.1f / rev %+.1f rpm (goal %.1f), encoder %+" PRId32,
           meas[0] * XL430_RPM_PER_UNIT, meas[1] * XL430_RPM_PER_UNIT, v * XL430_RPM_PER_UNIT, moved);
}

static void test_motors(void)
{
    motor_t m[2] = {
        { .id = CONFIG_HWT_DXL_ID_LEFT, .name = "Left" },
        { .id = CONFIG_HWT_DXL_ID_RIGHT, .name = "Right" },
    };
    if (!test_dxl_bus()) {
        return;
    }
    find_motors(m, 2);

    section("6. Motor details");
    for (int i = 0; i < 2; i++) {
        if (m[i].found) {
            motor_details(&m[i]);
        }
    }

    section("7. Bus reliability");
    for (int i = 0; i < 2; i++) {
        if (m[i].found) {
            bus_reliability(&m[i]);
        }
    }

    section("8. LED + spin test");
    for (int i = 0; i < 2; i++) {
        if (m[i].found) {
            printf("  %s motor LED blinking...\n", m[i].name);
            for (int k = 0; k < 3; k++) {
                dxl_write_u8(m[i].id, XL430_LED, 1);
                vTaskDelay(pdMS_TO_TICKS(200));
                dxl_write_u8(m[i].id, XL430_LED, 0);
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        }
    }
#if CONFIG_HWT_SPIN_TEST
    if (m[0].found || m[1].found) {
        printf("  Motors will turn SLOWLY (%.1f rpm) forward and back. Keep the wheels free.\n",
               CONFIG_HWT_SPIN_VELOCITY * XL430_RPM_PER_UNIT);
        for (int s = 5; s > 0; s--) {
            printf("  starting in %d...\n", s);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        for (int i = 0; i < 2; i++) {
            if (m[i].found) {
                spin_test(&m[i]);
            }
        }
    }
#else
    result("Motor spin", SKIP, "disabled in menuconfig");
#endif
}

/* ------------------------------------------------------------------ main */
void app_main(void)
{
    for (int s = 3; s > 0; s--) {
        printf("Hardware test starts in %d...\n", s);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("\n######## TWO-WHEEL ROBOT HARDWARE TEST ########\n");

    test_board();
    uint8_t mpu_addr;
    i2c_master_bus_handle_t bus = test_i2c(&mpu_addr);
    test_mpu(bus, mpu_addr);
    if (bus) {
        i2c_del_master_bus(bus);
    }
    test_motors();

    section("9. SUMMARY");
    int counts[4] = { 0 };
    for (int i = 0; i < s_nres; i++) {
        counts[s_res[i].v]++;
        printf("  [%s] %-26s %s\n", VSTR[s_res[i].v], s_res[i].name, s_res[i].note);
    }
    printf("\n  %d PASS, %d WARN, %d FAIL, %d SKIP -> %s\n", counts[PASS], counts[WARN], counts[FAIL], counts[SKIP],
           counts[FAIL] ? "FIX THE FAILURES ABOVE" : (counts[WARN] ? "OK, but read the warnings" : "ALL HARDWARE OK"));
    printf("\nTest finished. Press RST on the board to run it again.\n");
}
