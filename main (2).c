/*
 * ESP32 + MPU6500：陀螺儀 Z 軸積分出 yaw，經 UART0(USB) 以固定封包傳給樹莓派。
 * 另外：接收樹莓派指令，控制 GPIO17 輸出 3.3V / PWM。
 *
 * ---- ESP32 -> 樹莓派（11 bytes，小端序，float32）----
 *   [0]     0xAA
 *   [1]     0x55
 *   [2]     seq        uint8，每包 +1
 *   [3-6]   yaw        float32，yaw 角度 (度)
 *   [7-10]  rate       float32，角速度 (度/秒)
 *
 * ---- 樹莓派 -> ESP32（7 bytes，小端序）----
 *   [0]     0xA5
 *   [1]     0x5A
 *   [2]     enable     uint8，1 = 輸出，0 = 關閉 (0V)
 *   [3]     duty       uint8，工作比 0~100 (%)；100 = 持續高電位 3.3V（不是 PWM）
 *   [4-5]   freq       uint16，PWM 頻率 (Hz)，限制 100~20000
 *   [6]     checksum   = [2] ^ [3] ^ [4] ^ [5]
 *
 * ---- 樹莓派 -> ESP32：丟壘包（6 bytes）----
 *   [0]     0xA5
 *   [1]     0x5B
 *   [2]     seq        uint8，同一個 seq 只動作一次（樹莓派每 0.1 s 重送到 ESP32 回報收到為止）
 *   [3]     dur        uint8，輸出脈衝寬度，單位 10 ms（0 = 用預設 DROP_DEFAULT_MS）
 *   [4]     checksum   = [2] ^ [3]
 *   動作：DROP_GPIO 輸出高電位 dur*10 ms 後自動回到低電位（脈衝時間到一定會關，不依賴後續指令）。
 *
 * ---- ESP32 -> 樹莓派：丟壘包回報（5 bytes，約 10 Hz，穿插在 11 bytes 角度封包之間）----
 *   [0]     0xAA
 *   [1]     0x56
 *   [2]     seq        uint8，最近一次丟壘包的 seq
 *   [3]     state      uint8，0 = 開機後還沒丟過、1 = 丟壘包中、2 = 已完成
 *   [4]     checksum   = [2] ^ [3]
 *
 *   安全：超過 CMD_TIMEOUT_MS 沒收到有效指令 -> 自動關閉輸出（樹莓派/網頁/control 當掉也會停）。
 */
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ---------- 可調參數 ---------- */
#define I2C_SCL_IO        7
#define I2C_SDA_IO        6
#define I2C_PORT          I2C_NUM_0
#define I2C_FREQ_HZ       400000
#define MPU_ADDR          0x68

#define UART_PORT         UART_NUM_0
#define UART_BAUD         115200
#define SAMPLE_MS         10              /* 100 Hz */

#define DEADBAND_DPS      0.3f
#define BIAS_ALPHA        0.01f
#define CALIB_SAMPLES     500
#define CALIB_MAX_STD     30.0

/* ---- GPIO17 輸出 ---- */
#define OUT_GPIO          17
#define OUT_TIMER         LEDC_TIMER_0
#define OUT_CHANNEL       LEDC_CHANNEL_0
#define OUT_SPEED_MODE    LEDC_LOW_SPEED_MODE   /* ESP32-C3 只有 low speed；其他型號也可用 */
#define OUT_RES           LEDC_TIMER_10_BIT
#define OUT_DUTY_MAX      1023
#define OUT_DEFAULT_FREQ  1000
#define CMD_TIMEOUT_MS    500                   /* 超過這個時間沒收到指令就關閉輸出 */

/* ---- 丟壘包輸出（脈衝）---- */
#define DROP_GPIO         5                     /* 接丟壘包的繼電器 / 電磁閥 / 驅動板；請依實際接線修改 */
#define DROP_ACTIVE_LEVEL 1                     /* 1 = 高電位動作；繼電器板若是低電位觸發改成 0 */
#define DROP_DEFAULT_MS   1000
#define DROP_MAX_MS       2550

/* ---------- MPU 暫存器 ---------- */
#define REG_PWR_MGMT_1    0x6B
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_GYRO_ZOUT_H   0x47
#define LSB_PER_DPS       131.0f

static float gz_bias = 0.0f;

/* ================= GPIO17 輸出（LEDC PWM） ================= */
static uint8_t  out_en = 0, out_duty = 0;
static uint32_t out_freq = OUT_DEFAULT_FREQ;
static int64_t  last_cmd_ms = 0;

static void out_apply(void)
{
    ledc_set_freq(OUT_SPEED_MODE, OUT_TIMER, out_freq);
    if (!out_en || out_duty == 0) {
        ledc_stop(OUT_SPEED_MODE, OUT_CHANNEL, 0);          /* 持續低電位 0V */
    } else if (out_duty >= 100) {
        ledc_stop(OUT_SPEED_MODE, OUT_CHANNEL, 1);          /* 持續高電位 3.3V（無 PWM 缺口） */
    } else {
        ledc_set_duty(OUT_SPEED_MODE, OUT_CHANNEL, (uint32_t)out_duty * OUT_DUTY_MAX / 100);
        ledc_update_duty(OUT_SPEED_MODE, OUT_CHANNEL);
    }
}

static void out_init(void)
{
    ledc_timer_config_t tcfg = {
        .speed_mode = OUT_SPEED_MODE,
        .duty_resolution = OUT_RES,
        .timer_num = OUT_TIMER,
        .freq_hz = OUT_DEFAULT_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&tcfg);
    ledc_channel_config_t ccfg = {
        .gpio_num = OUT_GPIO,
        .speed_mode = OUT_SPEED_MODE,
        .channel = OUT_CHANNEL,
        .timer_sel = OUT_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&ccfg);
    out_en = 0;
    out_apply();                                            /* 開機先輸出 0V */
}

/* ================= 丟壘包（GPIO 脈衝） ================= */
static int64_t drop_end_ms = 0;          /* 脈衝結束時間；0 = 目前沒有在丟 */
static int     drop_last_seq = -1;       /* 上一次已執行的 seq（-1 = 開機後還沒丟過） */

static void drop_init(void)
{
    gpio_config_t g = {
        .pin_bit_mask = 1ULL << DROP_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_set_level(DROP_GPIO, !DROP_ACTIVE_LEVEL);          /* 先設好未動作電位再變輸出，避免開機瞬間誤動作 */
    gpio_config(&g);
    gpio_set_level(DROP_GPIO, !DROP_ACTIVE_LEVEL);
}

static void drop_start(uint8_t seq, uint8_t dur10)
{
    if (seq == drop_last_seq) return;                       /* 重送的同一個指令，忽略 */
    if (drop_end_ms != 0) return;                           /* 正在丟，不重複觸發（也不記 seq，之後新的 seq 仍可執行） */
    drop_last_seq = seq;
    uint32_t ms = dur10 ? (uint32_t)dur10 * 10 : DROP_DEFAULT_MS;
    if (ms > DROP_MAX_MS) ms = DROP_MAX_MS;
    drop_end_ms = esp_timer_get_time() / 1000 + ms;
    if (drop_end_ms == 0) drop_end_ms = 1;
    gpio_set_level(DROP_GPIO, DROP_ACTIVE_LEVEL);
}

static void send_drop_status(void)
{
    uint8_t seq = drop_last_seq < 0 ? 0 : (uint8_t)drop_last_seq;
    uint8_t state = drop_last_seq < 0 ? 0 : (drop_end_ms != 0 ? 1 : 2);
    uint8_t f[5] = {0xAA, 0x56, seq, state, (uint8_t)(seq ^ state)};
    uart_write_bytes(UART_PORT, (const char *)f, sizeof f);
}

static void drop_poll(void)
{
    if (drop_end_ms != 0 && esp_timer_get_time() / 1000 >= drop_end_ms) {
        gpio_set_level(DROP_GPIO, !DROP_ACTIVE_LEVEL);
        drop_end_ms = 0;
    }
}

/* 解析樹莓派指令：
 *   輸出控制  A5 5A | en | duty | freq_lo | freq_hi | chk
 *   丟壘包    A5 5B | seq | dur10 | chk              */
static void cmd_poll(void)
{
    static int st = 0, idx = 0, need = 0;
    static uint8_t type = 0;
    static uint8_t p[4];
    uint8_t buf[32];
    int n = uart_read_bytes(UART_PORT, buf, sizeof buf, 0);   /* 非阻塞 */
    for (int i = 0; i < n; i++) {
        uint8_t b = buf[i];
        if (st == 0) {
            if (b == 0xA5) st = 1;
        } else if (st == 1) {
            if (b == 0x5A)      { type = b; need = 4; idx = 0; st = 2; }
            else if (b == 0x5B) { type = b; need = 2; idx = 0; st = 2; }
            else                 st = (b == 0xA5) ? 1 : 0;
        } else if (st == 2) {
            p[idx++] = b;
            if (idx == need) st = 3;
        } else {
            if (type == 0x5A) {
                if (b == (uint8_t)(p[0] ^ p[1] ^ p[2] ^ p[3])) {
                    uint8_t  en = p[0] ? 1 : 0;
                    uint8_t  duty = p[1] > 100 ? 100 : p[1];
                    uint32_t freq = (uint32_t)p[2] | ((uint32_t)p[3] << 8);
                    if (freq < 100) freq = 100;
                    if (freq > 20000) freq = 20000;
                    last_cmd_ms = esp_timer_get_time() / 1000;
                    if (en != out_en || duty != out_duty || freq != out_freq) {
                        out_en = en; out_duty = duty; out_freq = freq;
                        out_apply();
                    }
                }
            } else if (type == 0x5B) {
                if (b == (uint8_t)(p[0] ^ p[1])) drop_start(p[0], p[1]);
            }
            st = 0;
        }
    }
    /* 失效保護：指令中斷就關閉 */
    if (out_en && (esp_timer_get_time() / 1000 - last_cmd_ms) > CMD_TIMEOUT_MS) {
        out_en = 0;
        out_apply();
    }
    drop_poll();                                              /* 丟壘包脈衝時間到就關 */
}

/* ================= MPU ================= */
static esp_err_t i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_PORT, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

static esp_err_t mpu_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(I2C_PORT, MPU_ADDR, buf, 2, pdMS_TO_TICKS(100));
}

static esp_err_t mpu_read_gz(int16_t *raw)
{
    uint8_t reg = REG_GYRO_ZOUT_H, b[2];
    esp_err_t err = i2c_master_write_read_device(I2C_PORT, MPU_ADDR, &reg, 1, b, 2, pdMS_TO_TICKS(100));
    if (err == ESP_OK) *raw = (int16_t)((b[0] << 8) | b[1]);
    return err;
}

static void mpu_init(void)
{
    mpu_write(REG_PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));
    mpu_write(REG_PWR_MGMT_1, 0x01);
    vTaskDelay(pdMS_TO_TICKS(50));
    mpu_write(REG_CONFIG, 0x03);
    mpu_write(REG_GYRO_CONFIG, 0x00);
    vTaskDelay(pdMS_TO_TICKS(50));
}

static void calibrate_gyro(void)
{
    for (int attempt = 0; attempt < 3; attempt++) {
        double sum = 0, sq = 0;
        int n = 0;
        for (int i = 0; i < CALIB_SAMPLES; i++) {
            int16_t r;
            if (mpu_read_gz(&r) == ESP_OK) {
                sum += r;
                sq += (double)r * r;
                n++;
            }
            vTaskDelay(pdMS_TO_TICKS(3));
        }
        if (n == 0) continue;
        double mean = sum / n;
        double var = sq / n - mean * mean;
        gz_bias = (float)mean;
        if (var <= CALIB_MAX_STD * CALIB_MAX_STD) break;
    }
}

static void send_frame(uint8_t seq, float yaw, float dps)
{
    uint8_t f[11];
    f[0] = 0xAA;
    f[1] = 0x55;
    f[2] = seq;
    memcpy(&f[3], &yaw, 4);   /* 直接複製 4 bytes float32 */
    memcpy(&f[7], &dps, 4);   /* 直接複製 4 bytes float32 */
    uart_write_bytes(UART_PORT, (const char *)f, sizeof f);
}

void app_main(void)
{
    uart_config_t ucfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(UART_PORT, &ucfg);
    uart_driver_install(UART_PORT, 512, 0, 0, NULL, 0);

    out_init();                 /* GPIO17 先設成 0V，再做耗時的 MPU 校正 */
    drop_init();                /* 丟壘包腳位先設成未動作電位 */

    i2c_init();
    mpu_init();
    calibrate_gyro();

    float yaw = 0.0f;
    uint8_t seq = 0;
    int status_div = 0;
    int64_t t_prev = esp_timer_get_time();
    TickType_t wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(SAMPLE_MS));

        cmd_poll();             /* 先處理樹莓派指令，MPU 讀取失敗時也不影響輸出控制 */
        if (++status_div >= 10) { status_div = 0; send_drop_status(); }   /* 10 Hz 丟壘包回報（MPU 讀取失敗時也照送） */

        int64_t now = esp_timer_get_time();
        float dt = (float)(now - t_prev) / 1e6f;
        t_prev = now;
        if (dt > 0.05f) dt = SAMPLE_MS / 1000.0f;

        int16_t raw;
        if (mpu_read_gz(&raw) != ESP_OK) continue;

        float dps = ((float)raw - gz_bias) / LSB_PER_DPS;
        if (fabsf(dps) < DEADBAND_DPS) {
            dps = 0.0f;
            gz_bias += BIAS_ALPHA * ((float)raw - gz_bias);
        }

        yaw += dps * dt;
        if (yaw > 180.0f) yaw -= 360.0f;
        else if (yaw <= -180.0f) yaw += 360.0f;

        send_frame(seq++, yaw, dps);
    }
}
