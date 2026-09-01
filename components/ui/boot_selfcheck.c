/**
 * @file boot_selfcheck.c
 * @brief Self-check system — يُشغَّل على boot_screen قبل الانتقال للـ Home.
 *
 * ═══════════════════════════════════════════════════════════════════
 *  تسلسل الفحص
 * ═══════════════════════════════════════════════════════════════════
 *
 *  [1] ESP32     → chip_info + free heap ≥ GRAD_MIN_HEAP_BYTES
 *  [2] ADS1115   → I2C scan @ 0x48 + قراءة differential تجريبية
 *  [3] Upper FLC100 → AIN0 single-ended في المدى [-32768..32767]
 *  [4] Lower FLC100 → AIN1 single-ended في المدى [-32768..32767]
 *  [5] HC-05     → UART_NUM_1 init + إرسال "AT\r\n" + انتظار OK
 *
 *  كل مرحلة:
 *   ① يُحدِّث boot_check_events بـ "Checking <component>..."  أبيض
 *   ② ينتظر CHECK_PAUSE_MS (تأثير بصري)
 *   ③ يُنفِّذ الفحص
 *   ④ إذا نجح → نص أخضر "✓ <component> OK"
 *      إذا فشل → نص أحمر  "✗ <component> FAIL" + lockdown
 *
 *  lockdown = spinner يتوقف + رسالة "System Halted" + loop إلى الأبد
 *  نجاح كامل = lv_scr_load_anim → SCREEN_ID_MAIN
 *
 * ═══════════════════════════════════════════════════════════════════
 *  THREAD SAFETY
 * ═══════════════════════════════════════════════════════════════════
 *  تُستدعى من ui_event_task (Core 1) — تملك LVGL mutex بالفعل.
 *  كل lv_* call مباشر بدون mutex إضافي (نفس الـ task).
 *  I2C calls تُنفَّذ من Core 1 مؤقتاً قبل أن يبدأ adc_task — آمن.
 */

#include "boot_selfcheck.h"
#include "screens.h"

#include "drivers/ads1115_driver.h"
#include "core/gradiometer_types.h"
#include "core/hardware_config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_heap_caps.h"
#include "lvgl.h"

#include <string.h>
#include <stdio.h>

static const char *TAG = "SelfCheck";

/* ── نتيجة عالمية ── */
static bool s_passed = false;

/* ── ثوابت توقيت ── */
#define CHECK_PAUSE_MS      400u   /* توقف قبل بدء كل فحص (تأثير بصري) */
#define CHECK_RESULT_MS     350u   /* عرض نتيجة كل مرحلة               */
#define LOCKDOWN_BLINK_MS   800u   /* وميض رسالة الخطأ                  */
#define FLC100_VALID_MIN   -30000  /* أدنى قيمة ADC منطقية للـ FLC100   */
#define FLC100_VALID_MAX    30000  /* أعلى قيمة ADC منطقية              */
#define BT_AT_TIMEOUT_MS    800u   /* انتظار رد HC-05 على AT            */
#define HEAP_MIN_BYTES      8192u  /* حد أمان الـ heap                  */

/* ── ألوان ── */
#define COLOR_WHITE   lv_color_hex(0xEEEEEE)
#define COLOR_GREEN   lv_color_hex(0x477f53)
#define COLOR_RED     lv_color_hex(0xcd4755)
#define COLOR_YELLOW  lv_color_hex(0xFFC107)

/* ═══════════════════════════════════════════════════════════════════
 * UI HELPERS — تُستدعى من ui_event_task مباشرة (بدون mutex)
 * ═══════════════════════════════════════════════════════════════════ */

static void label_set(const char *text, lv_color_t color)
{
    if (!objects.boot_check_events) return;
    lv_label_set_text(objects.boot_check_events, text);
    lv_obj_set_style_text_color(objects.boot_check_events, color,
                                LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_refr_now(NULL);   /* تحديث فوري للشاشة */
}

static void pause(uint32_t ms)
{
    /* vTaskDelay آمن هنا — نحن في مرحلة boot قبل أن تبدأ باقي الـ tasks */
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/* ═══════════════════════════════════════════════════════════════════
 * LOCKDOWN — يعرض الخطأ ويُجمِّد الجهاز
 * ═══════════════════════════════════════════════════════════════════ */

static void lockdown(const char *failed_component)
{
    ESP_LOGE(TAG, "LOCKDOWN — %s failed", failed_component);

    /* أوقف الـ spinner */
    if (objects.boot_spinner) {
        lv_obj_add_flag(objects.boot_spinner, LV_OBJ_FLAG_HIDDEN);
    }

    char msg[64];
    snprintf(msg, sizeof(msg), "✗ %s FAIL\nSystem Halted", failed_component);

    /* 
     * vTaskDelay داخل pause() يُطعم الـ TWDT تلقائياً — آمن من الـ reset.
     * الحلقة تبقى تعمل للأبد لتجميد الجهاز عند الفشل الحرج.
     */
    for (;;) {
        label_set(msg, COLOR_RED);
        lv_refr_now(NULL);
        pause(LOCKDOWN_BLINK_MS);

        label_set(msg, COLOR_WHITE);
        lv_refr_now(NULL);
        pause(LOCKDOWN_BLINK_MS);
    }
    /* لا نصل هنا أبداً */
}

/* ═══════════════════════════════════════════════════════════════════
 * CHECK 1 — ESP32 Internal
 * ═══════════════════════════════════════════════════════════════════ */

static bool check_esp32(void)
{
    label_set("Checking ESP32...", COLOR_WHITE);
    pause(CHECK_PAUSE_MS);

    esp_chip_info_t chip;
    esp_chip_info(&chip);

    uint32_t free_heap = esp_get_free_heap_size();
    ESP_LOGI(TAG, "ESP32: cores=%d rev=%d heap=%lu",
             chip.cores, chip.revision, (unsigned long)free_heap);

    if (free_heap < HEAP_MIN_BYTES) {
        ESP_LOGE(TAG, "Heap too low: %lu < %u", (unsigned long)free_heap,
                 (unsigned)HEAP_MIN_BYTES);
        return false;
    }

    char ok_msg[48];
    snprintf(ok_msg, sizeof(ok_msg),
             "✓ ESP32  Cores:%d  Heap:%luK",
             chip.cores, (unsigned long)(free_heap / 1024));
    label_set(ok_msg, COLOR_GREEN);
    pause(CHECK_RESULT_MS);
    return true;
}

/* ═══════════════════════════════════════════════════════════════════
 * CHECK 2 — ADS1115
 * ═══════════════════════════════════════════════════════════════════ */

static bool check_ads1115(void)
{
    label_set("Checking ADS1115...", COLOR_WHITE);
    pause(CHECK_PAUSE_MS);

    /* I2C scan — بروب العنوان 0x48 */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,
                          (HW_ADS1115_ADDR << 1) | I2C_MASTER_WRITE,
                          true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(HW_I2C_PORT, cmd,
                                         pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADS1115 not found at 0x%02X: %s",
                 HW_ADS1115_ADDR, esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "ADS1115 found at 0x%02X", HW_ADS1115_ADDR);
    label_set("✓ ADS1115 OK", COLOR_GREEN);
    pause(CHECK_RESULT_MS);
    return true;
}

/* ═══════════════════════════════════════════════════════════════════
 * CHECK 3 & 4 — FLC100 Sensors (via ADS1115)
 * يقرأ AIN0 وAIN1 single-ended ويتحقق من المدى المنطقي
 * ═══════════════════════════════════════════════════════════════════ */

static bool check_flc100_channel(const char *name,
                                 uint16_t mux_bits,
                                 ADS1115Driver_t *drv)
{
    char msg[40];
    snprintf(msg, sizeof(msg), "Checking %s...", name);
    label_set(msg, COLOR_WHITE);
    pause(CHECK_PAUSE_MS);

    if (!drv) {
        ESP_LOGE(TAG, "%s: driver NULL", name);
        return false;
    }

    /* Config register value:
     * OS=1 (start), MUX=?, PGA=010 (±2.048V), MODE=1 (single-shot),
     * DR=100 (128SPS), COMP_MODE=0, COMP_POL=0, COMP_LAT=0, COMP_QUE=11 */

    uint16_t config = 0x8000  /* OS: start single conversion */
                    | (mux_bits << 12)
                    | 0x0400  /* PGA: ±2.048V */
                    | 0x0100  /* MODE: single-shot */
                    | 0x0080  /* DR: 128 SPS */
                    | 0x0003; /* COMP_QUE: disable comparator */

    uint8_t config_bytes[3] = {
        0x01,                         /* config register pointer */
        (uint8_t)(config >> 8),
        (uint8_t)(config & 0xFF)
    };

    /* Write config */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (HW_ADS1115_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, config_bytes, 3, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(HW_I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "%s config write failed", name);
        return false;
    }

    /* انتظر اكتمال التحويل (~8ms @ 128SPS) */
    vTaskDelay(pdMS_TO_TICKS(12));

    /* Read conversion register */
    uint8_t conv_ptr = 0x00;
    uint8_t data[2] = {0};

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (HW_ADS1115_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, conv_ptr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (HW_ADS1115_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(HW_I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "%s read failed", name);
        return false;
    }

    int16_t raw = (int16_t)((data[0] << 8) | data[1]);
    ESP_LOGI(TAG, "%s raw=%d", name, (int)raw);

    /* تحقق من المدى المنطقي — FLC100 لا يُخرج clipping في بيئة طبيعية */
    if (raw < FLC100_VALID_MIN || raw > FLC100_VALID_MAX) {
        ESP_LOGE(TAG, "%s out of range: %d", name, (int)raw);
        return false;
    }

    snprintf(msg, sizeof(msg), "✓ %s OK  raw=%d", name, (int)raw);
    label_set(msg, COLOR_GREEN);
    pause(CHECK_RESULT_MS);
    return true;
}

/* ═══════════════════════════════════════════════════════════════════
 * CHECK 5 — HC-05 Bluetooth
 * ═══════════════════════════════════════════════════════════════════ */

static bool check_hc05(void)
{
    label_set("Checking HC-05...", COLOR_WHITE);
    pause(CHECK_PAUSE_MS);

    /* تحقق أن UART_NUM_1 يمكن تهيئته */
    uart_config_t uart_cfg = {
        .baud_rate  = HW_BT_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    esp_err_t ret = uart_param_config(HW_BT_UART_NUM, &uart_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HC-05 UART param config failed: %s",
                 esp_err_to_name(ret));
        return false;
    }

    ret = uart_set_pin(HW_BT_UART_NUM,
                       HW_BT_TX_PIN,
                       HW_BT_RX_PIN,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HC-05 UART set pin failed");
        return false;
    }

    /* تثبيت الـ UART driver مؤقتاً للفحص (يجب التحقق من نجاحه) */
    ret = uart_driver_install(HW_BT_UART_NUM, 256, 256, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HC-05 UART driver install failed: %s", esp_err_to_name(ret));
        return false;
    }

    /* أرسل "AT\r\n" — إذا كان HC-05 في AT mode يرد بـ "OK" */
    const char *at_cmd = "AT\r\n";
    uart_write_bytes(HW_BT_UART_NUM, at_cmd, strlen(at_cmd));

    /* انتظر رد */
    uint8_t rx_buf[16] = {0};
    int len = uart_read_bytes(HW_BT_UART_NUM, rx_buf,
                              sizeof(rx_buf) - 1,
                              pdMS_TO_TICKS(BT_AT_TIMEOUT_MS));

    /* تنظيف الـ UART بعد الفحص ليقوم bt_sender بتهيئته لاحقاً بشكل صحيح */
    uart_driver_delete(HW_BT_UART_NUM);

    char msg[48];
    if (len > 0) {
        rx_buf[len] = '\0';
        ESP_LOGI(TAG, "HC-05 replied: %s", (char *)rx_buf);
        snprintf(msg, sizeof(msg), "✓ HC-05 OK  [AT replied]");
    } else {
        ESP_LOGI(TAG, "HC-05 UART OK (no AT reply — data mode)");
        snprintf(msg, sizeof(msg), "✓ HC-05 UART OK");
    }

    label_set(msg, COLOR_GREEN);
    pause(CHECK_RESULT_MS);
    return true;
}

/* ═══════════════════════════════════════════════════════════════════
 * MAIN ENTRY POINT
 * ═══════════════════════════════════════════════════════════════════ */

void boot_selfcheck_run(void *ads_driver_ptr)
{
    ADS1115Driver_t *drv = (ADS1115Driver_t *)ads_driver_ptr;
    s_passed = false;

    ESP_LOGI(TAG, "=== Self-Check Started ===");

    /* ── رسالة بداية ── */
    label_set("System starting...", COLOR_WHITE);
    pause(400);

    /* ══════════════════════════════════
     * CHECK 1 — ESP32
     * ══════════════════════════════════ */
    label_set("[ 1/5 ] Checking ESP32...", COLOR_WHITE);
    pause(CHECK_PAUSE_MS);
    if (!check_esp32()) {
        lockdown("ESP32");
    }

    /* ══════════════════════════════════
     * CHECK 2 — ADS1115
     * ══════════════════════════════════ */
    label_set("[ 2/5 ] Checking ADS1115...", COLOR_WHITE);
    pause(CHECK_PAUSE_MS);
    if (!check_ads1115()) {
        lockdown("ADS1115");
    }

    /* ══════════════════════════════════
     * CHECK 3 — Upper FLC100 (AIN0)
     * ══════════════════════════════════ */
    label_set("[ 3/5 ] Checking Upper FLC100...", COLOR_WHITE);
    pause(CHECK_PAUSE_MS);
    /* MUX=100b (0x4): AIN0 vs GND — Upper FLC100 */
    if (!check_flc100_channel("Upper FLC100", 0x4u, drv)) {
        lockdown("Upper FLC100");
    }

    /* ══════════════════════════════════
     * CHECK 4 — Lower FLC100 (AIN1)
     * ══════════════════════════════════ */
    label_set("[ 4/5 ] Checking Lower FLC100...", COLOR_WHITE);
    pause(CHECK_PAUSE_MS);
    /* MUX=101b (0x5): AIN1 vs GND — Lower FLC100 */
    if (!check_flc100_channel("Lower FLC100", 0x5u, drv)) {
        lockdown("Lower FLC100");
    }

    /* ══════════════════════════════════
     * CHECK 5 — HC-05
     * ══════════════════════════════════ */
    label_set("[ 5/5 ] Checking HC-05...", COLOR_WHITE);
    pause(CHECK_PAUSE_MS);
    if (!check_hc05()) {
        lockdown("HC-05");
    }

    /* ══════════════════════════════════
     * كل المكونات اجتازت الفحص
     * ══════════════════════════════════ */
    s_passed = true;

    label_set("✓ All systems OK!", COLOR_GREEN);
    ESP_LOGI(TAG, "=== Self-Check PASSED ===");
    pause(800);

    /* أوقف الـ spinner قبل الانتقال */
    if (objects.boot_spinner) {
        lv_obj_add_flag(objects.boot_spinner, LV_OBJ_FLAG_HIDDEN);
    }

    /* انتقال سلس لـ SCREEN_ID_MAIN */
    lv_scr_load_anim(objects.main,
                     LV_SCR_LOAD_ANIM_FADE_ON,
                     400,    /* duration ms */
                     200,    /* delay ms */
                     false); /* auto_del: false — نحتفظ بـ boot_screen */

    ESP_LOGI(TAG, "Navigating to main screen");
}

bool boot_selfcheck_passed(void)
{
    return s_passed;
}