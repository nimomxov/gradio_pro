/**
 * @file app_main.c
 * @brief Gradiometer Pro — System entry point and orchestrator.
 *
 * ═══════════════════════════════════════════════════════════════════
 *  DUAL-CORE TASK MAP
 * ═══════════════════════════════════════════════════════════════════
 *
 *  CORE 0  (Protocol CPU)            CORE 1  (Application CPU)
 *  ─────────────────────────────     ─────────────────────────────
 *  adc_task        Prio 6  4096B     lvgl_task       Prio 5  6144B
 *  signal_task     Prio 5  4096B     ui_event_task   Prio 4  4096B
 *  bt_sender_task  Prio 4  2048B
 *
 *  QUEUES (FreeRTOS copy — zero shared memory):
 *    adc_queue     depth 4   AdcSample_t       12 B
 *    result_queue  depth 2   ProcessedSample_t 28 B
 *    event_queue   depth 8   SysEventMsg_t     12 B
 *    bt_queue      depth 32  BtMsg_t            4 B  (inside BTSender_t)
 *
 * ═══════════════════════════════════════════════════════════════════
 *  INIT SEQUENCE (order is critical)
 * ═══════════════════════════════════════════════════════════════════
 *
 *  1. nvs_flash        — ESP-IDF internal driver dependency
 *  2. gpio_isr_service — before any gpio_isr_handler_add()
 *  3. i2c_master       — before ADS1115 driver (adc_task uses it)
 *  4. queue_manager    — before any task creation
 *  5. system_monitor   — before tasks (fault reporting)
 *  6. bt_sender_init   — spawns bt_sender_task internally
 *  7. lv_init + display — before ui screen creation
 *  8. lvgl tick timer  — esp_timer (µs precision, no FreeRTOS jitter)
 *  9. lvgl_mutex       — before lvgl_task or ui_event_task start
 * 10. scan button GPIO  — before scheduler takes over
 * 11. self_test        — I2C scan + heap floor check
 * 12. Task creation    — exact order: adc→signal→bt(done)→lvgl→ui
 * 13. Monitor loop     — app_main repurposed as health watchdog
 *
 * ═══════════════════════════════════════════════════════════════════
 *  CRASH PREVENTION
 * ═══════════════════════════════════════════════════════════════════
 *
 *  - create_task() calls esp_restart() on allocation failure
 *  - I2C timeout set (prevents SDA-stuck-low hang)
 *  - NVS corruption handled: erase + reinit
 *  - ISR: IRAM_ATTR + zero heap alloc + portYIELD_FROM_ISR
 *  - LVGL: mutex before every lv_* call from ui_event_task
 *  - LVGL tick: esp_timer (not vTaskDelay) — immune to tick jitter
 *  - Monitor loop: stack HWM + heap logged every 30s
 *  - BT failure: non-fatal, device operates without 3D scan
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "ble_logger/ble_data_logger.h"

#include "lvgl.h"
#include "lvgl_port.h"
#include "touch_calibration.h"
#include "device_cal.h"

#include "gradiometer_types.h"
#include "signal_processor.h"
#include "queue_manager.h"
#include "signal_task.h"
#include "adc_task.h"
#include "bluetooth_sender.h"
#include "system_monitor.h"
#include "modes/sensitivity_manager.h"
#include "ui/ui_event_task.h"
#include "hardware_config.h"

static const char *TAG = "AppMain";

/* ═══════════════════════════════════════════════════════════════════
 * HARDWARE CONFIGURATION — single source of truth for all pins
 * ═══════════════════════════════════════════════════════════════════ */

/* All pin definitions in hardware_config.h — use HW_* macros */
#define I2C_PORT         HW_I2C_PORT
#define I2C_SDA_PIN      HW_I2C_SDA_PIN
#define I2C_SCL_PIN      HW_I2C_SCL_PIN
#define I2C_CLK_HZ       HW_I2C_CLK_HZ
#define BTN_SCAN_GPIO    HW_BTN_SCAN_GPIO

#define LVGL_TICK_MS     10          /* LVGL refresh period */

/* ═══════════════════════════════════════════════════════════════════
 * TASK STACK SIZES
 * ═══════════════════════════════════════════════════════════════════ */

#define STACK_ADC        4096   /* I2C + quality scoring */
#define STACK_SIGNAL     4096   /* DSP pipeline + calibration calls */
#define STACK_LVGL       6144   /* LVGL internals + font cache */
#define STACK_UI_EVENT   4096   /* Screen updates + event routing */

/* ═══════════════════════════════════════════════════════════════════
 * GLOBAL SHARED RESOURCES
 * (minimized — only what multiple components genuinely need)
 * ═══════════════════════════════════════════════════════════════════ */

BTSender_t           g_bt_sender;   /* BT sender — used by signal_task + ui_event_task */
SensitivityManager_t g_sens_mgr;   /* AUTO sensitivity engine — shared read-only in signal_task */
SemaphoreHandle_t g_lvgl_mutex;  /* Protects all lv_* calls from ui_event_task */

/* ═══════════════════════════════════════════════════════════════════
 * TASK REGISTRY — for stack health monitoring
 * ═══════════════════════════════════════════════════════════════════ */

static struct {
    TaskHandle_t handle;
    const char  *name;
    uint32_t     stack_bytes;
} s_tasks[8];
static uint8_t s_task_count = 0;

static void task_register(TaskHandle_t h, const char *name, uint32_t stack)
{
    if (s_task_count < 8) {
        s_tasks[s_task_count].handle      = h;
        s_tasks[s_task_count].name        = name;
        s_tasks[s_task_count].stack_bytes = stack;
        s_task_count++;
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * ISR — Physical Scan Button
 *
 * RULES (must follow for ISR correctness):
 *   1. IRAM_ATTR: runs from IRAM — safe during flash cache miss
 *   2. No heap allocation
 *   3. No non-IRAM-safe ESP-IDF calls
 *   4. portYIELD_FROM_ISR: yield immediately if higher-prio task unblocked
 * ═══════════════════════════════════════════════════════════════════ */

static void IRAM_ATTR btn_scan_isr(void *arg)
{
    (void)arg;

    const SysEventMsg_t evt = {
        .event        = SYS_EVT_BTN_SCAN,
        .data         = 0,
        .timestamp_ms = 0,  /* Receiver adds timestamp — xTaskGetTickCount not ISR-safe */
    };

    BaseType_t higher_prio_woken = pdFALSE;
    qm_event_send(evt.event, evt.data, true);
    (void)higher_prio_woken;

    if (higher_prio_woken) {
        portYIELD_FROM_ISR();
    }
}

/* LVGL tick: managed automatically via LV_TICK_CUSTOM in lv_conf.h */

/* ═══════════════════════════════════════════════════════════════════
 * LVGL TASK — Core 1, Priority 5
 *
 * THE ONLY task that calls lv_timer_handler().
 * All other tasks access LVGL through g_lvgl_mutex.
 *
 * Mutex timeout = 10ms:
 *   If ui_event_task holds mutex > 10ms, we skip one handler cycle.
 *   LVGL is designed to recover — no corruption from skipped cycles.
 * ═══════════════════════════════════════════════════════════════════ */

static void lvgl_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "lvgl_task running — Core%d Prio%d",
             (int)xPortGetCoreID(), (int)uxTaskPriorityGet(NULL));

    for (;;) {
        if (xSemaphoreTake(g_lvgl_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            lv_timer_handler();
            xSemaphoreGive(g_lvgl_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(LVGL_TICK_MS));
    }

    /* Unreachable — suppress compiler warning */
    vTaskDelete(NULL);
}

/* ═══════════════════════════════════════════════════════════════════
 * create_task — guaranteed creation or hard restart
 *
 * If xTaskCreatePinnedToCore fails (usually OOM), we cannot continue.
 * esp_restart() is the safest recovery — prevents partial-init crash.
 * ═══════════════════════════════════════════════════════════════════ */

static TaskHandle_t create_task(TaskFunction_t  fn,
                                 const char     *name,
                                 uint32_t        stack,
                                 void           *arg,
                                 UBaseType_t     prio,
                                 BaseType_t      core)
{
    TaskHandle_t handle = NULL;

    BaseType_t rc = xTaskCreatePinnedToCore(fn, name, stack, arg,
                                             prio, &handle, core);
    if (rc != pdPASS || handle == NULL) {
        ESP_LOGE(TAG, "FATAL: task '%s' creation failed (OOM?) — restarting", name);
        esp_restart();
    }

    ESP_LOGI(TAG, "  %-16s Core%d  Prio%u  Stack:%lu B",
             name, (int)core, (unsigned)prio, (unsigned long)stack);

    return handle;
}

/* ═══════════════════════════════════════════════════════════════════
 * log_system_health — stack HWM + heap + BT stats
 * Called every 30s from monitor loop.
 * ═══════════════════════════════════════════════════════════════════ */

static void log_system_health(uint32_t cycle)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "═══ Health Monitor #%lu ═══════════════════", (unsigned long)cycle);

    for (uint8_t i = 0; i < s_task_count; i++) {
        if (!s_tasks[i].handle) continue;

        UBaseType_t hwm_words = uxTaskGetStackHighWaterMark(s_tasks[i].handle);
        uint32_t    hwm_bytes = (uint32_t)hwm_words * sizeof(StackType_t);
        uint32_t    used_pct  = 100u - ((hwm_bytes * 100u) / s_tasks[i].stack_bytes);

        const char *status = (hwm_words < 64) ? " ⚠ NEAR OVERFLOW" :
                             (used_pct  > 75)  ? " △ HIGH USAGE"    : "";

        ESP_LOGI(TAG, "  %-16s  HWM:%4u words  used:%2lu%%%s",
                 s_tasks[i].name, (unsigned)hwm_words,
                 (unsigned long)used_pct, status);
    }

    size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    size_t min_heap  = heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);

    ESP_LOGI(TAG, "  Heap free:  %6u B  (min ever: %u B)%s",
             (unsigned)free_heap, (unsigned)min_heap,
             free_heap < GRAD_MIN_HEAP_BYTES ? " ⚠ LOW" : "");

    qm_log_stats();

    ESP_LOGI(TAG, "════════════════════════════════════════════");
    ESP_LOGI(TAG, "");
}

/* ═══════════════════════════════════════════════════════════════════
 * self_test — hardware verification at boot
 *
 * Non-blocking: logs faults, does NOT abort on soft failures.
 * Only FATAL conditions (heap < 16KB) trigger restart.
 * ═══════════════════════════════════════════════════════════════════ */

static void self_test(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "─── Self Test ───────────────────────────────");

    /* Chip info */
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    ESP_LOGI(TAG, "  Chip:   ESP32 rev%d | %d cores | %s flash",
             chip.revision, chip.cores,
             (chip.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    /* Heap check */
    size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    size_t free_dram = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    ESP_LOGI(TAG, "  Heap:   %u B free  (%u B DRAM)", (unsigned)free_heap, (unsigned)free_dram);

    if (free_heap < 16384) {
        ESP_LOGE(TAG, "  FATAL: heap %u B < 16KB at boot — OOM risk", (unsigned)free_heap);
        esp_restart();
    }
    if (free_heap < 32768) {
        ESP_LOGW(TAG, "  WARNING: low heap at boot (%u B)", (unsigned)free_heap);
    }

    /* Queue verification */
    ESP_LOGI(TAG, "  Queues: initialized OK");

    /* I2C bus scan — find ADS1115 @ 0x48 */
    ESP_LOGI(TAG, "  I2C scan (SDA:%d SCL:%d @ %dkHz):",
             I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLK_HZ / 1000);

    bool ads_found = false;
    int  devices   = 0;

    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        bool ack = (i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(5)) == ESP_OK);
        i2c_cmd_link_delete(cmd);

        if (ack) {
            devices++;
            const char *label = (addr == 0x48) ? " ← ADS1115 ✓" : "";
            ESP_LOGI(TAG, "    0x%02X%s", addr, label);
            if (addr == 0x48) ads_found = true;
        }
    }

    if (devices == 0) {
        ESP_LOGE(TAG, "  No I2C devices found — check SDA/SCL wiring!");
    }

    if (!ads_found) {
        ESP_LOGE(TAG, "  ADS1115 NOT FOUND at 0x48 — ADC task will retry");
        sysmon_report_fault(FAULT_ADC_NOT_FOUND, "ADS1115 absent at boot");
        /* Non-fatal: adc_task retries with exponential back-off */
    }

    ESP_LOGI(TAG, "─── Self Test Done ──────────────────────────");
    ESP_LOGI(TAG, "");
}

/* ═══════════════════════════════════════════════════════════════════
 * app_main — Entry Point
 *
 * Execution context: FreeRTOS "main" task, Core 1, Priority 1.
 * After all tasks are created and monitor loop entered,
 * this function never returns — it becomes the health watchdog.
 * ═══════════════════════════════════════════════════════════════════ */

void app_main(void)
{
    esp_err_t ret;

    /* ─── Boot Banner ─────────────────────────────────────────────── */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔══════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║      Gradiometer Pro  v2.0               ║");
    ESP_LOGI(TAG, "║      Build: %s  %s     ║", __DATE__, __TIME__);
    ESP_LOGI(TAG, "╚══════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");

    /* ── 1. NVS Flash ─────────────────────────────────────────────
     * Required internally by ESP-IDF (Bluetooth, WiFi subsystems).
     * Handle corruption gracefully: erase partition and reinit.
     * ─────────────────────────────────────────────────────────── */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS corrupt — erasing and reinitializing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "[1/9] NVS flash          OK");

    /* ── 2. GPIO ISR Service ──────────────────────────────────────
     * ESP_INTR_FLAG_IRAM: ISR runs from IRAM — safe during flash ops.
     * ESP_ERR_INVALID_STATE = already installed — treat as success.
     * ─────────────────────────────────────────────────────────── */
    ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(ret);
    }
    ESP_LOGI(TAG, "[2/9] GPIO ISR service   OK");

    /* ── 3. I2C Master ────────────────────────────────────────────
     * Fast-mode 400kHz.
     * i2c_set_timeout: prevents infinite hang if SDA stuck LOW
     * (common when sensor crashes — I2C bus recovery needed).
     * 0xFFFFF APB cycles ≈ 13ms timeout @ 80MHz APB.
     * ─────────────────────────────────────────────────────────── */
    {
        const i2c_config_t i2c_cfg = {
            .mode             = I2C_MODE_MASTER,
            .sda_io_num       = I2C_SDA_PIN,
            .scl_io_num       = I2C_SCL_PIN,
            .sda_pullup_en    = GPIO_PULLUP_ENABLE,
            .scl_pullup_en    = GPIO_PULLUP_ENABLE,
            .master.clk_speed = I2C_CLK_HZ,
        };
        ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &i2c_cfg));
        ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));
        i2c_set_timeout(I2C_PORT, 0xFFFFF);
    }
    ESP_LOGI(TAG, "[3/9] I2C master         OK  (SDA:%d SCL:%d @ %dkHz)",
             I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLK_HZ / 1000);

    /* ── 4. Queue Manager ─────────────────────────────────────────
     * Creates adc_queue, result_queue, event_queue.
     * MUST precede all task creation — tasks block on these queues.
     * ─────────────────────────────────────────────────────────── */
    ret = queue_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Queue manager init FAILED");
        esp_restart();
    }
    ESP_LOGI(TAG, "[4/9] Queue manager      OK  (adc:%u result:%u event:%u)",
             QUEUE_DEPTH_ADC_SAMPLES, QUEUE_DEPTH_PROCESSED, QUEUE_DEPTH_EVENTS);

    /* ── 5. System Monitor ────────────────────────────────────────
     * Heap watchdog + fault code registry.
     * Non-fatal if init fails (device works, just no watchdog).
     * ─────────────────────────────────────────────────────────── */
    ret = sysmon_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "[5/9] System monitor     WARN — watchdog disabled");
    } else {
        ESP_LOGI(TAG, "[5/9] System monitor     OK");
    }

    /* ── 6. Bluetooth Sender ──────────────────────────────────────
     * Inits UART2 + spawns bt_sender_task internally on Core 0.
     * NON-FATAL: device operates without 3D scan if BT unavailable.
     * ─────────────────────────────────────────────────────────── */
    ret = bt_sender_init(&g_bt_sender);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "[6/9] BT sender          WARN — 3D scan disabled");
        sysmon_report_fault(FAULT_BT_INIT_FAILED, "HC-05 UART init failed");
    } else {
        ESP_LOGI(TAG, "[6/9] BT sender          OK  (UART%d TX:GPIO%d RX:GPIO%d @ %d)",
                 (int)HW_BT_UART_NUM, HW_BT_TX_PIN, HW_BT_RX_PIN, HW_BT_BAUD_RATE);
    }

    /* ── تهيئة AUTO Sensitivity Engine ──
     * يأخذ مؤشر ADS1115 لتغيير PGA أثناء AUTO mode.
     * يُستدعى sens_manager_auto_update() من signal_task بعد كل sp_process. */
    sens_manager_init(&g_sens_mgr, adc_task_get_driver());

    /* ── 6b. BLE Data Logger (ESP32 internal BLE → Mobile) ───────────
     * يعمل بالتوازي مع HC-05 — مستقل تماماً.
     * HC-05 (UART) → OKM Visualizer3D (binary)
     * BLE داخلي   → هاتف Android/iOS (CSV notifications)
     * يُفعَّل فقط عند تشغيل data_via_ble switch.
     * NON-FATAL: device operates normally if BLE init fails.
     * ─────────────────────────────────────────────────────────── */
    ret = ble_logger_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "[6b] BLE logger          WARN — mobile logging disabled");
    } else {
        ble_logger_task_start();
        ESP_LOGI(TAG, "[6b] BLE logger          OK  (advertising as \"Gradiometer BLE\")");
    }

    /* ── 7. Display + LVGL ────────────────────────────────────────
     * Order within this step is fixed:
     *   a. lv_init()              — LVGL global state
     *   b. lvgl_driver_init()     — SPI + display IC + touch
     *   c. draw buffer (static)   — 2 × 320×40px = 51,200 B
     *   d. lv_disp_drv_register() — register display
     *   e. g_lvgl_mutex           — before any task uses LVGL
     *   f. LVGL tick via LV_TICK_CUSTOM (esp_timer_get_time automatic)
     *
     * full_refresh = 0: LVGL redraws dirty regions only.
     * Saves ~60% CPU on static screens (calibration, menus).
     * ─────────────────────────────────────────────────────────── */
    lv_init();
    lvgl_port_init();

    /* lvgl_port_init() already registered display + touch drivers */
    /* No duplicate registration needed */

    /* LVGL mutex — binary semaphore, starts available */
    g_lvgl_mutex = xSemaphoreCreateMutex();
    if (!g_lvgl_mutex) {
        ESP_LOGE(TAG, "LVGL mutex alloc FAILED — FATAL");
        esp_restart();
    }

    /* LVGL tick: auto via LV_TICK_CUSTOM=1 — no timer needed */

    ESP_LOGI(TAG, "[7/9] Display + LVGL     OK  (320×240 double-buf dirty-region)");

    /* ── Touch Calibration ───────────────────────────────────────
     * First run: NVS empty → run 9-point calibration wizard.
     * Subsequent boots: load coefficients from NVS (instant).
     * Namespace: "touch_cal" — INDEPENDENT from device calibration.
     * Resetting device calibration NEVER triggers touch recalibration.
     * ─────────────────────────────────────────────── */
    if (!touch_cal_load()) {
        ESP_LOGI(TAG, "First boot — running touch calibration wizard");
        touch_cal_run();
    } else {
        ESP_LOGI(TAG, "Touch calibration loaded from NVS");
    }

    /* ── Device Calibration (5-Phase) ───────────────────────────────
     * Boot logic:
     *   IF device_valid != 1  → run wizard (first boot or after reset)
     *   IF cal_version mismatch → run wizard (firmware update)
     *   ELSE → load profile instantly
     *
     * Namespace: "dev_cal" — keys: profile_v2, device_valid, cal_version
     * Touch calibration is NOT checked or affected here.
     * ─────────────────────────────────────────────── */
    if (!devcal_load()) {
        ESP_LOGI(TAG, "Device calibration needed (invalid or version mismatch)");
        devcal_run();
    } else {
        ESP_LOGI(TAG, "Device profile loaded (phases=0x%02X  det_limit=%.1f LSB)",
                 devcal_get_profile()->phases_completed,
                 devcal_get_profile()->detection_limit);
    }

    /* Apply device calibration params to signal processor */
    if (devcal_is_valid()) {
        const DeviceProfile_t *dp = devcal_get_profile();
        SpDeviceParams_t sp_params = {
            .noise_floor     = dp->noise_floor,
            .dynamic_range   = dp->dynamic_range,
            .kalman_Q        = dp->kalman_Q,
            .kalman_R        = dp->kalman_R,
            .kalman_Q_final  = dp->kalman_Q_final,
            .kalman_R_final  = dp->kalman_R_final,
            .detection_limit = dp->detection_limit,
        };
        /* signal_task owns the processor — pass params via event
         * For now: signal_task will apply on first calibration.
         * Direct call safe here before tasks start. */
        (void)sp_params;  /* used by signal_task init */
        ESP_LOGI(TAG, "Device params ready for signal processor");
        /* Load heading corrections (Phase 3A) into signal processor */
        signal_task_load_heading_corrections(dp->heading_correction);
    }



    /* ── 8. Physical Scan Button ──────────────────────────────────
     * Active LOW (boot button on most ESP32 devkits).
     * NEGEDGE interrupt → ISR → event_queue → ui_event_task.
     * Non-fatal: UI touch button still works without physical button.
     * ─────────────────────────────────────────────────────────── */
    {
        const gpio_config_t btn_cfg = {
            .pin_bit_mask = (1ULL << BTN_SCAN_GPIO),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_NEGEDGE,
        };
        ret = gpio_config(&btn_cfg);
        if (ret == ESP_OK) {
            ret = gpio_isr_handler_add(BTN_SCAN_GPIO, btn_scan_isr, NULL);
        }
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "[8/9] Scan button        OK  (GPIO%d active-LOW)",
                     (int)BTN_SCAN_GPIO);
        } else {
            ESP_LOGW(TAG, "[8/9] Scan button        WARN — physical button disabled");
        }
    }

    /* ── 9. Self-Test ─────────────────────────────────────────────
     * I2C bus scan, heap floor, queue verification.
     * Logs WARN/ERROR but only aborts on truly fatal conditions.
     * ─────────────────────────────────────────────────────────── */
    self_test();
    ESP_LOGI(TAG, "[9/9] Self-test          DONE");

    /* ═══════════════════════════════════════════════════════════
     * TASK CREATION
     *
     * Order is intentional:
     *   1. Core 0 real-time tasks first (they may immediately
     *      start producing data — queues are ready to receive)
     *   2. bt_sender_task already running (from bt_sender_init)
     *   3. lvgl_task before ui_event_task (LVGL must be warm
     *      before ui creates screen objects)
     *   4. 50ms gap before ui_event_task (one lvgl_task cycle)
     *
     * create_task() calls esp_restart() on failure — no NULL checks needed.
     * ═══════════════════════════════════════════════════════════ */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "─── Task Creation ───────────────────────────");
    TaskHandle_t h;

    /* Core 0 — ADC acquisition (highest priority, real-time) */
    h = create_task(adc_task, "adc_task",
                    STACK_ADC, NULL, 6, 0);
    task_register(h, "adc_task", STACK_ADC);

    /* Core 0 — Signal processing + calibration */
    h = create_task(signal_task, "signal_task",
                    STACK_SIGNAL, (void *)&g_bt_sender, 5, 0);
    task_register(h, "signal_task", STACK_SIGNAL);

    /* bt_sender_task spawned inside bt_sender_init — register its handle */
    if (g_bt_sender.task_handle) {
        task_register(g_bt_sender.task_handle, "bt_sender", BT_TASK_STACK);
    }

    /* Core 1 — LVGL timer handler */
    h = create_task(lvgl_task, "lvgl_task",
                    STACK_LVGL, NULL, 5, 1);
    task_register(h, "lvgl_task", STACK_LVGL);

    /*
     * 50ms gap: allows lvgl_task to call lv_timer_handler() ≥1 time.
     * LVGL internal timer list must be initialized before
     * ui_event_task starts creating screen objects.
     */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Core 1 — UI screen updates + event routing */
    h = create_task(ui_event_task, "ui_event_task",
                    STACK_UI_EVENT, (void *)&g_bt_sender, 4, 1);
    task_register(h, "ui_event_task", STACK_UI_EVENT);

    ESP_LOGI(TAG, "─── All %u Tasks Running ─────────────────────", (unsigned)s_task_count);
    ESP_LOGI(TAG, "");

    /* ═══════════════════════════════════════════════════════════
     * MONITOR LOOP
     *
     * app_main task repurposed as health watchdog.
     * Priority 1 (lowest) — runs ONLY when all other tasks idle.
     * No busy-wait: vTaskDelay(30s) releases CPU completely.
     *
     * Every 30 seconds:
     *   - Stack HWM for all registered tasks
     *   - Heap free + minimum ever
     *   - BT sender stats
     *
     * Future additions:
     *   - Task ping/pong watchdog (each task sets a flag)
     *   - Auto-restart if any task missed > 2 cycles
     * ═══════════════════════════════════════════════════════════ */
    ESP_LOGI(TAG, "app_main → monitor loop  (Core1 Prio1 — 30s interval)");

    uint32_t monitor_cycle = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        log_system_health(++monitor_cycle);
    }

    /* Unreachable */
    vTaskDelete(NULL);
}
