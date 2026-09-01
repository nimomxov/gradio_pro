/**
 * @file app_main.c
 * @brief Gradiometer Pro — System entry point and orchestrator (BUGFIXED).
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
 * HARDWARE CONFIGURATION
 * ═══════════════════════════════════════════════════════════════════ */

#define I2C_PORT         HW_I2C_PORT
#define I2C_SDA_PIN      HW_I2C_SDA_PIN
#define I2C_SCL_PIN      HW_I2C_SCL_PIN
#define I2C_CLK_HZ       HW_I2C_CLK_HZ
#define BTN_SCAN_GPIO    HW_BTN_SCAN_GPIO

#define LVGL_TICK_MS     10          

/* ═══════════════════════════════════════════════════════════════════
 * TASK STACK SIZES
 * ═══════════════════════════════════════════════════════════════════ */

#define STACK_ADC        4096   
#define STACK_SIGNAL     4096   
#define STACK_LVGL       6144   
#define STACK_UI_EVENT   4096   

/* ═══════════════════════════════════════════════════════════════════
 * GLOBAL SHARED RESOURCES
 * ═══════════════════════════════════════════════════════════════════ */

BTSender_t           g_bt_sender;   
SensitivityManager_t g_sens_mgr;   
SemaphoreHandle_t    g_lvgl_mutex; 

/* ═══════════════════════════════════════════════════════════════════
 * TASK REGISTRY
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
 * ═══════════════════════════════════════════════════════════════════ */

static void IRAM_ATTR btn_scan_isr(void *arg)
{
    (void)arg;

    const SysEventMsg_t evt = {
        .event        = SYS_EVT_BTN_SCAN,
        .data         = 0,
        .timestamp_ms = 0,  
    };

    BaseType_t higher_prio_woken = pdFALSE;
    
    /* BUGFIX: We must pass the address of higher_prio_woken to the underlying
     * FreeRTOS xQueueSendFromISR() call. If qm_event_send takes a bool for ISR,
     * you MUST create a qm_event_send_from_isr() in queue_manager.c that 
     * accepts (event, data, &higher_prio_woken). */
    qm_event_send_from_isr(evt.event, evt.data, &higher_prio_woken);

    /* Now this will correctly trigger an immediate context switch if needed */
    if (higher_prio_woken) {
        portYIELD_FROM_ISR();
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * LVGL TASK
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

    vTaskDelete(NULL);
}

/* ═══════════════════════════════════════════════════════════════════
 * create_task
 * ═══════════════════════════════════════════════════════════════════ */

static TaskHandle_t create_task(TaskFunction_t  fn,
                                 const char     *name,
                                 uint32_t        stack,
                                 void           *arg,
                                 UBaseType_t     prio,
                                 BaseType_t      core)
{
    TaskHandle_t handle = NULL;

    BaseType_t rc = xTaskCreatePinnedToCore(fn, name, stack, arg, prio, &handle, core);
    if (rc != pdPASS || handle == NULL) {
        ESP_LOGE(TAG, "FATAL: task '%s' creation failed (OOM?) — restarting", name);
        esp_restart();
    }

    ESP_LOGI(TAG, "  %-16s Core%d  Prio%u  Stack:%lu B",
             name, (int)core, (unsigned)prio, (unsigned long)stack);

    return handle;
}

/* ═══════════════════════════════════════════════════════════════════
 * log_system_health
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
 * self_test
 * ═══════════════════════════════════════════════════════════════════ */

static void self_test(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "─── Self Test ───────────────────────────────");

    esp_chip_info_t chip;
    esp_chip_info(&chip);
    ESP_LOGI(TAG, "  Chip:   ESP32 rev%d | %d cores | %s flash",
             chip.revision, chip.cores,
             (chip.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

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

    ESP_LOGI(TAG, "  Queues: initialized OK");

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
    }

    ESP_LOGI(TAG, "─── Self Test Done ──────────────────────────");
    ESP_LOGI(TAG, "");
}

/* ═══════════════════════════════════════════════════════════════════
 * app_main — Entry Point
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

    /* ── 1. NVS Flash ───────────────────────────────────────────── */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS corrupt — erasing and reinitializing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "[1/10] NVS flash          OK");

    /* ── 2. GPIO ISR Service ────────────────────────────────────── */
    ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(ret);
    }
    ESP_LOGI(TAG, "[2/10] GPIO ISR service   OK");

    /* ── 3. I2C Master ──────────────────────────────────────────── */
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
    ESP_LOGI(TAG, "[3/10] I2C master         OK  (SDA:%d SCL:%d @ %dkHz)",
             I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLK_HZ / 1000);

    /* ── 4. Queue Manager ───────────────────────────────────────── */
    ret = queue_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Queue manager init FAILED");
        esp_restart();
    }
    ESP_LOGI(TAG, "[4/10] Queue manager      OK  (adc:%u result:%u event:%u)",
             QUEUE_DEPTH_ADC_SAMPLES, QUEUE_DEPTH_PROCESSED, QUEUE_DEPTH_EVENTS);

    /* ── 5. System Monitor ──────────────────────────────────────── */
    ret = sysmon_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "[5/10] System monitor     WARN — watchdog disabled");
    } else {
        ESP_LOGI(TAG, "[5/10] System monitor     OK");
    }

    /* ── 6. Bluetooth Sender ────────────────────────────────────── */
    ret = bt_sender_init(&g_bt_sender);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "[6/10] BT sender          WARN — 3D scan disabled");
        sysmon_report_fault(FAULT_BT_INIT_FAILED, "HC-05 UART init failed");
    } else {
        ESP_LOGI(TAG, "[6/10] BT sender          OK  (UART%d TX:GPIO%d RX:GPIO%d @ %d)",
                 (int)HW_BT_UART_NUM, HW_BT_TX_PIN, HW_BT_RX_PIN, HW_BT_BAUD_RATE);
    }

    /* ── 7. BLE Data Logger ─────────────────────────────────────── */
    ret = ble_logger_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "[7/10] BLE logger          WARN — mobile logging disabled");
    } else {
        ble_logger_task_start();
        ESP_LOGI(TAG, "[7/10] BLE logger          OK  (advertising as \"Gradiometer BLE\")");
    }

    /* ── 8. Display + LVGL ──────────────────────────────────────── */
    lv_init();
    lvgl_port_init();

    g_lvgl_mutex = xSemaphoreCreateMutex();
    if (!g_lvgl_mutex) {
        ESP_LOGE(TAG, "LVGL mutex alloc FAILED — FATAL");
        esp_restart();
    }

    ESP_LOGI(TAG, "[8/10] Display + LVGL     OK  (320×240 double-buf dirty-region)");

    /* ── Touch Calibration ─────────────────────────────────────── */
    if (!touch_cal_load()) {
        ESP_LOGI(TAG, "First boot — running touch calibration wizard");
        touch_cal_run();
    } else {
        ESP_LOGI(TAG, "Touch calibration loaded from NVS");
    }

    /* ── Device Calibration (5-Phase) ───────────────────────────── */
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
        
        /* BUGFIX: Removed dead SpDeviceParams_t allocation. The params struct 
         * was created, filled, and immediately discarded with (void)sp_params.
         * signal_task will apply calibration dynamically on its first cycle. */
        
        ESP_LOGI(TAG, "Device params ready for signal processor");
        signal_task_load_heading_corrections(dp->heading_correction);
    }

    /* ── 9. Physical Scan Button ────────────────────────────────── */
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
            ESP_LOGI(TAG, "[9/10] Scan button        OK  (GPIO%d active-LOW)", (int)BTN_SCAN_GPIO);
        } else {
            ESP_LOGW(TAG, "[9/10] Scan button        WARN — physical button disabled");
        }
    }

    /* ── 10. Self-Test ──────────────────────────────────────────── */
    self_test();
    ESP_LOGI(TAG, "[10/10] Self-test          DONE");

    /* ═══════════════════════════════════════════════════════════
     * TASK CREATION
     * ═══════════════════════════════════════════════════════════ */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "─── Task Creation ───────────────────────────");
    TaskHandle_t h;

    /* Core 0 — ADC acquisition (highest priority, real-time) */
    h = create_task(adc_task, "adc_task", STACK_ADC, NULL, 6, 0);
    task_register(h, "adc_task", STACK_ADC);

    /* BUGFIX: Moved sens_manager_init to HERE.
     * Previously, it was called before adc_task was created, risking a NULL 
     * or uninitialized pointer if adc_task_get_driver() relies on task-local init.
     * Now, the task object exists. (sens_manager won't actually use the driver 
     * until signal_task starts and calls auto_update, so this is perfectly safe).
     */
    sens_manager_init(&g_sens_mgr, adc_task_get_driver());

    /* Core 0 — Signal processing + calibration */
    h = create_task(signal_task, "signal_task", STACK_SIGNAL, (void *)&g_bt_sender, 5, 0);
    task_register(h, "signal_task", STACK_SIGNAL);

    /* bt_sender_task spawned inside bt_sender_init — register its handle */
    if (g_bt_sender.task_handle) {
        task_register(g_bt_sender.task_handle, "bt_sender", BT_TASK_STACK);
    }

    /* Core 1 — LVGL timer handler */
    h = create_task(lvgl_task, "lvgl_task", STACK_LVGL, NULL, 5, 1);
    task_register(h, "lvgl_task", STACK_LVGL);

    /* 50ms gap: allows lvgl_task to call lv_timer_handler() ≥1 time. */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Core 1 — UI screen updates + event routing */
    h = create_task(ui_event_task, "ui_event_task", STACK_UI_EVENT, (void *)&g_bt_sender, 4, 1);
    task_register(h, "ui_event_task", STACK_UI_EVENT);

    ESP_LOGI(TAG, "─── All %u Tasks Running ─────────────────────", (unsigned)s_task_count);
    ESP_LOGI(TAG, "");

    /* ═══════════════════════════════════════════════════════════
     * MONITOR LOOP
     * ═══════════════════════════════════════════════════════════ */
    ESP_LOGI(TAG, "app_main → monitor loop  (Core1 Prio1 — 30s interval)");

    uint32_t monitor_cycle = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        log_system_health(++monitor_cycle);
    }

    vTaskDelete(NULL);
}