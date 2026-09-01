/**
 * @file system_monitor.c
 * @brief System health monitor — implementation.
 */

#include "system_monitor.h"
#include "core/queue_manager.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "SysMon";

/* =========================================================================
 * PRIVATE STATE
 * ========================================================================= */

#define MAX_MONITORED_TASKS     8u
#define MONITOR_INTERVAL_MS     5000u   ///< Check interval (5 seconds)
#define HEAP_CRITICAL_BYTES     4096u   ///< Emergency threshold — fault now

typedef struct {
    TaskHandle_t handle;
    char         name[16];
    uint32_t     stack_warning_bytes;
} MonitoredTask_t;

static struct {
    MonitoredTask_t tasks[MAX_MONITORED_TASKS];
    uint8_t         task_count;
    FaultCode_t     active_faults;
    uint32_t        check_count;
    bool            initialized;
} s_mon = {0};

/* =========================================================================
 * TASK REGISTRY
 * ========================================================================= */

void sysmon_register_task(TaskHandle_t handle, const char *name,
                          uint32_t stack_warning_bytes)
{
    if (s_mon.task_count >= MAX_MONITORED_TASKS) {
        ESP_LOGW(TAG, "Task registry full — cannot monitor '%s'", name);
        return;
    }

    MonitoredTask_t *entry = &s_mon.tasks[s_mon.task_count++];
    entry->handle              = handle;
    entry->stack_warning_bytes = stack_warning_bytes;
    strlcpy(entry->name, name, sizeof(entry->name));

    ESP_LOGI(TAG, "Registered task '%s' for monitoring (warn at %lu bytes stack)",
             name, (unsigned long)stack_warning_bytes);
}

/* =========================================================================
 * INITIALIZATION
 * ========================================================================= */

esp_err_t sysmon_init(void)
{
    memset(&s_mon, 0, sizeof(s_mon));
    s_mon.initialized = true;
    ESP_LOGI(TAG, "System monitor initialized");
    return ESP_OK;
}

/* =========================================================================
 * HEALTH CHECK INTERNALS
 * ========================================================================= */

static void check_heap(void)
{
    size_t free_heap = esp_get_free_heap_size();
    size_t min_heap  = esp_get_minimum_free_heap_size();

    if (free_heap < HEAP_CRITICAL_BYTES) {
        ESP_LOGE(TAG, "CRITICAL: Free heap %u bytes — system at risk!", (unsigned)free_heap);
        sysmon_report_fault(FAULT_LOW_MEMORY, "Heap critically low");
    } else if (free_heap < GRAD_MIN_HEAP_BYTES) {
        ESP_LOGW(TAG, "Low heap: %u bytes free (min ever: %u)", 
                 (unsigned)free_heap, (unsigned)min_heap);
    } else {
        ESP_LOGI(TAG, "Heap: %u free, %u min-ever", 
                 (unsigned)free_heap, (unsigned)min_heap);
    }
}

static void check_stacks(void)
{
    for (uint8_t i = 0; i < s_mon.task_count; i++) {
        MonitoredTask_t *t = &s_mon.tasks[i];
        if (!t->handle) continue;

        UBaseType_t remaining = uxTaskGetStackHighWaterMark(t->handle);
        uint32_t remaining_bytes = remaining * sizeof(StackType_t);

        if (remaining_bytes < t->stack_warning_bytes) {
            ESP_LOGW(TAG, "Stack LOW — task '%s': %lu bytes remaining",
                     t->name, (unsigned long)remaining_bytes);
        } else {
            ESP_LOGI(TAG, "Stack '%s': %lu bytes remaining",
                     t->name, (unsigned long)remaining_bytes);
        }
    }
}

static void check_queues(void)
{
    uint32_t adc_waiting = qm_adc_waiting();
    uint32_t drop_count  = qm_get_adc_drop_count();

    if (drop_count > 0) {
        ESP_LOGW(TAG, "ADC drops: %lu (queue backpressure!)", (unsigned long)drop_count);
        sysmon_report_fault(FAULT_QUEUE_OVERFLOW, "ADC queue overflow");
    }

    if (adc_waiting >= QUEUE_DEPTH_ADC_SAMPLES) {
        ESP_LOGW(TAG, "ADC queue full (%lu/%u) — Signal task too slow?",
                 (unsigned long)adc_waiting, QUEUE_DEPTH_ADC_SAMPLES);
    }
}

/* =========================================================================
 * TASK ENTRY POINT
 * ========================================================================= */

void sysmon_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Monitor task started on Core %d", (int)xPortGetCoreID());

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        s_mon.check_count++;

        ESP_LOGI(TAG, "=== Health Check #%lu ===", (unsigned long)s_mon.check_count);

        check_heap();
        check_stacks();
        check_queues();

        if (s_mon.active_faults != FAULT_NONE) {
            ESP_LOGW(TAG, "Active faults: 0x%02X", (unsigned)s_mon.active_faults);
        }

        qm_log_stats();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MONITOR_INTERVAL_MS));
    }
}

/* =========================================================================
 * FAULT MANAGEMENT
 * ========================================================================= */

void sysmon_report_fault(FaultCode_t code, const char *msg)
{
    /* OR in the new fault — faults accumulate until explicitly cleared */
    s_mon.active_faults = (FaultCode_t)(s_mon.active_faults | code);

    ESP_LOGE(TAG, "FAULT [0x%02X]: %s", (unsigned)code, msg ? msg : "(no message)");

    /* Notify other tasks via event queue */
    qm_event_send(SYS_EVT_FAULT, (uint32_t)code);
}

FaultCode_t sysmon_get_faults(void)
{
    return s_mon.active_faults;
}

void sysmon_clear_fault(FaultCode_t code)
{
    s_mon.active_faults = (FaultCode_t)(s_mon.active_faults & ~code);
    ESP_LOGI(TAG, "Fault 0x%02X cleared", (unsigned)code);
}

bool sysmon_has_fault(void)
{
    return s_mon.active_faults != FAULT_NONE;
}
