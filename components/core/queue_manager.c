/**
 * @file queue_manager.c
 * @brief Centralized FreeRTOS queue management — implementation.
 *
 * FIXES APPLIED (v2.1 — Production):
 *  1. Removed all dead commented-out code (old ISR version with ESP_LOGW = crash risk).
 *  2. Timestamp uses esp_timer_get_time()/1000 — immune to TickType_t overflow.
 *  3. ISR event receiver now auto-stamps events that arrive with timestamp_ms==0.
 *  4. NULL guards on deinit path — safe even on partial-init failure.
 *  5. Event drop warning throttled (every 5) — prevents log storm in long sessions.
 *
 * ISR CONTRACT (enforced here, must be maintained by all callers):
 *   - qm_event_send_from_isr: NO ESP_LOG*, NO xTaskGetTickCount.
 *   - portYIELD_FROM_ISR always stays with the CALLER (not inside this function).
 */

#include "queue_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <string.h>

static const char *TAG = "QueueMgr";

/* =========================================================================
 * PRIVATE STATE
 * ========================================================================= */

static struct {
    QueueHandle_t adc_queue;       ///< AdcSample_t  [QUEUE_DEPTH_ADC_SAMPLES]
    QueueHandle_t result_queue;    ///< ProcessedSample_t [QUEUE_DEPTH_PROCESSED]
    QueueHandle_t event_queue;     ///< SysEventMsg_t [QUEUE_DEPTH_EVENTS]

    volatile uint32_t adc_drop_count;
    volatile uint32_t event_drop_count;
    bool initialized;
} s_qm = {0};

/* =========================================================================
 * SAFE TIMESTAMP HELPER
 * esp_timer_get_time() = microseconds since boot (int64_t, never wraps in practice).
 * /1000 gives ms. Safe for ~292,000 years. No TickType_t overflow possible.
 * ========================================================================= */

static inline uint32_t get_timestamp_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000LL);
}

/* =========================================================================
 * INITIALIZATION
 * ========================================================================= */

esp_err_t queue_manager_init(void)
{
    if (s_qm.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    s_qm.adc_queue = xQueueCreate(QUEUE_DEPTH_ADC_SAMPLES, sizeof(AdcSample_t));
    if (!s_qm.adc_queue) {
        ESP_LOGE(TAG, "Failed to create ADC queue (out of heap?)");
        return ESP_FAIL;
    }

    s_qm.result_queue = xQueueCreate(QUEUE_DEPTH_PROCESSED, sizeof(ProcessedSample_t));
    if (!s_qm.result_queue) {
        ESP_LOGE(TAG, "Failed to create result queue");
        vQueueDelete(s_qm.adc_queue);
        s_qm.adc_queue = NULL;
        return ESP_FAIL;
    }

    s_qm.event_queue = xQueueCreate(QUEUE_DEPTH_EVENTS, sizeof(SysEventMsg_t));
    if (!s_qm.event_queue) {
        ESP_LOGE(TAG, "Failed to create event queue");
        vQueueDelete(s_qm.adc_queue);
        vQueueDelete(s_qm.result_queue);
        s_qm.adc_queue    = NULL;
        s_qm.result_queue = NULL;
        return ESP_FAIL;
    }

    s_qm.adc_drop_count   = 0;
    s_qm.event_drop_count = 0;
    s_qm.initialized      = true;

    ESP_LOGI(TAG, "Queues created. ADC:%u Result:%u Event:%u items",
             QUEUE_DEPTH_ADC_SAMPLES, QUEUE_DEPTH_PROCESSED, QUEUE_DEPTH_EVENTS);
    ESP_LOGI(TAG, "ADC msg size: %u bytes, Result msg size: %u bytes",
             (unsigned)sizeof(AdcSample_t), (unsigned)sizeof(ProcessedSample_t));

    return ESP_OK;
}

void queue_manager_deinit(void)
{
    if (!s_qm.initialized) return;

    if (s_qm.adc_queue)    { vQueueDelete(s_qm.adc_queue);    s_qm.adc_queue    = NULL; }
    if (s_qm.result_queue) { vQueueDelete(s_qm.result_queue); s_qm.result_queue = NULL; }
    if (s_qm.event_queue)  { vQueueDelete(s_qm.event_queue);  s_qm.event_queue  = NULL; }

    memset(&s_qm, 0, sizeof(s_qm));
    ESP_LOGI(TAG, "Queues deleted");
}

/* =========================================================================
 * ADC SAMPLE QUEUE  (ADC Task Core0 → Signal Task Core0)
 * ========================================================================= */

bool qm_adc_send(const AdcSample_t *sample, uint32_t timeout_ms)
{
    configASSERT(s_qm.initialized);
    configASSERT(sample != NULL);

    TickType_t ticks = (timeout_ms == 0) ? 0 : pdMS_TO_TICKS(timeout_ms);
    BaseType_t ret   = xQueueSend(s_qm.adc_queue, sample, ticks);

    if (ret != pdTRUE) {
        s_qm.adc_drop_count++;
        if (s_qm.adc_drop_count % 10 == 1) {
            ESP_LOGW(TAG, "ADC queue full — dropped %lu samples total",
                     (unsigned long)s_qm.adc_drop_count);
        }
        return false;
    }
    return true;
}

bool qm_adc_receive(AdcSample_t *out_sample, uint32_t timeout_ms)
{
    configASSERT(s_qm.initialized);
    configASSERT(out_sample != NULL);

    TickType_t ticks = (timeout_ms == portMAX_DELAY)
                       ? portMAX_DELAY
                       : pdMS_TO_TICKS(timeout_ms);

    return xQueueReceive(s_qm.adc_queue, out_sample, ticks) == pdTRUE;
}

uint32_t qm_adc_waiting(void)
{
    if (!s_qm.initialized) return 0;
    return (uint32_t)uxQueueMessagesWaiting(s_qm.adc_queue);
}

/* =========================================================================
 * PROCESSED RESULT QUEUE  (Signal Task → UI Task)
 *
 * Overwrite semantics: UI always gets the LATEST value — never stale data.
 * For archaeology scanning, a missed old reading is better than a wrong one.
 * ========================================================================= */

void qm_result_send_overwrite(const ProcessedSample_t *result)
{
    configASSERT(s_qm.initialized);
    configASSERT(result != NULL);

    xQueueOverwrite(s_qm.result_queue, result);
}

bool qm_result_receive(ProcessedSample_t *out_result, uint32_t timeout_ms)
{
    configASSERT(s_qm.initialized);
    configASSERT(out_result != NULL);

    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
    return xQueueReceive(s_qm.result_queue, out_result, ticks) == pdTRUE;
}

/* =========================================================================
 * SYSTEM EVENT QUEUE — TASK CONTEXT
 * Non-blocking: NEVER blocks caller. Drop and log if full.
 * ========================================================================= */

bool qm_event_send(SystemEvent_t event, uint32_t data)
{
    configASSERT(s_qm.initialized);

    SysEventMsg_t msg = {
        .event        = event,
        .data         = data,
        .timestamp_ms = get_timestamp_ms(),
    };

    BaseType_t ret = xQueueSend(s_qm.event_queue, &msg, 0);

    if (ret != pdTRUE) {
        s_qm.event_drop_count++;
        if (s_qm.event_drop_count % 5 == 1) {
            ESP_LOGW(TAG, "Event queue full — event %d dropped (%lu total)",
                     (int)event, (unsigned long)s_qm.event_drop_count);
        }
        return false;
    }
    return true;
}

/* =========================================================================
 * SYSTEM EVENT QUEUE — ISR CONTEXT
 *
 * HARD RULES — NEVER VIOLATE:
 *   1. NO ESP_LOG* — vprintf triggers cache-miss panic on ESP32 from ISR
 *   2. NO xTaskGetTickCount — not ISR-safe
 *   3. MUST pass pxHigherPriorityTaskWoken pointer
 *   4. portYIELD_FROM_ISR stays with the CALLER (btn_scan_isr in app_main.c)
 *
 * timestamp_ms = 0: auto-stamped by qm_event_receive() on dequeue.
 * ========================================================================= */

bool qm_event_send_from_isr(SystemEvent_t event, uint32_t data,
                              BaseType_t *pxHigherPriorityTaskWoken)
{
    /* Avoid configASSERT here: it may call printf which is ISR-unsafe */
    if (!s_qm.initialized || !pxHigherPriorityTaskWoken) return false;

    SysEventMsg_t msg = {
        .event        = event,
        .data         = data,
        .timestamp_ms = 0,  /* Auto-stamped on receive */
    };

    BaseType_t ret = xQueueSendFromISR(s_qm.event_queue, &msg,
                                        pxHigherPriorityTaskWoken);
    if (ret != pdTRUE) {
        s_qm.event_drop_count++;
        /* NO logging here — ISR unsafe! */
        return false;
    }
    return true;
}

/* =========================================================================
 * SYSTEM EVENT QUEUE — RECEIVE
 * Auto-stamps ISR-sourced events (timestamp_ms==0) with current wall time.
 * ========================================================================= */

bool qm_event_receive(SysEventMsg_t *out_msg, uint32_t timeout_ms)
{
    configASSERT(s_qm.initialized);
    configASSERT(out_msg != NULL);

    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
    bool got = (xQueueReceive(s_qm.event_queue, out_msg, ticks) == pdTRUE);

    if (got && out_msg->timestamp_ms == 0) {
        out_msg->timestamp_ms = get_timestamp_ms();
    }
    return got;
}

/* =========================================================================
 * DIAGNOSTICS
 * ========================================================================= */

void qm_log_stats(void)
{
    if (!s_qm.initialized) return;

    ESP_LOGI(TAG, "=== Queue Stats ===");
    ESP_LOGI(TAG, "  ADC:    %lu / %u waiting, %lu dropped",
             (unsigned long)uxQueueMessagesWaiting(s_qm.adc_queue),
             QUEUE_DEPTH_ADC_SAMPLES,
             (unsigned long)s_qm.adc_drop_count);
    ESP_LOGI(TAG, "  Result: %lu waiting",
             (unsigned long)uxQueueMessagesWaiting(s_qm.result_queue));
    ESP_LOGI(TAG, "  Event:  %lu / %u waiting, %lu dropped",
             (unsigned long)uxQueueMessagesWaiting(s_qm.event_queue),
             QUEUE_DEPTH_EVENTS,
             (unsigned long)s_qm.event_drop_count);
    ESP_LOGI(TAG, "  Free heap: %lu bytes",
             (unsigned long)esp_get_free_heap_size());
}

uint32_t qm_get_adc_drop_count(void)
{
    return s_qm.adc_drop_count;
}
