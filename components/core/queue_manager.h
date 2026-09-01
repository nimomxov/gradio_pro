/**
 * @file queue_manager.h
 * @brief Centralized FreeRTOS queue management.
 *
 * DESIGN PHILOSOPHY:
 *  - One place owns all queue handles. No extern handles scattered across files.
 *  - All queue operations go through typed wrapper functions.
 *  - Wrappers enforce timeouts — no task blocks forever.
 *  - Non-blocking "try" variants available for ISR/time-critical contexts.
 */

#pragma once

#include "gradiometer_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * INITIALIZATION
 * ========================================================================= */

/**
 * @brief Create all system queues. Call ONCE from app_main before tasks start.
 * @return ESP_OK on success, ESP_FAIL if any queue creation fails (fatal).
 */
esp_err_t queue_manager_init(void);

/**
 * @brief Delete all queues (used in unit tests / reset scenarios).
 */
void queue_manager_deinit(void);

/* =========================================================================
 * ADC SAMPLE QUEUE  (ADC Task → Signal Task)
 * ========================================================================= */

/**
 * @brief Send an ADC sample to the signal processing pipeline.
 *
 * @param sample  Pointer to sample (copied into queue, safe to reuse).
 * @param timeout_ms  Max wait time. Use 0 for non-blocking.
 * @return true if enqueued, false if queue full (sample dropped).
 */
bool qm_adc_send(const AdcSample_t *sample, uint32_t timeout_ms);

/**
 * @brief Receive an ADC sample in the signal processing task.
 *
 * @param out_sample  Output buffer.
 * @param timeout_ms  Max wait time. Use portMAX_DELAY to block until available.
 * @return true if sample received, false if timeout.
 */
bool qm_adc_receive(AdcSample_t *out_sample, uint32_t timeout_ms);

/**
 * @brief Get number of samples currently waiting in ADC queue.
 */
uint32_t qm_adc_waiting(void);

/* =========================================================================
 * PROCESSED RESULT QUEUE  (Signal Task → UI Task)
 * ========================================================================= */

/**
 * @brief Send processed result to UI.
 *
 * Uses "overwrite" semantics: if queue is full, the OLD item is discarded
 * and replaced. UI always gets the LATEST value, never stale data.
 *
 * @param result  Pointer to result (copied).
 */
void qm_result_send_overwrite(const ProcessedSample_t *result);

/**
 * @brief Receive processed result in UI task.
 *
 * @param out_result  Output buffer.
 * @param timeout_ms  Max wait time.
 * @return true if result received.
 */
bool qm_result_receive(ProcessedSample_t *out_result, uint32_t timeout_ms);

/* =========================================================================
 * SYSTEM EVENT QUEUE  (Any Task → Any Task)
 * ========================================================================= */

/**
 * @brief Send a system event (e.g., button press, calibration done).
 *
 * @param event     Event type.
 * @param data      Optional payload.
 * @param from_isr  Set true if calling from an ISR context.
 * @return true if sent successfully.
 */
//bool qm_event_send(SystemEvent_t event, uint32_t data, bool from_isr);
bool qm_event_send(SystemEvent_t event, uint32_t data);
bool qm_event_send_from_isr(SystemEvent_t event, uint32_t data, BaseType_t *pxHigherPriorityTaskWoken);
/**
 * @brief Receive a system event.
 *
 * @param out_msg   Output message buffer.
 * @param timeout_ms  Max wait. Use 0 to poll without blocking.
 * @return true if event received.
 */
bool qm_event_receive(SysEventMsg_t *out_msg, uint32_t timeout_ms);

/* =========================================================================
 * DIAGNOSTICS
 * ========================================================================= */

/**
 * @brief Log queue depths and overflow counters via ESP_LOGI.
 * Call periodically from monitor task.
 */
void qm_log_stats(void);

/**
 * @brief Return total number of ADC samples dropped due to queue overflow.
 */
uint32_t qm_get_adc_drop_count(void);

#ifdef __cplusplus
}
#endif
