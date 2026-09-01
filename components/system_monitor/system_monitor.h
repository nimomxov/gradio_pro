/**
 * @file system_monitor.h
 * @brief System health monitor — heap, stack, queue watchdog.
 *
 * Runs on Core 0 at LOW priority. Catches problems BEFORE they crash the system.
 * Triggers controlled fault handling instead of silent watchdog resets.
 */

#pragma once

#include "gradiometer_types.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * TASK REGISTRY
 * Monitors must know about all tasks to check their stack usage.
 * ========================================================================= */

/**
 * @brief Register a task handle for stack monitoring.
 * Call after each xTaskCreate, before starting the scheduler.
 *
 * @param handle  Task handle from xTaskCreate.
 * @param name    Human-readable name (for logging).
 * @param stack_warning_bytes  Log warning if remaining stack < this value.
 */
void sysmon_register_task(TaskHandle_t handle, const char *name,
                          uint32_t stack_warning_bytes);

/* =========================================================================
 * LIFECYCLE
 * ========================================================================= */

/**
 * @brief Initialize the system monitor. Call before starting the task.
 */
esp_err_t sysmon_init(void);

/**
 * @brief FreeRTOS task function. Pin to Core 0, priority 2 (low).
 *
 * @code
 *   xTaskCreatePinnedToCore(sysmon_task, "sysmon", 2048, NULL, 2, &handle, 0);
 * @endcode
 */
void sysmon_task(void *arg);

/* =========================================================================
 * FAULT MANAGEMENT
 * ========================================================================= */

/**
 * @brief Report a hardware or software fault.
 * Sends SYS_EVT_FAULT to the event queue and logs the error.
 * Does NOT assert/abort — lets the system handle it gracefully.
 *
 * @param code  Fault code (can OR multiple faults: FAULT_ADC_NOT_FOUND | FAULT_LOW_MEMORY)
 * @param msg   Optional human-readable message (can be NULL).
 */
void sysmon_report_fault(FaultCode_t code, const char *msg);

/**
 * @brief Get the current accumulated fault flags.
 */
FaultCode_t sysmon_get_faults(void);

/**
 * @brief Clear a specific fault flag (call after recovery).
 */
void sysmon_clear_fault(FaultCode_t code);

/**
 * @brief Returns true if any fault is currently active.
 */
bool sysmon_has_fault(void);

#ifdef __cplusplus
}
#endif
