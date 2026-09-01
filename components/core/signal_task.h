/**
 * @file signal_task.h
 * @brief Signal processing orchestrator — Core 0, Priority 5.
 *
 * RESPONSIBILITY:
 *  - Reads AdcSample_t from adc_queue
 *  - Runs 5-stage DSP pipeline via signal_processor
 *  - During calibration: feeds calib_engine, sends progress to result_queue
 *  - During active: sends ProcessedSample_t to result_queue + bt_sender (live)
 *  - Handles system events: SYS_EVT_TOUCH_CALIB, SYS_EVT_SENS_CHANGE
 *
 * STATE MACHINE:
 *
 *   BOOT ──▶ IDLE ──▶ CALIBRATING ──▶ ACTIVE ◀──▶ SLEEPING
 *                          │
 *                     (10s window)
 *                          │
 *                    calib_engine
 *                          │
 *                    result → UI
 *
 * INTER-TASK COMMUNICATION:
 *  Reads:  adc_queue     (from adc_task)
 *  Reads:  event_queue   (from ui_event_task / ISR)
 *  Writes: result_queue  (to ui_event_task)
 *  Writes: bt_sender     (via bt_enqueue_live — non-blocking)
 *
 * TASK CREATION (from app_main):
 *  xTaskCreatePinnedToCore(signal_task, "signal_task", 4096, NULL, 5, &h, 0);
 *
 * NOTE:
 *  signal_task receives a BTSender_t* via task arg (passed from app_main).
 *  This allows it to call bt_enqueue_live() directly without a global.
 */

#pragma once

#include "bluetooth_sender.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief FreeRTOS task entry point — signal processing.
 *
 * @param arg  Pointer to BTSender_t (cast from void*).
 *             Pass &g_bt_sender from app_main.
 *             If NULL, live BT streaming is disabled (non-fatal).
 */
void signal_task(void *arg);

/**
 * @brief Enable or disable Kalman filter at runtime.
 * Thread-safe: uses atomic flag read by signal_task.
 */
void signal_task_set_kalman(bool enable);

/**
 * @brief Enable or disable Kalman+Spatial filter (Experimental).
 * Enabling spatial auto-enables Kalman.
 * Disabling Kalman auto-disables spatial.
 */
void signal_task_set_spatial(bool enable);

/**
 * @brief Notify signal_task that scan mode changed.
 * Resets spatial filter history.
 * @param scanning true=scan active, false=live/idle
 */
void signal_task_set_scan_mode(bool scanning);

/**
 * @brief Get current Kalman enabled state.
 */
bool signal_task_kalman_enabled(void);

/**
 * @brief Get current Spatial enabled state.
 */
bool signal_task_spatial_enabled(void);

/**
 * @brief Set BT output mode for Visualizer.
 * @param mode  0=RAW  1=NORMALIZED  2=ENHANCED
 */
void signal_task_set_bt_mode(uint8_t mode);
uint8_t signal_task_get_bt_mode(void);

/**
 * @brief Set scan heading for Earth field compensation.
 * @param heading 0=N 1=E 2=S 3=W 4=disable
 */
void signal_task_set_heading(uint8_t heading);

/**
 * @brief Enable/disable heading compensation.
 * Auto-loads corrections from device calibration profile.
 */
void signal_task_set_heading_comp(bool enable);

/**
 * @brief Load heading correction table from device calibration.
 * Call once from app_main after devcal_load().
 * @param corrections Array of 4 floats [N, E, S, W]
 */
void signal_task_load_heading_corrections(const float corrections[4]);

/**
 * @brief Enable/disable Boost Mode.
 *
 * BOOST OFF (normal):
 *   Live:        8 samples  (+9dB)
 *   Scan:       32 samples  (+15dB)
 *
 * BOOST ON (max depth):
 *   Live:       64 samples  (+18dB)
 *   Scan:      256 samples  (+24dB)
 *
 * Boost scan beep: 1000Hz/150ms (vs normal 800Hz/90ms)
 */
void signal_task_set_boost(bool enable);
bool signal_task_boost_enabled(void);
bool signal_task_is_calibrated(void);
uint8_t signal_task_get_warmup_pct(void);
#ifdef __cplusplus
}
#endif
/**
 * @file signal_task.h
 * @brief Signal processing orchestrator — Core 0, Priority 5.
 *
 * RESPONSIBILITY:
 *  - Reads AdcSample_t from adc_queue
 *  - Runs 5-stage DSP pipeline via signal_processor
 *  - During calibration: feeds calib_engine, sends progress to result_queue
 *  - During active: sends ProcessedSample_t to result_queue + bt_sender (live)
 *  - Handles system events: SYS_EVT_TOUCH_CALIB, SYS_EVT_SENS_CHANGE
 *
 * STATE MACHINE:
 *
 *   BOOT ──▶ IDLE ──▶ CALIBRATING ──▶ ACTIVE ◀──▶ SLEEPING
 *                          │
 *                     (10s window)
 *                          │
 *                    calib_engine
 *                          │
 *                    result → UI
 *
 * INTER-TASK COMMUNICATION:
 *  Reads:  adc_queue     (from adc_task)
 *  Reads:  event_queue   (from ui_event_task / ISR)
 *  Writes: result_queue  (to ui_event_task)
 *  Writes: bt_sender     (via bt_enqueue_live — non-blocking)
 *
 * TASK CREATION (from app_main):
 *  xTaskCreatePinnedToCore(signal_task, "signal_task", 4096, NULL, 5, &h, 0);
 *
 * NOTE:
 *  signal_task receives a BTSender_t* via task arg (passed from app_main).
 *  This allows it to call bt_enqueue_live() directly without a global.
 */

#pragma once

#include "bluetooth_sender.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief FreeRTOS task entry point — signal processing.
 *
 * @param arg  Pointer to BTSender_t (cast from void*).
 *             Pass &g_bt_sender from app_main.
 *             If NULL, live BT streaming is disabled (non-fatal).
 */
void signal_task(void *arg);

/**
 * @brief Enable or disable Kalman filter at runtime.
 * Thread-safe: uses atomic flag read by signal_task.
 */
void signal_task_set_kalman(bool enable);

/**
 * @brief Enable or disable Kalman+Spatial filter (Experimental).
 * Enabling spatial auto-enables Kalman.
 * Disabling Kalman auto-disables spatial.
 */
void signal_task_set_spatial(bool enable);

/**
 * @brief Notify signal_task that scan mode changed.
 * Resets spatial filter history.
 * @param scanning true=scan active, false=live/idle
 */
void signal_task_set_scan_mode(bool scanning);

/**
 * @brief Get current Kalman enabled state.
 */
bool signal_task_kalman_enabled(void);

/**
 * @brief Get current Spatial enabled state.
 */
bool signal_task_spatial_enabled(void);

/**
 * @brief Set BT output mode for Visualizer.
 * @param mode  0=RAW  1=NORMALIZED  2=ENHANCED
 */
void signal_task_set_bt_mode(uint8_t mode);
uint8_t signal_task_get_bt_mode(void);
/** Returns true if soil calibration has been completed in this session. */
bool signal_task_is_calibrated(void);

/**
 * @brief Returns uncalibrated warmup progress 0–100 (100 = warmup complete).
 * Returns 100 immediately if calibrated (warmup not applicable).
 */
uint8_t signal_task_get_warmup_pct(void);

/**
 * @brief Set scan heading for Earth field compensation.
 * @param heading 0=N 1=E 2=S 3=W 4=disable
 */
void signal_task_set_heading(uint8_t heading);

/**
 * @brief Enable/disable heading compensation.
 * Auto-loads corrections from device calibration profile.
 */
void signal_task_set_heading_comp(bool enable);

/**
 * @brief Load heading correction table from device calibration.
 * Call once from app_main after devcal_load().
 * @param corrections Array of 4 floats [N, E, S, W]
 */
void signal_task_load_heading_corrections(const float corrections[4]);

/**
 * @brief Enable/disable Boost Mode.
 *
 * BOOST OFF (normal):
 *   Live:        8 samples  (+9dB)
 *   Scan:       32 samples  (+15dB)
 *
 * BOOST ON (max depth):
 *   Live:       64 samples  (+18dB)
 *   Scan:      256 samples  (+24dB)
 *
 * Boost scan beep: 1000Hz/150ms (vs normal 800Hz/90ms)
 */
void signal_task_set_boost(bool enable);
bool signal_task_boost_enabled(void);

#ifdef __cplusplus
}
#endif
