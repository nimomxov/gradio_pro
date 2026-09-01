/**
 * @file ui_event_task.h
 * @brief الجسر الوحيد بين Backend (Core 0) و EEZ Studio Frontend (Core 1).
 *
 * ═══════════════════════════════════════════════════════════════════
 *  THREAD SAFETY RULE
 * ═══════════════════════════════════════════════════════════════════
 *  كل lv_* يُستدعى من ui_event_task فقط (Core 1).
 *  g_lvgl_mutex مطلوب قبل أي lv_* call خارج lvgl_task.
 *
 * ═══════════════════════════════════════════════════════════════════
 *  PUBLIC API — تُستدعى من screens.c / actions الخاصة بـ EEZ
 * ═══════════════════════════════════════════════════════════════════
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "bluetooth_sender.h"

#ifdef __cplusplus
extern "C" {
#endif

/** FreeRTOS task entry — pass &g_bt_sender من app_main */
void ui_event_task(void *arg);

/* ── Live Scan Tab ── */
void ui_request_calibration(void);          ///< زر CALIBRATE

/* ── 3D Scan Tab ── */
void ui_request_manual_scan(void);          ///< زر Manual Scan
void ui_request_auto_scan(uint8_t steps,    ///< زر Start في auto panel
                          uint8_t seconds);
void ui_cancel_scan(void);                  ///< Cancel (auto أو manual)

/* ── Settings Tab ── */
void ui_set_sensitivity(uint8_t dropdown_index);  ///< sensibility_settings
void ui_set_manual_sensitivity_mode(bool enabled);///< manual_sensibility_switch

/* ── Filter controls (Settings tab) ── */
void ui_set_kalman(bool enabled);          ///< kalman_filter checkbox
void ui_set_kalman_spatial(bool enabled);  ///< kalman_spatial checkbox

/* ── BT output mode (3D Scan tab) ── */
void ui_set_bt_mode(uint8_t idx);  ///< scan_mode dropdown: 0=RAW 1=NORM 2=ENH

/* ── Heading compensation (3D Scan only) ── */
/**
 * @brief Set scan heading for Earth field compensation.
 * Used in 3D scan (Manual/Auto) only — NOT in Live scan.
 * Removes Earth magnetic field component for the scan direction,
 * making the two sensors behave as paired (gradient = 0 at no target).
 * heading: 0=North  1=East  2=South  3=West  4=Disable
 */
void ui_set_scan_heading(uint8_t heading);

/**
 * @brief Enable/disable heading compensation for 3D scan.
 * Requires Phase 3A device calibration to be valid.
 */
void ui_set_heading_comp(bool enable);

/* ── Boost Mode ── */
/**
 * @brief Enable/disable Boost Mode (max depth + SNR).
 * Live: 8→64 samples. Scan: 32→256 samples.
 * Scan beep changes: 800Hz/90ms → 1000Hz/150ms.
 */
void ui_set_boost_mode(bool enable);

/**
 * @brief Show confirmation dialog then reset device calibration.
 *
 * Flow:
 *   1. Modal dialog: "Reset device calibration?"  [Confirm] [Cancel]
 *   2. On Confirm:
 *        - devcal_nvs_clear()   → erases profile_v2 + device_valid + cal_version
 *        - Touch calibration ("touch_cal") is NOT touched
 *        - esp_restart()        → boots into device calibration wizard
 *   3. On Cancel: dialog dismissed, no action.
 */
void ui_request_reset_device_calib(void);

/**
 * @brief Enable/disable Adaptive Sampling (Stability Gate) in Auto Scan.
 *
 * When ON (default):
 *   Auto Scan waits until signal stabilizes before taking each sample.
 *   threshold = 1.5 × noise_floor  (from device calibration)
 *   MIN wait: 1.5s  |  MAX wait: 5s (then forced)
 *
 * When OFF:
 *   Auto Scan takes sample immediately at countdown = 0.
 *
 * Manual Scan is NOT affected — always immediate.
 */
void ui_set_stability_gate(bool enabled);

#ifdef __cplusplus
}
#endif
