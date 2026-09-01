/**
 * @file sensitivity_manager.h
 * @brief Sensitivity mode management — software + hardware (PGA) coordination.
 *
 * SENSITIVITY IS TWO-LAYERED:
 *
 *  Layer 1 — Hardware (ADS1115 PGA):
 *    Controls the physical amplifier gain.
 *    More gain → more voltage range used → better LSB resolution.
 *    But too much gain → clipping on mineralized soil.
 *
 *    HIGH sensitivity: PGA ±1.024V (31.25µV/LSB) — best for deep targets
 *    MEDIUM:           PGA ±2.048V (62.5µV/LSB)  — default, safe
 *    LOW:              PGA ±4.096V (125µV/LSB)    — mineralized soil
 *
 *  Layer 2 — Software (DSP filter params):
 *    Controls moving average window, EMA alpha, outlier threshold.
 *    Managed by signal_processor.c.
 *
 * WHY COORDINATE THEM HERE:
 *    Changing PGA without changing filter params → incoherent system.
 *    This manager ensures both layers change atomically when user
 *    or the calibration engine requests a sensitivity change.
 *
 * FLOW:
 *  1. Calibration engine recommends a mode → sensitivity_manager_apply()
 *  2. User overrides → sensitivity_manager_set_user()
 *  3. Manager sends SYS_EVT_SENS_CHANGE to signal_task (software)
 *  4. Manager calls ads1115_set_pga() directly (hardware)
 *  5. UI shows current mode + recommendation
 */

#pragma once

#include "core/gradiometer_types.h"
#include "drivers/ads1115_driver.h"
#include "freertos/semphr.h"           // <-- for SemaphoreHandle_t
#include "esp_err.h"


#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * MODE DESCRIPTION TABLE (for UI display)
 * ========================================================================= */

typedef struct {
    SensitivityMode_t mode;
    const char       *name;         ///< Short name: "HIGH", "MEDIUM", "LOW"
    const char       *description;  ///< One-line description for UI
    ADS1115_PGA_t     pga;          ///< Hardware PGA setting
    float             lsb_uv;       ///< Resulting LSB in microvolts
} SensModeInfo_t;

/* Table is accessible for UI display */
extern const SensModeInfo_t SENS_MODE_INFO[6];

/* =========================================================================
 * MANAGER STATE
 * ========================================================================= */

typedef struct {
    SensitivityMode_t  current_mode;
    SensitivityMode_t  recommended_mode;   ///< From last calibration
    SensitivityMode_t  effective_mode;      //// Added manualy by me
	SemaphoreHandle_t  lock;                // <-- ADD THIS
	bool               user_override;      ///< true = user chose, false = auto
    ADS1115Driver_t   *adc_driver;         ///< Pointer to ADS1115 instance
    bool               initialized;
} SensitivityManager_t;

/* =========================================================================
 * LIFECYCLE
 * ========================================================================= */

/**
 * @brief Initialize sensitivity manager.
 * @param mgr        Manager instance.
 * @param adc_driver Pointer to initialized ADS1115 driver.
 *                   Used for hardware PGA changes.
 */
void sens_manager_init(SensitivityManager_t *mgr, ADS1115Driver_t *adc_driver);

/* =========================================================================
 * MODE CONTROL
 * ========================================================================= */

/**
 * @brief Apply calibration-recommended sensitivity mode.
 * Sets both hardware PGA and sends software event.
 * Clears user override flag — system is in AUTO mode.
 *
 * @param mgr   Manager instance.
 * @param mode  Recommended mode from CalibResult_t.recommended_mode.
 */
esp_err_t sens_manager_apply_recommendation(SensitivityManager_t *mgr,
                                            SensitivityMode_t mode);

/**
 * @brief Apply user-selected sensitivity mode.
 * Sets user_override = true. Will not be overwritten by next calibration.
 * User must explicitly select AUTO to return to calibration recommendation.
 *
 * @param mgr   Manager instance.
 * @param mode  User-selected mode.
 */
esp_err_t sens_manager_set_user(SensitivityManager_t *mgr, SensitivityMode_t mode);

/**
 * @brief Return to calibration-recommended mode (clears user override).
 */
esp_err_t sens_manager_set_auto(SensitivityManager_t *mgr);

/* =========================================================================
 * QUERIES
 * ========================================================================= */

SensitivityMode_t   sens_manager_get_current(const SensitivityManager_t *mgr);
SensitivityMode_t   sens_manager_get_recommended(const SensitivityManager_t *mgr);
bool                sens_manager_is_user_override(const SensitivityManager_t *mgr);
/**
 * @brief Auto engine — call after every sp_process() with current metrics.
 * Only acts when mode == AUTO and no user override.
 * Includes: noise EMA, stability gate, time lock, hysteresis FSM.
 */
void sens_manager_auto_update(SensitivityManager_t *mgr,
                              float noise_floor,
                              float snr,
                              float stability,
                              bool  stable,
                              uint32_t now_ms);

SensitivityMode_t sens_manager_get_effective(const SensitivityManager_t *mgr);
const SensModeInfo_t *sens_manager_get_info(SensitivityMode_t mode);

/**
 * @brief Get human-readable status string for UI display.
 * Example: "HIGH (user)" or "MEDIUM (auto)"
 * @param buf     Output buffer.
 * @param buf_len Buffer length (recommend >= 32).
 */
void sens_manager_get_status_str(const SensitivityManager_t *mgr,
                                 char *buf, size_t buf_len);

#ifdef __cplusplus
}
#endif
