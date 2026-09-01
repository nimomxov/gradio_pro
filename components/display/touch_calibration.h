#pragma once
/**
 * @file touch_calibration.h
 * @brief Professional 9-point touch calibration system
 *
 * Algorithm: Affine Transform with Least Squares Fitting
 *   raw (Xr,Yr) → screen (Xs,Ys) via 6-coefficient matrix:
 *   Xs = A*Xr + B*Yr + C
 *   Ys = D*Xr + E*Yr + F
 *
 * Storage: ESP32 NVS ("touch_cal" namespace)
 * Accuracy: Sub-pixel on 320×240 ILI9341 + XPT2046
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Calibration coefficients (Affine Transform) ── */
typedef struct {
    float A, B, C;   /* X = A*raw_x + B*raw_y + C */
    float D, E, F;   /* Y = D*raw_x + E*raw_y + F */
    bool  valid;
} TouchCalCoeffs_t;

/**
 * @brief Load calibration from NVS.
 * @return true if valid calibration found, false if first run.
 */
bool touch_cal_load(void);

/**
 * @brief Run the 9-point calibration UI (blocking).
 *
 * Draws crosshair targets on screen, collects raw touch samples,
 * computes Affine Transform via Least Squares, saves to NVS.
 *
 * Must be called after lv_init() and display init.
 * Blocks until user completes all 9 points.
 */
void touch_cal_run(void);

/**
 * @brief Apply calibration to raw XPT2046 reading.
 * @param raw_x  Raw ADC X from XPT2046
 * @param raw_y  Raw ADC Y from XPT2046
 * @param out_x  Calibrated screen X [0..319]
 * @param out_y  Calibrated screen Y [0..239]
 */
void touch_cal_apply(int32_t raw_x, int32_t raw_y,
                     int32_t *out_x, int32_t *out_y);

/**
 * @brief Check if calibration data is loaded and valid.
 */
bool touch_cal_is_valid(void);

/**
 * @brief Clear calibration from NVS (force recalibration on next boot).
 */
void touch_cal_clear(void);

/**
 * @brief Get current coefficients (for diagnostics).
 */
const TouchCalCoeffs_t *touch_cal_get_coeffs(void);

#ifdef __cplusplus
}
#endif
