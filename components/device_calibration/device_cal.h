#pragma once
/**
 * @file device_cal.h
 * @brief 5-Phase Professional Device Calibration — Public API
 *
 * Architecture:
 *   device_cal.c       — orchestrator (calls all phases)
 *   devcal_common.c    — UI, button (GPIO16), sampling, NVS
 *   devcal_phase1.c    — Sensor Offset + Noise Floor
 *   devcal_phase2.c    — Sensor Matching (magnet N+S poles)
 *   devcal_phase3.c    — Directional (N / E / S / W)
 *   devcal_phase4.c    — Tilt (N↔S axis + E↔W axis)
 *   devcal_phase5.c    — Final Noise Floor + Kalman Q/R
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════
 * DEVICE PROFILE — saved to NVS
 * ═══════════════════════════════════════════════════════════════════ */
typedef struct {
    /* Phase 1 — Sensor Offset + Noise Floor */
    int32_t  nominal_offset;      ///< Raw gradient at zero field
    float    noise_floor;         ///< Std dev at rest (LSB)
    float    kalman_R;            ///< R = noise_floor²

    /* Phase 2 — Sensor Matching */
    float    gradient_N;          ///< Response to North pole magnet
    float    gradient_S;          ///< Response to South pole magnet
    float    dynamic_range;       ///< |grad_N| + |grad_S|
    float    asymmetry;           ///< (grad_N + grad_S)/2 — sensor mismatch
    float    sensitivity;         ///< Relative sensitivity (LSB/unit)
    float    kalman_Q;            ///< Q = (noise/range)² × 0.1

    /* Phase 3A — Horizontal Compass (Earth field compensation) */
    float    horiz_grad[4];       ///< Gradient when horizontal [N,E,S,W]
    float    sensor_mismatch;     ///< (grad_N+grad_S)/2 — sensor offset diff
    float    heading_coeff_ns;    ///< (grad_N-grad_S)/2 — N-S Earth sensitivity
    float    heading_coeff_ew;    ///< (grad_E-grad_W)/2 — E-W Earth sensitivity
    float    heading_correction[4]; ///< Per-direction correction [N,E,S,W]

    /* Phase 3B — Vertical Rotation (device body iron) */
    float    hard_iron[4];        ///< Baseline per direction [N,E,S,W]
    float    dir_variation;       ///< Max - Min across directions
    float    dir_correction;      ///< Soft iron correction coefficient

    /* Phase 3C — Static Sensor Matching (4000 samples per channel) */
    float    sensor_offset2;      ///< AIN0_mean - AIN1_mean in Earth field (LSB)
    float    sensor_gain2;        ///< std_AIN0 / std_AIN1 — sensitivity ratio
    float    ain1_dc_level;       ///< AIN1 absolute DC level in Earth field (LSB)
    float    sensor_match_addend; ///< Precomputed: ain1_dc*(1-gain2)+offset2*gain2
    float    p3c_std_ain0;        ///< AIN0 std dev during Phase 3C (diagnostic)
    float    p3c_std_ain1;        ///< AIN1 std dev during Phase 3C (diagnostic)
    uint32_t p3c_samples;         ///< Actual samples collected

    /* Phase 4 — Tilt */
    float    tilt_ns;             ///< LSB/degree — North-South axis tilt
    float    tilt_ew;             ///< LSB/degree — East-West axis tilt
    float    tilt_threshold_deg;  ///< Acceptable tilt before error

    /* Phase 5 — Final */
    float    detection_limit;     ///< Min detectable signal (LSB) = 3×std
    float    kalman_Q_final;      ///< Optimised final Q
    float    kalman_R_final;      ///< Optimised final R

    /* Metadata */
    uint32_t calibration_epoch;   ///< xTaskGetTickCount() at save time
    uint8_t  phases_completed;    ///< Bitmask bit0=P1 … bit4=P5
    bool     valid;
} DeviceProfile_t;

/* ═══════════════════════════════════════════════════════════════════
 * CALIBRATION RESULT — passed to signal_processor
 * ═══════════════════════════════════════════════════════════════════ */
typedef struct {
    float    kalman_Q;
    float    kalman_R;
    int32_t  nominal_offset;
    float    asymmetry;
    float    detection_limit;
    bool     valid;
} DevCalResult_t;

/* ═══════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ═══════════════════════════════════════════════════════════════════ */

/** Load profile from NVS. Returns true if valid. */
bool devcal_load(void);

/** Run 5-phase calibration wizard (blocking). */
void devcal_run(void);

/** True if a valid device profile is loaded. */
bool devcal_is_valid(void);

/** Get Kalman Q/R and corrections for signal_processor. */
DevCalResult_t devcal_get_result(void);

/** Get full profile (read-only). */
const DeviceProfile_t *devcal_get_profile(void);

/** Erase profile from NVS. */
void devcal_clear(void);

/** Apply Phase 3C sensor matching correction to a raw differential reading.
 *  corrected = (raw - sensor_offset2) * sensor_gain2
 *  Returns raw unchanged if Phase 3C not yet completed. */
int32_t devcal_apply_sensor_match(int32_t raw);

/** Apply sensor asymmetry correction to a raw reading. */
int32_t devcal_apply_asymmetry(int32_t raw);

/** Apply directional correction. heading: 0=N 1=E 2=S 3=W */
int32_t devcal_apply_direction(int32_t raw, uint8_t heading);

/** Apply full heading compensation (Phase 3A).
 *  heading: 0=N 1=E 2=S 3=W
 *  Returns corrected gradient with Earth field removed. */
int32_t devcal_apply_heading_compensation(int32_t raw, uint8_t heading);

/** Get heading correction table (for signal_processor use). */
const float *devcal_get_heading_corrections(void);

#ifdef __cplusplus
}
#endif
