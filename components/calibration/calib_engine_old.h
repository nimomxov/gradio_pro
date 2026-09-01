/**
 * @file calib_engine.h
 * @brief 10-second calibration engine with soil analysis and sensitivity recommendation.
 *
 * CALIBRATION PHILOSOPHY:
 *  A gradiometer is only as good as its baseline. Rushing calibration
 *  produces a bad baseline → false positives everywhere → useless device.
 *
 *  This engine enforces a MINIMUM 10-second collection window because:
 *   1. FLC100 coils have a thermal settling time (~5s after power-on)
 *   2. 10s at 40Hz effective rate = 400 samples → statistically robust
 *   3. Allows the operator to sweep a known-clean area (no targets)
 *   4. Captures environmental drift trend for drift correction
 *
 * WHAT CALIBRATION COMPUTES:
 *
 *  Phase 1 — Collection (0-10s):
 *    Gathers raw ADC samples into a ring buffer.
 *    Rejects obvious hardware glitches (quality < threshold).
 *    Tracks running mean and variance using Welford's algorithm.
 *    (Welford = numerically stable, no catastrophic cancellation, O(1) memory)
 *
 *  Phase 2 — Analysis (instant, after collection):
 *    a) Outlier removal: removes samples > 3σ from mean (Chauvenet criterion)
 *    b) Drift estimation: linear regression on sample sequence
 *       → if drift > threshold: warn operator, calib may be unreliable
 *    c) Soil classification: based on std_dev magnitude
 *       PRISTINE  std < 1.0   → noise floor ممتاز جداً → VERY_HIGH
 *       CLEAN     std 1-4     → noise floor منخفض       → HIGH
 *       MINERAL   std 4-12    → تمعدن معتدل             → MEDIUM
 *       NOISY     std 12-30   → ضوضاء عالية             → LOW
 *       EXTREME   std > 30    → تداخل صناعي/بركاني      → VERY_LOW
 *    d) Sensitivity recommendation: maps soil type → SensitivityMode_t
 *    e) Filter parameter computation: window_size, alpha, outlier_sigma
 *
 *  Phase 3 — Validation:
 *    Checks that enough valid samples were collected (>= 90%).
 *    Checks that drift is within acceptable bounds.
 *    Sets CalibResult_t.is_valid accordingly.
 *
 * USAGE:
 *  @code
 *    calib_engine_init(&engine);
 *    calib_engine_start(&engine);
 *    // In sample loop:
 *    CalibEngineStatus_t s = calib_engine_feed(&engine, &sample);
 *    if (s == CALIB_STATUS_DONE) {
 *        calib_engine_get_result(&engine, &result);
 *    }
 *  @endcode
 */

#pragma once

#include "core/gradiometer_types.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * CONSTANTS
 * ========================================================================= */

/** Duration of calibration collection window */
#define CALIB_DURATION_MS           10000u

/** Ring buffer size — must hold all samples from 10s window + margin */
/** At 50Hz with some overhead: 50 × 10 = 500. Use 512 (power of 2). */
#define CALIB_RING_SIZE             512u

/** Minimum fraction of expected samples that must be valid */
#define CALIB_MIN_VALID_RATIO       0.90f

/** Outlier rejection: remove samples > this many σ from mean */
#define CALIB_OUTLIER_SIGMA         3.0f

/** Maximum acceptable drift (ADC counts per second) before warning */
#define CALIB_MAX_DRIFT_RATE        2.0f

/** Minimum quality score for a sample to be accepted during calibration */
#define CALIB_MIN_SAMPLE_QUALITY    30u

/* Soil type noise thresholds (in ADC counts std dev)
 *
 * تقسيم خماسي يتوافق مع خمسة مستويات حساسية:
 *
 *  std < 1.0   → SOIL_TYPE_PRISTINE  → VERY_HIGH
 *  std 1-4     → SOIL_TYPE_CLEAN     → HIGH
 *  std 4-12    → SOIL_TYPE_MINERAL   → MEDIUM
 *  std 12-30   → SOIL_TYPE_NOISY     → LOW
 *  std > 30    → SOIL_TYPE_EXTREME   → VERY_LOW
 */
#define SOIL_THRESHOLD_PRISTINE     1.0f
#define SOIL_THRESHOLD_CLEAN        4.0f
#define SOIL_THRESHOLD_MINERAL      12.0f
#define SOIL_THRESHOLD_NOISY        30.0f
/* Above NOISY threshold → SOIL_TYPE_EXTREME */

/* =========================================================================
 * STATUS
 * ========================================================================= */

typedef enum {
    CALIB_STATUS_IDLE        = 0,  ///< Not started
    CALIB_STATUS_IN_PROGRESS = 1,  ///< Collecting samples
    CALIB_STATUS_COMPUTING   = 2,  ///< Running analysis (brief)
    CALIB_STATUS_DONE        = 3,  ///< Complete — call get_result()
    CALIB_STATUS_ERROR       = 4,  ///< Failed — not enough valid samples
} CalibEngineStatus_t;

/* =========================================================================
 * ENGINE STATE
 * ========================================================================= */

typedef struct {
    /* Sample collection */
    int16_t  ring[CALIB_RING_SIZE];   ///< Raw differential samples
    uint16_t ring_head;               ///< Write index
    uint16_t ring_count;              ///< Total samples stored

    /* Welford's online algorithm state */
    double   welford_mean;            ///< Running mean (double for precision)
    double   welford_M2;              ///< Running sum of squared deviations
    uint32_t welford_n;               ///< Welford sample count

    /* Collection tracking */
    uint32_t start_tick;              ///< xTaskGetTickCount() at start
    uint32_t total_fed;               ///< All samples fed (including rejected)
    uint32_t rejected_count;          ///< Samples rejected for low quality

    /* State */
    CalibEngineStatus_t status;
    bool                started;

    /* Result (filled after DONE) */
    CalibResult_t result;

} CalibEngine_t;

/* =========================================================================
 * API
 * ========================================================================= */

/**
 * @brief Initialize calibration engine. Call once at startup.
 */
void calib_engine_init(CalibEngine_t *eng);

/**
 * @brief Start a new calibration session.
 * Resets all state and begins the 10-second collection window.
 * Any previous result is invalidated.
 */
void calib_engine_start(CalibEngine_t *eng);

/**
 * @brief Feed one ADC sample into the engine.
 *
 * Call from signal_task every time a sample arrives while in CALIBRATING state.
 * The engine manages the 10-second window internally.
 *
 * @return CALIB_STATUS_IN_PROGRESS while collecting.
 *         CALIB_STATUS_DONE when 10 seconds elapsed and analysis is complete.
 *         CALIB_STATUS_ERROR if analysis failed (not enough valid samples).
 */
CalibEngineStatus_t calib_engine_feed(CalibEngine_t *eng, const AdcSample_t *sample);

/**
 * @brief Get calibration progress as percentage [0-100].
 * Based on elapsed time vs CALIB_DURATION_MS.
 */
uint8_t calib_engine_get_progress(const CalibEngine_t *eng);

/**
 * @brief Retrieve calibration result after CALIB_STATUS_DONE.
 * @param out_result  Output — filled with calibration data.
 * @return ESP_OK if result is valid, ESP_ERR_INVALID_STATE if not done yet.
 */
esp_err_t calib_engine_get_result(const CalibEngine_t *eng, CalibResult_t *out_result);

/**
 * @brief Get current engine status.
 */
CalibEngineStatus_t calib_engine_get_status(const CalibEngine_t *eng);

/**
 * @brief Classify soil type from standard deviation.
 * Exposed for testing / UI display.
 */
SoilType_t calib_classify_soil(float std_dev);

/**
 * @brief Map soil type to recommended sensitivity mode.
 * Exposed for UI display of recommendation rationale.
 */
SensitivityMode_t calib_recommend_sensitivity(SoilType_t soil);

/**
 * @brief Compute filter parameters for a given soil type and sensitivity.
 * Fills window_size, alpha_smooth, outlier_sigma in the result struct.
 */
void calib_compute_filter_params(SoilType_t soil, SensitivityMode_t mode,
                                 CalibResult_t *out_result);

#ifdef __cplusplus
}
#endif
