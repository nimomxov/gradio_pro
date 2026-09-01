#pragma once
/**
 * @file calib_engine.h
 * @brief Two-phase soil calibration: Static (10s) + Dynamic (20s walking).
 *
 * PHASE 1 — STATIC (10s, device still):
 *   baseline_fixed + noise_floor_static + kalman_R initial
 *   Buzzer: single beep at start
 *
 * PHASE 2 — DYNAMIC (20s, slow walk ~0.5 m/s):
 *   noise_floor_dynamic = std(signal - local_moving_average)
 *   Local MA removes spatial drift → only true soil noise remains
 *   Outlier rejection excludes real targets from noise estimate
 *   Buzzer: double beep at phase transition
 *
 * COMBINATION:
 *   baseline_final    = baseline_fixed  (Phase 1 only — motion-independent)
 *   noise_floor_final = max(noise_static, noise_dynamic)
 *   kalman_R_final    = noise_floor_final²
 *
 * PROGRESS: 0-50% = Phase 1, 50-100% = Phase 2
 */

#include "gradiometer_types.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CALIB_STATIC_MS          10000u
#define CALIB_DYNAMIC_MS         20000u
#define CALIB_DURATION_MS        (CALIB_STATIC_MS + CALIB_DYNAMIC_MS)
#define CALIB_RING_SIZE          512u
#define CALIB_MIN_VALID_RATIO    0.85f
#define CALIB_OUTLIER_SIGMA      3.0f
#define CALIB_MAX_DRIFT_RATE     2.0f
#define CALIB_MIN_SAMPLE_QUALITY 30u
#define CALIB_DYN_LOCAL_WIN      16u

#define SOIL_THRESHOLD_PRISTINE  1.0f
#define SOIL_THRESHOLD_CLEAN     4.0f
#define SOIL_THRESHOLD_MINERAL   12.0f
#define SOIL_THRESHOLD_NOISY     30.0f

typedef enum {
    CALIB_PHASE_STATIC  = 0,
    CALIB_PHASE_DYNAMIC = 1,
} CalibPhaseInternal_t;

typedef enum {
    CALIB_STATUS_IDLE        = 0,
    CALIB_STATUS_IN_PROGRESS = 1,
    CALIB_STATUS_COMPUTING   = 2,
    CALIB_STATUS_DONE        = 3,
    CALIB_STATUS_ERROR       = 4,
} CalibEngineStatus_t;

typedef struct {
    /* Phase 1 — Static */
    int16_t  ring[CALIB_RING_SIZE];
    uint16_t ring_head;
    uint16_t ring_count;
    double   welford_mean;
    double   welford_M2;
    uint32_t welford_n;

    /* Phase 2 — Dynamic */
    float    dyn_local_buf[CALIB_DYN_LOCAL_WIN];
    uint8_t  dyn_local_head;
    uint8_t  dyn_local_count;
    double   dyn_welford_mean;
    double   dyn_welford_M2;
    uint32_t dyn_welford_n;
    uint32_t dyn_outliers;

    /* Common */
    uint32_t             start_tick;
    uint32_t             total_fed;
    uint32_t             rejected_count;
    CalibPhaseInternal_t current_phase;
    CalibEngineStatus_t  status;
    bool                 started;
    bool                 phase2_notified;

    /* Results */
    float         noise_static;
    float         noise_dynamic;
    CalibResult_t result;
} CalibEngine_t;

void                calib_engine_init(CalibEngine_t *eng);
void                calib_engine_start(CalibEngine_t *eng);
CalibEngineStatus_t calib_engine_feed(CalibEngine_t *eng, const AdcSample_t *sample);
uint8_t             calib_engine_get_progress(const CalibEngine_t *eng);
esp_err_t           calib_engine_get_result(const CalibEngine_t *eng, CalibResult_t *out_result);
CalibEngineStatus_t calib_engine_get_status(const CalibEngine_t *eng);
bool                calib_engine_phase2_started(CalibEngine_t *eng);
CalibPhaseInternal_t calib_engine_get_phase(const CalibEngine_t *eng);
SoilType_t           calib_classify_soil(float std_dev);
SensitivityMode_t    calib_recommend_sensitivity(SoilType_t soil);
void                 calib_compute_filter_params(SoilType_t soil, SensitivityMode_t mode,
                                                  CalibResult_t *out_result);

#ifdef __cplusplus
}
#endif
