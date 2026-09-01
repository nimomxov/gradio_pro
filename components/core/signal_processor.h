#pragma once
/**
 * @file signal_processor.h
 * @brief DSP signal processing pipeline for gradiometer data.
 *
 * PIPELINE (in order):
 *  1. Outlier Gate      ? reject spikes beyond outlier_sigma
 *  2. Moving Average    ? window size from sensitivity / calibration
 *  3. [KALMAN FILTER]   ? optional, togglable, replaces EMA when active
 *     OR
 *  4. EMA Smoother      ? used when Kalman disabled
 *  5. Baseline Subtract ? remove Earth field / sensor offset
 *  6. Noise Gate        ? suppress sub-threshold output
 *  7. Output Scaler     ? map deviation to [0, 1024]
 *
 * KALMAN FILTER:
 *  One-dimensional scalar Kalman on the post-MA signal.
 *  Q (process noise):   from device calibration Phase 2 = (noise/range)^2
 *                       auto-increases when soil is noisy (adaptive Q)
 *  R (measurement noise): from device calibration Phase 1 = noise_floor^2
 *
 * BASELINE SYSTEM (dual-track):
 *  baseline_fixed:    Set at calibration. Primary reference.
 *  baseline_adaptive: Tracks slow thermal drift when signal is near zero.
 */

#include "gradiometer_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SP_MIN_QUALITY          40u
#define SP_ADAPTIVE_DRIFT_LIMIT 300.0f
#define SP_STABILITY_FAST       0.05f
#define SP_STABILITY_DECAY      0.90f
#define SP_STABILITY_THRESHOLD  80.0f
#define SP_ADAPTIVE_UPDATE_RATE 0.002f
#define SP_KALMAN_Q_ADAPT_MAX   8.0f

/* -- BT Output Mode -- */
typedef enum {
    BT_MODE_RAW         = 0,  ///< Legacy integer: signal+512, no compression
    BT_MODE_NORMALIZED  = 1,  ///< + slow DC drift removal (tau~100s)
    BT_MODE_ENHANCED    = 2,  ///< NORMALIZED + tighter range x0.7 (more contrast)
    BT_MODE_VIS3D_FLOAT = 3,  ///< Float precision, linear, optimised for Visualizer3D
} BtOutputMode_t;

/* Backward-compatible alias */
#define BT_MODE_RAW_INT BT_MODE_RAW

typedef struct {
    uint8_t window_size;
    float   alpha_ema;
    float   outlier_sigma;
    float   noise_gate;
} SensParams_t;

typedef struct {
    float x;            /* State estimate              */
    float p;            /* Error covariance            */
    float q_base;       /* Base process noise (calib)  */
    float r;            /* Measurement noise (calib)   */
    float q_adaptive;   /* Current Q multiplier        */
    bool  initialised;
} KalmanState_t;

/* -- Spatial Filter State -- */
typedef struct {
    float    g_prev;        /* G[n-1] ? previous scan point   */
    float    g_prev2;       /* G[n-2] ? for 3-point filter    */
    uint8_t  point_count;   /* Points collected so far        */
    float    alpha_blend;   /* Mix: signal vs spatial (0..1)  */
    bool     initialised;
} SpatialState_t;

typedef struct {
    float    ma_buffer[GRAD_FILTER_MAX_WINDOW];
    uint8_t  ma_head;
    uint8_t  ma_window;
    float    ma_sum;           /* Running sum for O(1) moving average */

    float    baseline_fixed;
    float    baseline_adaptive;
    float    baseline_slow;

    float    ema_output;
    float    alpha_ema;

    KalmanState_t kf;
    bool     kalman_enabled;

    SpatialState_t sf;
    bool     spatial_enabled;  /* Experimental ? requires kalman_enabled */

    float    noise_variance;
    float    signal_variance;
    float    stability_score;
    bool     is_stable;

    SensParams_t      active_params;
    SensitivityMode_t mode;

    float    prev_output;
    uint32_t sample_count;
    bool     calibrated;
    bool     initialized;
    bool     scan_mode;
    BtOutputMode_t bt_mode;
    uint8_t  soil_type;            /* SoilType_t ? set from CalibResult */
    uint8_t  scan_heading;         /* 0=N 1=E 2=S 3=W 4=NONE(disabled) */
    bool     heading_comp_enabled;
    float    heading_corrections[4]; /* from device_cal Phase 3A */
    float    bt_range_fixed;       /* k ? noise_floor ? fixed at calibration */
    /* FIX: bt_drift moved from static global into struct.
     * Previously a file-level static float, it was read by pipeline_bt (Core0)
     * and reset by sp_set_bt_mode (called from Core1). float is not atomic
     * on Xtensa dual-core. Now it lives here ? only signal_task touches it. */
    float    bt_drift;             /* slow BT DC drift tracker ? in-struct, Core0 only */
    float    last_bt_float;        /* VIS3D_FLOAT: float value before uint16 truncation */
    float    calibrated_noise_var; /* noise_variance frozen at calibration end ? HIGH_NOISE ref */
    /* -- Signal notification state (read-only observer ? never touches DSP) -- */
    uint8_t  spike_consec;         /* consecutive raw-spike samples counter    */
    uint32_t spike_start_ms;       /* timestamp of first spike sample (for width check) */
    float    spike_rate_ema;       /* EMA of spike events ? reflects sustained EMI env */
    float    prev_raw;             /* previous raw sample for d/dt spike detection */
    float    prev_filtered;        /* previous filtered value (kept for legacy callers) */
    uint32_t drift_report_ts_ms;   /* last drift notification timestamp */
    float    baseline_prev_report; /* baseline value at last drift report */
    /* Uncalibrated warmup ? first 100 samples build initial baseline */
    uint8_t  uncalib_warmup_n;     /* samples collected so far (0?100) */
    double   uncalib_warmup_sum;   /* running sum for mean computation  */
    float    uncalib_initial_mean; /* mean frozen after warmup          */
    bool     uncalib_warmup_done;  /* true after first 100 samples      */
} SignalProcessor_t;

void sp_init(SignalProcessor_t *sp);
void sp_apply_calibration(SignalProcessor_t *sp, const CalibResult_t *result);

/**
 * @brief Configure processor for uncalibrated operation (user skipped calibration).
 *
 * baseline_fixed = 0 ? output_scaled = clamp(signal + 512, 0, 1024)
 * Provides a valid centred signal for field testing without calibration.
 * sp->calibrated = false ? UI can show "? ??? ??????" indicator.
 */
void sp_apply_uncalibrated(SignalProcessor_t *sp);

/**
 * @brief Apply device calibration parameters to signal processor.
 * Uses flat params to avoid circular dependency with device_calibration.
 * Call from signal_task after devcal_load().
 */
typedef struct {
    float   noise_floor;      ///< Phase 1 std dev
    float   dynamic_range;    ///< Phase 2 peak-to-peak
    float   kalman_Q;         ///< Phase 2 computed Q
    float   kalman_R;         ///< Phase 1 computed R
    float   kalman_Q_final;   ///< Phase 5 optimised Q
    float   kalman_R_final;   ///< Phase 5 optimised R
    float   detection_limit;  ///< Phase 5 min detectable
} SpDeviceParams_t;

void sp_apply_device_params(SignalProcessor_t *sp,
                             const SpDeviceParams_t *params);

void sp_set_sensitivity(SignalProcessor_t *sp, SensitivityMode_t mode,
                        const CalibResult_t *calib);

/* Kalman control */
void                 sp_set_kalman(SignalProcessor_t *sp, bool enable);

/**
 * @brief Enable Kalman + Spatial filter (Experimental).
 *
 * Spatial filter computes the spatial gradient between consecutive
 * scan points: dG = G[n] - G[n-1], blended with the absolute signal.
 *
 * RULES:
 *   - Spatial requires Kalman ? enabling spatial auto-enables Kalman.
 *   - Disabling Kalman auto-disables spatial.
 *   - Spatial only activates in scan mode (set via sp_set_scan_mode).
 *   - In Live mode, spatial is silently bypassed.
 *
 * @param sp     Processor instance.
 * @param enable true = Kalman+Spatial, false = spatial off.
 */
void sp_set_spatial(SignalProcessor_t *sp, bool enable);

/**
 * @brief Notify processor of scan mode change.
 * Spatial filter resets its point history on mode entry.
 * @param sp        Processor instance.
 * @param scanning  true = scan active, false = live/idle.
 */
void sp_set_scan_mode(SignalProcessor_t *sp, bool scanning);

bool sp_spatial_is_enabled(const SignalProcessor_t *sp);
const KalmanState_t *sp_get_kalman_state(const SignalProcessor_t *sp);
bool                 sp_kalman_is_enabled(const SignalProcessor_t *sp);

bool  sp_process(SignalProcessor_t *sp,
                 const AdcSample_t *sample,
                 ProcessedSample_t *out);

float sp_get_stability(const SignalProcessor_t *sp);
float sp_get_snr(const SignalProcessor_t *sp);
bool  sp_is_stable(const SignalProcessor_t *sp);
float sp_get_baseline(const SignalProcessor_t *sp);
void  sp_reset_adaptive(SignalProcessor_t *sp);

/* Heading compensation */
void sp_set_heading(SignalProcessor_t *sp, uint8_t heading);
void sp_set_heading_comp(SignalProcessor_t *sp, bool enable);
void sp_load_heading_corrections(SignalProcessor_t *sp,
                                  const float corrections[4]);

/* BT output mode */
void           sp_set_bt_mode(SignalProcessor_t *sp, BtOutputMode_t mode);
BtOutputMode_t sp_get_bt_mode(const SignalProcessor_t *sp);

#ifdef __cplusplus
}
#endif
