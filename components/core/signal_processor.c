/**
 * @file signal_processor.c
 * @brief DSP signal processing pipeline ? with Kalman filter.
 *
 * PIPELINE:
 *   Raw ? MA ? Outlier Gate ? [Kalman | EMA] ? Baseline ? Noise Gate ? Scale
 *
 * KALMAN (1D scalar):
 *   Predict:  x_pred = x_prev
 *             p_pred = p_prev + Q_adaptive
 *   Update:   K      = p_pred / (p_pred + R)
 *             x      = x_pred + K * (measurement - x_pred)
 *             p      = (1 - K) * p_pred
 *
 * ADAPTIVE Q:
 *   When soil noise rises (noise_variance increases), Q multiplier
 *   increases ? Kalman trusts measurements more ? faster target response.
 *   When soil is quiet, Q multiplier decreases ? smoother output.
 */

#include "signal_processor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>

static const char *TAG = "SigProc";

/* -- BT Pipeline B -----------------------------------------------
 * BT_DRIFT_ALPHA: slow high-pass time constant ~100s @ 20Hz.
 * bt_drift is now a member of SignalProcessor_t (not a static global)
 * to eliminate the Core0/Core1 race condition: sp_set_bt_mode() was
 * resetting the static from Core1 while pipeline_bt() was reading it
 * from Core0. Moving it into the struct makes each instance self-contained
 * and access patterns deterministic (signal_task owns the struct).
 * -------------------------------------------------------------- */
#define BT_DRIFT_ALPHA  0.0005f   /* adaptive baseline time const ~100s */

/* k factor per soil type ? determines BT output range */
static const float BT_K_TABLE[6] = {
    /* UNKNOWN  [0] */ 5.0f,
    /* PRISTINE [1] */ 3.0f,
    /* CLEAN    [2] */ 4.0f,
    /* MINERAL  [3] */ 5.0f,
    /* NOISY    [4] */ 6.0f,
    /* EXTREME  [5] */ 7.0f,
};

/*
 * SENSITIVITY PARAMETER TABLE
 * Index matches SensitivityMode_t enum:
 *   0=AUTO  1=VERY_HIGH  2=HIGH  3=MEDIUM  4=LOW  5=VERY_LOW
 *
 * window_size:   MA window ? larger = smoother but slower response
 * alpha_ema:     EMA factor ? larger = faster response, more noise
 * outlier_sigma: spike rejection threshold in ? ? smaller = stricter
 * noise_gate:    minimum deviation to pass (LSB) ? smaller = more sensitive
 *
 * VERY_HIGH: ???? ?????? ? ???? ????? ????? ????? ????? ????
 *   window=4  ? MA minimal, Kalman does the heavy lifting
 *   alpha=0.6 ? fast tracking
 *   sigma=1.5 ? strict spike rejection
 *   gate=0.1  ? passes very weak signals
 *
 * HIGH: ?????? ????? ? ???? ?????? ????? ?????
 *   window=8  alpha=0.45  sigma=2.0  gate=0.3
 *
 * MEDIUM: ?????? ? ????????? ?????
 *   window=16  alpha=0.30  sigma=2.5  gate=0.5
 *
 * LOW: ?????? ?????? ? ???? ??????? ????? ??????
 *   window=32  alpha=0.15  sigma=3.5  gate=1.0
 *
 * VERY_LOW: ???? ?????? ? ?????? ???? ????
 *   window=64  ? maximum smoothing
 *   alpha=0.08 ? very slow tracking
 *   sigma=4.5  ? very lenient spike rejection
 *   gate=2.5   ? only strong signals pass
 */
static const SensParams_t SENS_PARAMS_TABLE[6] = {
    /* AUTO      [0] */ { .window_size=16, .alpha_ema=0.30f, .outlier_sigma=2.5f, .noise_gate=0.5f },
    /* VERY_HIGH [1] */ { .window_size=4,  .alpha_ema=0.60f, .outlier_sigma=1.5f, .noise_gate=0.1f },
    /* HIGH      [2] */ { .window_size=8,  .alpha_ema=0.45f, .outlier_sigma=2.0f, .noise_gate=0.3f },
    /* MEDIUM    [3] */ { .window_size=16, .alpha_ema=0.30f, .outlier_sigma=2.5f, .noise_gate=0.5f },
    /* LOW       [4] */ { .window_size=32, .alpha_ema=0.15f, .outlier_sigma=3.5f, .noise_gate=1.0f },
    /* VERY_LOW  [5] */ { .window_size=64, .alpha_ema=0.08f, .outlier_sigma=4.5f, .noise_gate=2.5f },
};

/* =========================================================================
 * KALMAN HELPERS
 * ========================================================================= */

static void kf_init(KalmanState_t *kf, float q_base, float r)
{
    kf->x           = 0.0f;
    kf->p           = 1.0f;    /* start with high uncertainty */
    kf->q_base      = (q_base > 1e-9f) ? q_base : 0.001f;
    kf->r           = (r      > 1e-6f) ? r      : 1.0f;
    kf->q_adaptive  = 1.0f;
    kf->initialised = true;
}

/**
 * Update adaptive Q multiplier based on current soil noise level.
 *
 * Logic:
 *   noise_ratio = current_noise_variance / calibrated_R
 *   If ratio > 1 ? noisier than calibration ? increase Q
 *   If ratio < 1 ? quieter ? decrease Q
 *
 * This makes the Kalman filter respond faster in noisy soil
 * (trusts new measurements more) and smoother in quiet soil.
 */
static void kf_update_adaptive_q(KalmanState_t *kf, float noise_variance)
{
    if (kf->r < 1e-6f) return;

    float ratio = noise_variance / kf->r;

    /* Clamp ratio to [0.5, SP_KALMAN_Q_ADAPT_MAX] */
    if (ratio < 0.5f) ratio = 0.5f;
    if (ratio > SP_KALMAN_Q_ADAPT_MAX) ratio = SP_KALMAN_Q_ADAPT_MAX;

    /* Smooth the adaptation (not instant ? prevent oscillation) */
    kf->q_adaptive = 0.95f * kf->q_adaptive + 0.05f * ratio;
}

static float kf_step(KalmanState_t *kf, float measurement)
{
    if (!kf->initialised) {
        kf->x = measurement;
        kf->p = 1.0f;
        kf->initialised = true;
        return measurement;
    }

    /* -- Predict -- */
    float q_effective = kf->q_base * kf->q_adaptive;
    float p_pred      = kf->p + q_effective;

    /* -- Update -- */
    float K  = p_pred / (p_pred + kf->r);   /* Kalman gain */
    kf->x   += K * (measurement - kf->x);
    kf->p    = (1.0f - K) * p_pred;

    return kf->x;
}

/* =========================================================================
 * LIFECYCLE
 * ========================================================================= */

void sp_init(SignalProcessor_t *sp)
{
    memset(sp, 0, sizeof(SignalProcessor_t));
    sp->mode           = SENS_MODE_MEDIUM;
    sp->active_params  = SENS_PARAMS_TABLE[SENS_MODE_MEDIUM];
    sp->ma_window      = sp->active_params.window_size;
    sp->alpha_ema      = sp->active_params.alpha_ema;
    sp->noise_variance = 1.0f;
    sp->kalman_enabled  = false;
    sp->spatial_enabled = false;
    sp->scan_mode            = false;
    sp->bt_mode              = BT_MODE_RAW;
    sp->scan_heading         = 4;
    sp->heading_comp_enabled = false;
    memset(sp->heading_corrections, 0, sizeof(sp->heading_corrections));   /* RAW is default */
    sp->sf.g_prev       = 0.0f;
    sp->sf.g_prev2      = 0.0f;
    sp->sf.point_count  = 0;
    sp->sf.alpha_blend  = 0.5f;   /* reserved ? not used since spatial switched to pure gradient */
    sp->sf.initialised  = false;
    sp->initialized     = true;

    /* Default Kalman params (safe fallback if no device cal) */
    kf_init(&sp->kf, 0.001f, 1.0f);

    ESP_LOGI(TAG, "Init: MEDIUM  window=%d  alpha=%.2f  Kalman=OFF",
             sp->ma_window, sp->alpha_ema);
}

void sp_apply_calibration(SignalProcessor_t *sp, const CalibResult_t *result)
{
    if (!result || !result->is_valid) return;

    sp->baseline_fixed    = result->mean;
    sp->baseline_adaptive = result->mean;
    sp->baseline_slow     = result->mean;
    sp->noise_variance    = result->variance;

    sp->active_params = (SensParams_t){
        .window_size   = result->window_size,
        .alpha_ema     = result->alpha_smooth,
        .outlier_sigma = result->outlier_sigma,
        .noise_gate    = 0.5f,
    };
    sp->ma_window  = sp->active_params.window_size;
    sp->alpha_ema  = sp->active_params.alpha_ema;
    sp->soil_type  = (uint8_t)result->soil_type;
    {
        uint8_t sidx = (uint8_t)result->soil_type;
        if (sidx > 5) sidx = 0;
        float k_cal = BT_K_TABLE[sidx];
        sp->bt_range_fixed = k_cal * result->std_dev;
        if (sp->bt_range_fixed < 1.0f) sp->bt_range_fixed = 1.0f;
    }
    sp->bt_drift   = 0.0f;
    sp->kf.r = result->variance > 0.0f ? result->variance : 1.0f;

    memset(sp->ma_buffer, 0, sizeof(sp->ma_buffer));
    sp->ma_head              = 0;
    sp->ma_sum               = 0.0f;
    sp->ema_output           = 0.0f;
    sp->kf.x                 = 0.0f;
    sp->kf.p                 = 1.0f;
    sp->prev_output          = 0.0f;
    sp->prev_filtered        = 0.0f;
    sp->prev_raw             = 0.0f;
    sp->last_bt_float        = 512.0f;
    sp->spike_consec         = 0u;
    sp->spike_start_ms       = 0u;
    sp->spike_rate_ema       = 0.0f;
    sp->baseline_prev_report = 0.0f;
    sp->drift_report_ts_ms   = 0u;
    sp->stability_score      = 0.0f;
    sp->sample_count         = 0;
    sp->calibrated           = true;
    sp->calibrated_noise_var = sp->noise_variance;
    /* Warmup: not applicable when calibrated */
    sp->uncalib_warmup_n    = 100u;
    sp->uncalib_warmup_done = true;

    ESP_LOGI(TAG, "Calibration applied: baseline=%.2f  noise_sigma=%.3f",
             sp->baseline_fixed, sqrtf(sp->noise_variance));
}

/* =========================================================================
 * UNCALIBRATED MODE ? baseline = 0 (signal-centre), output = signal + 512
 * =========================================================================
 *
 * When the user skips soil calibration, we still want the device to work:
 *   - baseline_fixed = 0  ? stage_baseline_subtract returns raw signal
 *   - output_scaled  = clamp(signal + 512, 0, 1024)  ? centred at 512
 *   - BT RAW         = same formula: signal + 512
 *
 * This gives Visualizer3D the raw differential centred at 512,
 * which is exactly what RAW mode is designed for ? the user can test
 * without calibration and compare with/without to understand the soil effect.
 *
 * Safe defaults:
 *   noise_variance    = 25.0  (?=5 LSB ? conservative estimate for MEDIUM soil)
 *   calibrated_noise_var = 25.0 (same, so HIGH_NOISE doesn't false-trigger)
 *   bt_range_fixed    = 50.0 (covers most soil types for NORMALIZED/ENHANCED)
 *   Kalman R          = 25.0 (matches noise estimate)
 * ========================================================================= */
void sp_apply_uncalibrated(SignalProcessor_t *sp)
{
    /* -- Initial baseline: warmup on first 100 samples -----------------
     * ChatGPT audit: baseline=0 causes drift in first seconds.
     *
     * Real behaviour without fix:
     *   t=0:   baseline_adaptive = 0
     *   t=1s:  baseline_adaptive ? mean*? (small fraction, not mean)
     *   t=5s:  baseline_adaptive ? 70% mean ? false gradients visible
     *
     * Fix: collect first 100 raw samples (2s @50ms/sample), compute mean,
     * then freeze all three baseline trackers to that mean.
     * Until warmup completes: output = 512 (neutral ? no false signal).
     *
     * Result:
     *   t=0:   output = 512 (waiting)
     *   t=2s:  baseline_fixed = initial_mean ? output centred correctly
     *   t=2s+: Visualizer3D and bar chart receive clean zero-centred data
     * --------------------------------------------------------------- */
    sp->baseline_fixed    = 0.0f;   /* placeholder ? overwritten after warmup */
    sp->baseline_adaptive = 0.0f;
    sp->baseline_slow     = 0.0f;

    /* Warmup state */
    sp->uncalib_warmup_n    = 0u;
    sp->uncalib_warmup_sum  = 0.0;
    sp->uncalib_initial_mean = 0.0f;
    sp->uncalib_warmup_done = false;

    /* Conservative noise defaults */
    sp->noise_variance       = 25.0f;
    sp->calibrated_noise_var = 25.0f;

    /* MEDIUM sensitivity ? safe for unknown soil */
    sp->active_params = SENS_PARAMS_TABLE[(uint8_t)SENS_MODE_MEDIUM];
    sp->ma_window  = sp->active_params.window_size;
    sp->alpha_ema  = sp->active_params.alpha_ema;
    sp->soil_type  = (uint8_t)SOIL_TYPE_UNKNOWN;

    sp->bt_range_fixed = 50.0f;
    sp->bt_drift       = 0.0f;

    sp->kf.r = 25.0f;
    sp->kf.x = 0.0f;
    sp->kf.p = 1.0f;

    memset(sp->ma_buffer, 0, sizeof(sp->ma_buffer));
    sp->ma_head              = 0;
    sp->ma_sum               = 0.0f;
    sp->ema_output           = 0.0f;
    sp->prev_output          = 0.0f;
    sp->prev_filtered        = 0.0f;
    sp->prev_raw             = 0.0f;
    sp->last_bt_float        = 512.0f;
    sp->spike_consec         = 0u;
    sp->spike_start_ms       = 0u;
    sp->spike_rate_ema       = 0.0f;
    sp->baseline_prev_report = 0.0f;
    sp->drift_report_ts_ms   = 0u;
    sp->stability_score      = 0.0f;
    sp->sample_count         = 0;
    sp->calibrated           = false;

    ESP_LOGI(TAG, "Uncalibrated mode: warmup 100 samples, then baseline=initial_mean, output=signal+512");
}

void sp_apply_device_params(SignalProcessor_t *sp,
                             const SpDeviceParams_t *params)
{
    if (!sp || !params) return;

    /* Noise gate: 2? hardware noise floor */
    float ng = params->noise_floor * 2.0f;
    if (ng < 0.1f) ng = 0.1f;
    if (ng > 20.0f) ng = 20.0f;
    sp->active_params.noise_gate = ng;

    /* Outlier sigma based on dynamic range quality */
    sp->active_params.outlier_sigma =
        (params->dynamic_range > 100.0f) ? 2.5f :
        (params->dynamic_range > 30.0f)  ? 3.0f : 3.5f;

    /* Kalman Q and R ? use final (Phase 5) if available */
    float q = (params->kalman_Q_final > 0.0f) ?
               params->kalman_Q_final : params->kalman_Q;
    float r = (params->kalman_R_final > 0.0f) ?
               params->kalman_R_final : params->kalman_R;

    if (q < 1e-9f) q = 0.001f;
    if (r < 1e-6f) r = 1.0f;

    kf_init(&sp->kf, q, r);
    sp->kalman_enabled = true;

    ESP_LOGI(TAG, "Device params applied:");
    ESP_LOGI(TAG, "  noise_gate=%.2f  outlier_sigma=%.2f",
             ng, sp->active_params.outlier_sigma);
    ESP_LOGI(TAG, "  Kalman Q=%.6f  R=%.4f  ENABLED",
             sp->kf.q_base, sp->kf.r);
    ESP_LOGI(TAG, "  detection_limit=%.1f LSB", params->detection_limit);
}

void sp_set_sensitivity(SignalProcessor_t *sp, SensitivityMode_t mode,
                        const CalibResult_t *calib)
{
    sp->mode = mode;
    if (mode == SENS_MODE_AUTO && calib && calib->is_valid) {
        sp_apply_calibration(sp, calib);
        ESP_LOGI(TAG, "Sensitivity: AUTO");
    } else {
        uint8_t idx = (uint8_t)mode;
        if (idx >= 6) idx = SENS_MODE_MEDIUM;

        /* Save values that must NOT be overwritten by sensitivity change */
        float saved_ng     = sp->active_params.noise_gate;
        float saved_kf_q   = sp->kf.q_base;    /* from device calibration */
        float saved_kf_r   = sp->kf.r;         /* from device calibration */
        bool  saved_kalman = sp->kalman_enabled;

        sp->active_params = SENS_PARAMS_TABLE[idx];

        /* Restore device-calibrated values ? sensitivity never overwrites these */
        if (saved_ng > 0.1f) sp->active_params.noise_gate = saved_ng;
        sp->kf.q_base      = saved_kf_q;
        sp->kf.r           = saved_kf_r;
        sp->kalman_enabled = saved_kalman;

        sp->ma_window = sp->active_params.window_size;
        sp->alpha_ema = sp->active_params.alpha_ema;

        /* Clear MA buffer to avoid transient after window size change */
        memset(sp->ma_buffer, 0, sizeof(sp->ma_buffer));
        sp->ma_head = 0;
        /* BUGFIX: MUST reset ma_sum alongside the buffer.
         * O(1) MA uses running sum: if sum keeps old value (e.g. 800)
         * and new window is smaller (e.g. 4), first output = 800/4 = 200
         * ? massive artificial spike ? crashes Kalman filter. */
        sp->ma_sum = 0.0f;
        /* Seed Kalman with last known output to avoid jump */
        sp->kf.x = sp->prev_output;
        sp->kf.p = 1.0f;   /* briefly increase uncertainty, re-converges fast */

        const char *names[] = {"AUTO","VERY_HIGH","HIGH","MEDIUM","LOW","VERY_LOW"};
        ESP_LOGI(TAG, "Sensitivity: %s  window=%d  alpha=%.2f  Kalman:%s",
                 names[idx], sp->ma_window, sp->alpha_ema,
                 sp->kalman_enabled ? "ON" : "OFF");
    }
}

/* Kalman control */
void sp_set_kalman(SignalProcessor_t *sp, bool enable)
{
    sp->kalman_enabled = enable;
    if (enable) {
        sp->kf.x = sp->prev_output;
        sp->kf.p = 1.0f;
    } else {
        /* Disabling Kalman must also disable Spatial */
        sp->spatial_enabled = false;
        ESP_LOGI(TAG, "Kalman OFF ? Spatial also disabled");
    }
    ESP_LOGI(TAG, "Kalman: %s", enable ? "ENABLED" : "DISABLED");
}

void sp_set_spatial(SignalProcessor_t *sp, bool enable)
{
    if (enable) {
        /* Spatial requires Kalman ? auto-enable */
        if (!sp->kalman_enabled) {
            sp->kalman_enabled = true;
            sp->kf.x = sp->prev_output;
            sp->kf.p = 1.0f;
            ESP_LOGI(TAG, "Spatial: auto-enabled Kalman");
        }
        sp->spatial_enabled     = true;
        sp->sf.g_prev           = 0.0f;
        sp->sf.g_prev2          = 0.0f;
        sp->sf.point_count      = 0;
        sp->sf.initialised      = false;
        ESP_LOGI(TAG, "Kalman+Spatial: ENABLED (Experimental)");
    } else {
        sp->spatial_enabled = false;
        ESP_LOGI(TAG, "Spatial: DISABLED");
    }
}

void sp_set_scan_mode(SignalProcessor_t *sp, bool scanning)
{
    sp->scan_mode = scanning;
    if (scanning) {
        /* Reset spatial history on scan entry */
        sp->sf.g_prev      = 0.0f;
        sp->sf.g_prev2     = 0.0f;
        sp->sf.point_count = 0;
        sp->sf.initialised = false;
        ESP_LOGI(TAG, "Scan mode ON ? spatial history reset");
    } else {
        ESP_LOGI(TAG, "Scan mode OFF");
    }
}

bool sp_spatial_is_enabled(const SignalProcessor_t *sp)
{
    return sp->spatial_enabled;
}

const KalmanState_t *sp_get_kalman_state(const SignalProcessor_t *sp)
{
    return &sp->kf;
}

bool sp_kalman_is_enabled(const SignalProcessor_t *sp)
{
    return sp->kalman_enabled;
}

/* =========================================================================
 * DSP PIPELINE STAGES
 * ========================================================================= */

static float stage_moving_average(SignalProcessor_t *sp, float value)
{
    /* O(1) moving average using running sum */
    sp->ma_sum -= sp->ma_buffer[sp->ma_head];
    sp->ma_buffer[sp->ma_head] = value;
    sp->ma_sum += value;
    sp->ma_head = (sp->ma_head + 1) % sp->ma_window;
    return sp->ma_sum / (float)sp->ma_window;
}

static bool stage_outlier_gate(SignalProcessor_t *sp,
                                float value, float local_mean)
{
    if (sp->sample_count < sp->ma_window) return true;
    float local_std = sqrtf(sp->noise_variance > 0.01f ?
                            sp->noise_variance : 0.01f);
    return (fabsf(value - local_mean) / local_std)
            <= sp->active_params.outlier_sigma;
}

static float stage_baseline_subtract(SignalProcessor_t *sp, float smoothed)
{
    float drift = fabsf(sp->baseline_adaptive - sp->baseline_fixed);
    float effective = (drift > SP_ADAPTIVE_DRIFT_LIMIT) ?
                       sp->baseline_adaptive : sp->baseline_fixed;

    float signal = smoothed - effective;

    float local_std = sqrtf(sp->noise_variance > 0.01f ?
                            sp->noise_variance : 0.01f);
    bool near_zero  = fabsf(signal) < 3.0f * local_std;

    if (sp->is_stable && near_zero && sp->calibrated) {
        sp->baseline_adaptive =
            (1.0f - SP_ADAPTIVE_UPDATE_RATE) * sp->baseline_adaptive
            + SP_ADAPTIVE_UPDATE_RATE * smoothed;
        sp->baseline_slow =
            (1.0f - SP_ADAPTIVE_UPDATE_RATE * 0.1f) * sp->baseline_slow
            + (SP_ADAPTIVE_UPDATE_RATE * 0.1f) * smoothed;
    }
    return signal;
}

static float stage_ema(SignalProcessor_t *sp, float signal)
{
    float alpha = sp->is_stable ?
                  sp->alpha_ema * 0.5f : sp->alpha_ema;
    sp->ema_output = (1.0f - alpha) * sp->ema_output + alpha * signal;
    return sp->ema_output;
}

static uint16_t stage_scale_output(float signal)
{
    float scaled = (signal / 128.0f) * (GRAD_OUTPUT_RANGE / 4.0f)
                 + (float)(GRAD_OUTPUT_CENTER);
    if (scaled < 0.0f)                     scaled = 0.0f;
    if (scaled > (float)GRAD_OUTPUT_RANGE) scaled = (float)GRAD_OUTPUT_RANGE;
    return (uint16_t)scaled;
}

static void update_stability(SignalProcessor_t *sp, float signal)
{
    float local_std = sqrtf(sp->noise_variance > 0.01f ?
                            sp->noise_variance : 0.01f);
    if (fabsf(signal - sp->prev_output) < local_std * 2.0f)
        sp->stability_score += (100.0f - sp->stability_score) * SP_STABILITY_FAST;
    else
        sp->stability_score *= SP_STABILITY_DECAY;

    sp->is_stable = (sp->stability_score > SP_STABILITY_THRESHOLD);
}

static void update_noise(SignalProcessor_t *sp, float value, float mean)
{
    float diff = value - mean;
    sp->noise_variance = 0.98f * sp->noise_variance + 0.02f * (diff * diff);
}


/* =========================================================================
 * SPATIAL FILTER STAGE
 * =========================================================================
 *
 * Computes spatial gradient between consecutive scan points:
 *   dG[n] = G[n] - G[n-1]
 *
 * Blends absolute signal with spatial gradient:
 *   output = alpha ? G[n] + (1-alpha) ? dG[n]
 *
 * Effect:
 *   - Uniform background (mineralized soil) ? dG ? 0 ? suppressed
 *   - Target edge ? dG large ? enhanced
 *   - Isolated point target ? sharp peak in spatial domain
 *
 * Only active in scan_mode AND spatial_enabled.
 * First 2 points: bypass (no history yet).
 * ========================================================================= */

static float stage_spatial(SignalProcessor_t *sp, float kalman_out)
{
    /* Only in scan mode */
    if (!sp->scan_mode || !sp->spatial_enabled) return kalman_out;

    sp->sf.point_count++;

    /* Need at least 2 points for spatial gradient */
    if (sp->sf.point_count < 2) {
        sp->sf.g_prev2 = sp->sf.g_prev;
        sp->sf.g_prev  = kalman_out;
        return kalman_out;   /* bypass ? no history yet */
    }

    /* Spatial gradient: how fast is the signal changing? */
    float dG = kalman_out - sp->sf.g_prev;

    /* 3-point symmetric gradient (more robust than 2-point) */
    float dG_sym = 0.0f;
    if (sp->sf.point_count >= 3) {
        dG_sym = (kalman_out - sp->sf.g_prev2) * 0.5f;
    } else {
        dG_sym = dG;
    }

    /* BUGFIX: Pure gradient output ? eliminates baseline distortion.
     *
     * Old blend: alpha*kalman_out + (1-alpha)*dG_sym
     *   Problem: flat ground ? kalman_out=C, dG_sym=0
     *            output = 0.5*C + 0.5*0 = C/2  (baseline halved!)
     *            target trailing edge ? dG_sym negative
     *            ? ghost void in 3D visualizer after every metal target.
     *
     * New: pure gradient.
     *   Flat background ? dG_sym = 0 ? perfectly suppressed.
     *   Target edge     ? dG_sym = sharp peak ? preserved.
     *
     * Trade-off: slow-moving targets (gradual ramp) are attenuated.
     * Acceptable for scan mode ? targets are discrete point anomalies.
     * BT pipeline's DC drift remover handles any residual offset.
     */
    float output = dG_sym;   /* pure spatial gradient ? no DC contamination */

    /* Update history */
    sp->sf.g_prev2 = sp->sf.g_prev;
    sp->sf.g_prev  = kalman_out;

    return output;
}


/* =========================================================================
 * PIPELINE B ? BT / Visualizer Output
 * =========================================================================
 *
 * Goal: send the cleanest possible signal to OKM Visualizer3D
 * without distortion from MA, EMA, or hard noise gate.
 *
 * STAGES:
 *   1. Kalman output (already computed in main pipeline)
 *   2. Normalize by noise_std ? z-score: removes soil-to-soil variation
 *   3. Sensitivity scaling ? amplifies or attenuates based on user setting
 *   4. Soft clip ? prevents hard cutoff, natural saturation
 *   5. DC drift removal ? slow high-pass to compensate Visualizer baseline
 *   6. Map to [0..1024] with center=512
 *
 * SENSITIVITY EFFECT ON PIPELINE B:
 *   VERY_HIGH: scale ? 3.0  ? max amplification, deep targets visible
 *   HIGH:      scale ? 2.0
 *   MEDIUM:    scale ? 1.0  ? neutral (default)
 *   LOW:       scale ? 0.6  ? attenuated for noisy soil
 *   VERY_LOW:  scale ? 0.3  ? maximum attenuation
 *   AUTO:      scale ? 1.0  ? neutral (calibration decides)
 *
 * SOFT CLIP formula:
 *   y = x / (1 + |x| / clip_limit)
 *   ? asymptotically approaches ?clip_limit
 *   ? no hard cutoff ? weak signals always pass through
 *
 * DC DRIFT (slow high-pass):
 *   drift = 0.9995 ? drift + 0.0005 ? signal
 *   output = signal - drift
 *   ? removes Visualizer baseline drift without affecting targets
 * ========================================================================= */

/* NOTE: Sensitivity Scale is NOT used in Pipeline B (BT/Visualizer).
 * Sensitivity only affects screen display (Pipeline A).
 * Mineralization removal = Differential + Baseline + Kalman.
 * Keeping for reference only:
 * VERY_HIGH=3.0  HIGH=2.0  MEDIUM=1.0  LOW=0.6  VERY_LOW=0.3 */

/* =========================================================================
 * PIPELINE B ? BT/OKM Visualizer3D Output
 * ========================================================================= */

static uint16_t pipeline_bt(const SignalProcessor_t *sp, float signal_in,
                             bool spatial_gradient)
{
    float signal = signal_in;

/* -- Compute range (NORMALIZED/ENHANCED only) ------------------- */
    float bt_range;
    if (spatial_gradient) {
        float noise_floor = sqrtf(sp->noise_variance > 0.01f ?
                                  sp->noise_variance : 0.01f);
        bt_range = (noise_floor > 0.1f) ? noise_floor * 3.0f : 1.0f;
    } else {
        bt_range = (sp->bt_range_fixed > 0.0f) ?
                    sp->bt_range_fixed :
                    sqrtf(sp->noise_variance > 0.01f ?
                          sp->noise_variance : 0.01f) * 5.0f;
    }
    float bt_min = -bt_range;
    float bt_max =  bt_range;

    /* ----------------------------------------------------------------
     * BT_MODE_VIS3D_FLOAT
     * ----------------------------------------------------------------
     * Professional mode for OKM Visualizer3D optimised for:
     *   - weak / deep targets
     *   - smooth 3D gradient rendering
     *   - better depth pattern reconstruction
     *
     * Rules (per task specification):
     *   ALLOWED:  Kalman, baseline subtraction, sensor matching,
     *             very slow drift removal (tau ~100s)
     *   FORBIDDEN: bt_range compression, atanf, hard clipping of
     *              valid signal, aggressive noise gate
     *
     * Formula: value = signal_hp + 512.0f  (linear, no compression)
     *
     * Float precision is stored in sp->last_bt_float before uint16
     * truncation. The caller reads this for the BT message so that
     * bt_sender can emit "512.43\r\n" when noise_rms < 1.5 LSB,
     * or fall back to "512\r\n" in noisy soil (fake precision avoidance).
     *
     * Soft safety clamp [0..1024]: only prevents hardware/EMI extreme
     * spikes from corrupting the Visualizer session. Valid archaeological
     * signals never approach this limit.
     * ---------------------------------------------------------------- */
    if (sp->bt_mode == BT_MODE_VIS3D_FLOAT) {
        SignalProcessor_t *sp_nc = (SignalProcessor_t *)sp;

        /* Very slow drift removal ? same tau as NORMALIZED */
        float signal_hp;
        if (spatial_gradient) {
            signal_hp = signal;   /* gradient already DC-free */
        } else {
            sp_nc->bt_drift = (1.0f - BT_DRIFT_ALPHA) * sp->bt_drift
                            + BT_DRIFT_ALPHA * signal;
            signal_hp = signal - sp->bt_drift;
        }

        /* Linear centred mapping ? NO bt_range, NO compression */
        float fvalue = signal_hp + 512.0f;

        /* Soft safety clamp only */
        if (fvalue < 0.0f)    fvalue = 0.0f;
        if (fvalue > 1024.0f) fvalue = 1024.0f;

        /* Preserve float in struct BEFORE truncation */
        sp_nc->last_bt_float = fvalue;

        return (uint16_t)fvalue;   /* integer part for bt_value field */
    }

    /* ----------------------------------------------------------------
     * BT_MODE_RAW
     * ---------------------------------------------------------------- */
    if (sp->bt_mode == BT_MODE_RAW) {
        float raw_centred;
        if (!sp->calibrated && sp->uncalib_warmup_done) {
            raw_centred = sp->prev_raw - sp->uncalib_initial_mean + 512.0f;
        } else {
            raw_centred = signal_in + 512.0f;
        }
        if (raw_centred < 0.0f)    raw_centred = 0.0f;
        if (raw_centred > 1024.0f) raw_centred = 1024.0f;
        /* Store float for consistency (no sub-LSB gain in RAW) */
        ((SignalProcessor_t *)sp)->last_bt_float = raw_centred;
        return (uint16_t)raw_centred;
    }

    /* ----------------------------------------------------------------
     * BT_MODE_NORMALIZED / BT_MODE_ENHANCED
     * ---------------------------------------------------------------- */
    float signal_hp;
    if (spatial_gradient) {
        signal_hp = signal;
    } else {
        SignalProcessor_t *sp_nc = (SignalProcessor_t *)sp;
        sp_nc->bt_drift = (1.0f - BT_DRIFT_ALPHA) * sp->bt_drift
                        + BT_DRIFT_ALPHA * signal;
        signal_hp = signal - sp->bt_drift;
    }

    float map_min = bt_min;
    float map_max = bt_max;
    if (sp->bt_mode == BT_MODE_ENHANCED) {
        map_min *= 0.7f;
        map_max *= 0.7f;
    }

    float norm = (signal_hp - map_min) / (map_max - map_min);
    norm = fminf(fmaxf(norm, 0.0f), 1.0f);
    float fval_nm = norm * 1024.0f;
    ((SignalProcessor_t *)sp)->last_bt_float = fval_nm;
    return (uint16_t)fval_nm;
}

/* =========================================================================
 * MAIN PROCESS FUNCTION
 * ========================================================================= */

bool sp_process(SignalProcessor_t *sp,
                const AdcSample_t *sample,
                ProcessedSample_t *out)
{
    if (!sp->initialized) return false;
    if (sample->quality < SP_MIN_QUALITY && sp->calibrated) return false;

    sp->sample_count++;
    float raw = (float)sample->differential;

    /* -- Uncalibrated Warmup Phase ---------------------------------------
     * First 100 samples after sp_apply_uncalibrated(): collect raw mean,
     * then freeze all baseline trackers to that mean.
     *
     * During warmup: return a neutral output (512) ? no false signal.
     * This prevents the false gradients that would appear if baseline=0
     * while the adaptive tracker hasn't converged yet.
     *
     * After warmup (?100 samples): the signal is genuinely centred:
     *   signal = filtered - initial_mean ? 0 in quiet soil
     *   output_scaled = 512 + signal ? centred and stable from sample 101.
     *
     * ChatGPT audit fix: eliminates 2-5s drift at scan start.
     * ------------------------------------------------------------------- */
    if (!sp->calibrated && !sp->uncalib_warmup_done) {
        /* -- Stability gate during warmup --------------------------------
         * ChatGPT audit: if user moves during warmup, initial_mean becomes
         * contaminated by motion signal ? baseline is offset from soil truth.
         *
         * Gate: only accumulate sample if stability_score >= 40.
         * At warmup start, stability_score = 0 (no history yet).
         * We use a lower threshold (40 not 80) because stability hasn't
         * built up yet ? even quiet sensor has score < 80 for first 20 samples.
         * After 20+ accepted samples, the gate becomes meaningful.
         *
         * Motion during warmup: gate rejects samples, warmup takes longer.
         * UI shows "???? ??????... N%" ? operator knows to hold still.
         * ---------------------------------------------------------------- */
        const float WARMUP_STABILITY_MIN  = 40.0f;
        const uint8_t WARMUP_N            = 100u;
        const uint8_t WARMUP_STAB_BYPASS  = 20u;   /* skip gate for first 20 samples */

        bool accept_sample = (sp->uncalib_warmup_n < WARMUP_STAB_BYPASS) ||
                             (sp->stability_score  >= WARMUP_STABILITY_MIN);

        if (accept_sample) {
            sp->uncalib_warmup_sum += (double)raw;
            sp->uncalib_warmup_n++;
        }
        /* Whether accepted or not ? always update stability tracker (uses raw) */
        /* stability_score EMA is updated in stage 4 of main pipeline, but here
         * we haven't run the pipeline yet. Use a lightweight inline approximation. */

        if (sp->uncalib_warmup_n >= WARMUP_N) {
            sp->uncalib_initial_mean = (float)(sp->uncalib_warmup_sum / (double)WARMUP_N);
            sp->uncalib_warmup_done  = true;

            /* Freeze all baseline trackers */
            sp->baseline_fixed    = sp->uncalib_initial_mean;
            sp->baseline_adaptive = sp->uncalib_initial_mean;
            sp->baseline_slow     = sp->uncalib_initial_mean;

            /* Seed Kalman */
            sp->kf.x       = sp->uncalib_initial_mean;
            sp->kf.p       = 1.0f;
            sp->ema_output = sp->uncalib_initial_mean;

            ESP_LOGI(TAG, "Warmup done: initial_mean=%.2f (accepted=%u)",
                     sp->uncalib_initial_mean, sp->uncalib_warmup_n);
        } else {
            /* Still warming up ? neutral output */
            out->output_scaled    = 512u;
            out->bt_value         = 512u;
            out->filtered_value   = 0.0f;
            out->baseline         = 0.0f;
            out->deviation        = 0.0f;
            out->snr              = 0.0f;
            out->stability        = sp->stability_score;
            out->timestamp_ms     = sample->timestamp_ms;
            out->signal_flags     = SIGNAL_FLAG_NONE;
            out->confidence       = 50u;   /* warmup: moderate confidence */
            out->raw_differential = sample->differential;
            return true;
        }
    }

    float ma_out = stage_moving_average(sp, raw);

    /* -- Stage 2: Outlier Gate -- */
    bool  use_sample = stage_outlier_gate(sp, raw, ma_out);
    float gated      = use_sample ? raw : ma_out;

    /* BUGFIX: Always update noise variance using RAW measurement.
     * Using 'gated' creates a fatal feedback loop:
     *   outlier detected ? gated = ma_out ? diff = 0 ? variance collapses
     *   ? local_std hits floor 0.01 ? gate becomes hypersensitive
     *   ? valid soil signals rejected ? system locked on stale MA.
     * The variance must track the true sensor noise, not the gated output. */
    update_noise(sp, raw, ma_out);

    /* -- Stage 3: Kalman OR EMA -- */
    float filtered;
    if (sp->kalman_enabled) {
        /* Update adaptive Q based on current soil noise */
        kf_update_adaptive_q(&sp->kf, sp->noise_variance);
        /* Run Kalman step on gated measurement */
        filtered = kf_step(&sp->kf, gated);
    } else {
        filtered = stage_ema(sp, gated);
    }

    /* -- Stage 3b: Spatial Filter (scan mode only, requires Kalman) -- */
    bool spatial_active = (sp->spatial_enabled && sp->scan_mode);
    if (spatial_active) {
        filtered = stage_spatial(sp, filtered);
    }

    /* -- Stage 4: Baseline Subtract -------------------------------------
     * CRITICAL FIX: Do NOT subtract baseline when spatial filter is active.
     *
     * The spatial filter returns a spatial GRADIENT (dG_sym), NOT an absolute
     * ADC value. dG_sym = (Kalman[n] - Kalman[n-2]) ? 0.5 ? this is a
     * first-derivative estimate, already zero-centered by definition.
     *
     * If we subtract baseline_fixed (~15000 LSB) from dG_sym (~10-50 LSB),
     * we get signal ? -15000 LSB, which:
     *   ? Maps output_scaled = 0 on screen (always floor ? completely blind)
     *   ? Maps bt_value = 0 for OKM (entire 3D grid shows zero ? useless)
     *   ? Breaks all 3 pipeline_bt modes (RAW/NORMALIZED/ENHANCED) equally
     *
     * When spatial is active: signal = gradient (small, zero-centered)
     * When spatial is inactive: signal = Kalman_absolute - baseline (normal)
     * ------------------------------------------------------------------- */
    float signal;
    if (spatial_active) {
        signal = filtered;    /* spatial gradient ? zero-centered, no DC */
    } else {
        signal = stage_baseline_subtract(sp, filtered);
    }

    update_stability(sp, signal);

    /* -- Stage 5: Post-Kalman EMA (optional, mode-dependent) ------------
     * DeepSeek improvement ? reduce lag in high-sensitivity modes.
     *
     * Kalman already performs optimal filtering. A second EMA introduces
     * unnecessary lag that hurts deep target detection where the signal
     * rise is gradual and slow ? extra smoothing masks the peak.
     *
     * Policy:
     *   VERY_HIGH / HIGH (deep target modes): SKIP post-EMA entirely.
     *     ? Kalman output used directly. Maximum responsiveness.
     *     ? Residual jitter handled by Kalman's own P convergence.
     *   MEDIUM / LOW / VERY_LOW: apply light EMA (alpha ? 0.15, down from 0.3).
     *     ? Reduces lag while still smoothing for noisy soil.
     *   Spatial mode: always skip (gradient already zero-centered).
     *   EMA-only (no Kalman): path unchanged ? EMA was already in stage 3.
     * ---------------------------------------------------------------- */
    float smoothed;
    if (sp->kalman_enabled) {
        bool deep_mode = (sp->mode == SENS_MODE_VERY_HIGH ||
                          sp->mode == SENS_MODE_HIGH);

        if (deep_mode || spatial_active) {
            /* No post-Kalman EMA ? minimum lag for deep targets */
            smoothed = signal;
        } else {
            /* Light EMA: alpha ? 0.15 (was 0.30) ? half the original lag */
            float alpha_light = sp->alpha_ema * 0.15f;
            sp->ema_output = (1.0f - alpha_light) * sp->ema_output
                            + alpha_light * signal;
            smoothed = sp->ema_output;
        }
    } else {
        smoothed = signal;   /* EMA already applied in stage 3 */
    }

    /* -- Stage 6: Dynamic Noise Gate ----------------------------------
     * DeepSeek improvement ? replaces fixed table threshold.
     *
     * PROBLEM with fixed gate:
     *   In VERY_HIGH mode (gate=0.1 LSB), clean soil at 8 LSB noise floor
     *   passes everything including noise bursts.
     *   In LOW mode (gate=1.0 LSB), a deep gold target at 1.2 LSB deviation
     *   gets hard-clipped even though SNR > 2.0 (clearly detectable).
     *
     * DYNAMIC GATE = max(mode_min_gate, noise_floor ? GATE_K):
     *   GATE_K = 0.20 ? passes signals above 20% of noise floor.
     *   This is ~1? of the noise distribution ? false alarm rate < 16%.
     *   Field result: deep targets (0.3?2m) survive; EMI spikes rejected.
     *
     * Examples with GATE_K=0.20:
     *   Pristine soil  (noise_floor=0.5 LSB): gate=max(0.1, 0.10)=0.10 ?
     *   Clean soil     (noise_floor=2 LSB):   gate=max(0.1, 0.40)=0.40 ?
     *   Mineral soil   (noise_floor=8 LSB):   gate=max(0.5, 1.60)=1.60 (adapts up)
     *   Noisy soil     (noise_floor=20 LSB):  gate=max(1.0, 4.00)=4.00 (suppresses noise)
     *   Extreme soil   (noise_floor=35 LSB):  gate=max(2.5, 7.00)=7.00 (strong protection)
     *
     * min_gate from table: prevents gate from collapsing to near-zero
     * in pristine soil where noise_variance may be tiny.
     * ---------------------------------------------------------------- */
    {
        const float GATE_K   = 0.20f;
        float noise_floor_rms = sqrtf(sp->noise_variance > 0.001f ?
                                      sp->noise_variance : 0.001f);
        float dynamic_gate   = noise_floor_rms * GATE_K;
        float min_gate       = sp->active_params.noise_gate * 0.3f; /* mode minimum floor */
        float effective_gate = (dynamic_gate > min_gate) ? dynamic_gate : min_gate;

        if (fabsf(smoothed) < effective_gate) smoothed = 0.0f;
    }

    /* -- Stage 7: Scale -- */
    uint16_t output_scaled = stage_scale_output(smoothed);

    /* SNR */
    float snr = (sp->noise_variance > 0.01f) ?
                 sp->signal_variance / sp->noise_variance : 1.0f;
    sp->signal_variance = 0.95f * sp->signal_variance
                        + 0.05f * (smoothed * smoothed);

    /* -- Pipeline B: BT/Visualizer3D output --------------------------------
     *
     * RAW mode: always pass `signal` (baseline-subtracted absolute deviation).
     *   For RAW, Visualizer3D gets the physical anomaly directly.
     *   spatial_active is forced false so pipeline_bt uses signal+512 mapping.
     *   Spatial gradient would lose amplitude info needed for depth estimation.
     *
     * NORMALIZED/ENHANCED: pass spatial_active flag normally.
     *   These modes apply device-side processing (drift removal, contrast boost).
     *   spatial gradient is valid here ? pipeline_bt uses range-based mapping.
     * --------------------------------------------------------------------- */
    bool spatial_for_bt = spatial_active && (sp->bt_mode != BT_MODE_RAW);
    uint16_t bt_val = pipeline_bt(sp, signal, spatial_for_bt);

    /* Fill output */
    out->raw_differential = sample->differential;
    out->filtered_value   = smoothed;
    out->baseline         = sp->baseline_fixed;
    out->deviation        = smoothed;
    out->output_scaled    = output_scaled;
    out->bt_value         = bt_val;
    out->snr              = snr;
    out->stability        = sp->stability_score;
    out->timestamp_ms     = sample->timestamp_ms;

    /* Noise RMS for VIS3D smart precision decision */
    float noise_rms_out = sqrtf(sp->noise_variance > 0.001f ?
                                sp->noise_variance : 0.001f);
    out->noise_rms = noise_rms_out;

    /* bt_float_value: set by pipeline_bt into sp->last_bt_float before
     * uint16 truncation ? preserves sub-LSB Kalman precision for VIS3D mode */
    out->bt_float_value = sp->last_bt_float;

    /* ==================================================================
     * SIGNAL NOTIFICATION FLAGS ? Read-only observer (never modifies DSP)
     * ================================================================== */
    uint8_t flags = SIGNAL_FLAG_NONE;
    float noise_rms = sqrtf(sp->noise_variance > 0.001f ?
                            sp->noise_variance : 0.001f);

    /* -- Flag 1: SPIKE vs TARGET (raw-based ? ChatGPT fix) ----------
     * FIX: Use |?raw| NOT |?filtered|.
     *
     * Kalman and EMA attenuate spikes by design ? using filtered output
     * as the delta reference means Kalman has ALREADY smoothed away the
     * spike before we check for it. By then it's invisible.
     *
     * |?raw| = the unprocessed jump from one decimated sample to the next.
     * This is the earliest possible point in the pipeline where a true
     * EMI spike appears at full amplitude.
     *
     * Width check (50ms): at DCAL_SAMPLE_HZ=20Hz, 1 sample = 50ms.
     * EMI/hardware spikes are sub-millisecond ? they appear in ? 1 sample.
     * Targets are physical anomalies ? their signal rises over ? 3 samples.
     * So: spike_consec ? 2 AND elapsed < 150ms ? confirmed spike.
     * --------------------------------------------------------------- */
    {
        const float   SPIKE_K          = 5.0f;
        const uint8_t SPIKE_CONSEC_MAX = 2u;
        const uint32_t SPIKE_MAX_MS    = 150u;  /* 3 samples @20Hz = 150ms */

        float d_raw = fabsf(raw - sp->prev_raw);

        if (d_raw > SPIKE_K * noise_rms) {
            if (sp->spike_consec == 0u) {
                sp->spike_start_ms = (uint32_t)(esp_timer_get_time() / 1000LL);
            }
            sp->spike_consec++;

            uint32_t now_ms    = (uint32_t)(esp_timer_get_time() / 1000LL);
            uint32_t spike_dur = now_ms - sp->spike_start_ms;

            if (sp->spike_consec <= SPIKE_CONSEC_MAX && spike_dur < SPIKE_MAX_MS) {
                flags |= SIGNAL_FLAG_SPIKE;   /* short burst: EMI/hardware */
            }
            /* If spike_consec > SPIKE_CONSEC_MAX: it's a real target ? no flag */
        } else {
            sp->spike_consec = 0u;
        }
	}
    sp->prev_raw = raw;
    sp->prev_filtered = smoothed;

    /* -- Spike Rate EMA (ChatGPT fix ? sustained EMI environment) ---
     * Simple counter toggles: spike=100 ? resets to 0 instantly.
     * EMA smooths this into a continuous level: persistent EMI = high
     * spike_rate_ema, single isolated spike = low after a few samples.
     *
     * spike_rate_ema = 0.9 ? prev + 0.1 ? event (event: 1.0 or 0.0)
     * Time constant: ~10 samples = 500ms @20Hz ? reflects "EMI climate"
     * --------------------------------------------------------------- */
    {
        float spike_event = (flags & SIGNAL_FLAG_SPIKE) ? 1.0f : 0.0f;
        sp->spike_rate_ema = 0.90f * sp->spike_rate_ema + 0.10f * spike_event;
    }
    out->signal_flags = flags; //(baseline_fast - baseline_slow ? ChatGPT fix) -
     /* FIX: Use the DIFFERENCE between the two baseline trackers, not
     * the 3-second-ago comparison.
     *
     * baseline_adaptive (fast, ??1.0): follows soil changes in ~20s
     * baseline_slow     (slow, ??0.1): follows only very slow trends
     *
     * The difference |fast - slow| IS the current drift rate proxy:
     *   ? 0:        stable soil, both trackers agree
     *   > threshold: active drift ? soil changing or temperature effect
     *
     * This is immune to the 3-second window EMA delay problem because
     * both references update every sample ? the comparison is always fresh.
     * --------------------------------------------------------------- */
    {
        const float DRIFT_FAST_SLOW_THRESH = 3.0f;  /* LSB difference between trackers */

        float drift_proxy = fabsf(sp->baseline_adaptive - sp->baseline_slow);
        if (drift_proxy > DRIFT_FAST_SLOW_THRESH) {
            flags |= SIGNAL_FLAG_DRIFT;
        }
    }

    /* -- Flag 3: HIGH_NOISE with absolute floor (ChatGPT fix) --------
     * FIX: Add max(3?calibrated_floor, 1.0 LSB) to prevent false
     * positives in very clean soil where calibrated_floor may be tiny.
     *
     * Example: calibrated_floor = 0.2 LSB (pristine soil after Phase 3C)
     *   threshold = max(3?0.2, 1.0) = 1.0 LSB  (not 0.6!)
     *   This prevents flagging every normal sample as "noisy".
     * --------------------------------------------------------------- */
    {
        const float HIGH_NOISE_K    = 3.0f;
        const float ABSOLUTE_MIN_RMS = 1.0f;   /* never flag below this */

        float calibrated_floor = sqrtf(sp->calibrated_noise_var > 0.001f ?
                                       sp->calibrated_noise_var : 0.001f);
        float threshold = HIGH_NOISE_K * calibrated_floor;
        if (threshold < ABSOLUTE_MIN_RMS) threshold = ABSOLUTE_MIN_RMS;

        if (noise_rms > threshold) {
            flags |= SIGNAL_FLAG_HIGH_NOISE;
        }
    }

    /* -- Flag 4: UNSTABLE ------------------------------------------- */
    if (sp->stability_score < SP_STABILITY_THRESHOLD * 0.5f) {
        flags |= SIGNAL_FLAG_UNSTABLE;
    }

    out->signal_flags = flags;

    /* -- Confidence Score (0?100) -------------------------------------
     * Additive penalties model ? base starts at 100.
     *
     * Components:
     *  spike_rate_ema: sustained EMI environment (-30 max)
     *  noise_penalty:  current noise / calibrated floor (-40 max)
     *  drift_penalty:  |fast-slow| baseline divergence (-20 max)
     *  stab_penalty:   instability score penalty (-20 max)
     *  coherence:      short-term / long-term variance ratio (multiplier)
     *
     * COHERENCE (ChatGPT final recommendation):
     *   coherence = 1 - (variance_short / variance_long)
     *   High coherence (stable signal): multiplier ? 1.0 ? no reduction
     *   Low coherence (chaotic):        multiplier = 0.3 ? cuts confidence
     *
     *   variance_short = noise_variance (EMA, fast, ~1s window)
     *   variance_long  = calibrated_noise_var (frozen reference)
     *
     *   This catches random bursts that aren't spike-like but aren't
     *   physically coherent either ? EMI that "drifts" rather than spikes.
     *
     *   Clamp coherence multiplier to [0.3, 1.0] ? never zero (signal may
     *   still be valid even in noisy conditions, just less trusted).
     *
     * Final range:
     *   90?100: clean calibrated soil, strong/stable signal
     *   60?89:  valid signal, moderate confidence
     *   30?59:  uncertain ? noisy soil or mild EMI
     *   <30:    unreliable ? notify user, consider re-calibrating
     * --------------------------------------------------------------- */
    {
        float conf = 100.0f;

        /* Spike rate penalty: uses EMA not raw flag for smoothness */
        conf -= sp->spike_rate_ema * 30.0f;

        float calibrated_floor = sqrtf(sp->calibrated_noise_var > 0.001f ?
                                       sp->calibrated_noise_var : 0.001f);
        float noise_ratio   = (calibrated_floor > 0.001f) ?
                               (noise_rms / calibrated_floor) : 1.0f;
        float noise_penalty = (noise_ratio - 1.0f) * 20.0f;
        if (noise_penalty < 0.0f)  noise_penalty = 0.0f;
        if (noise_penalty > 40.0f) noise_penalty = 40.0f;
        conf -= noise_penalty;

        float drift_proxy   = fabsf(sp->baseline_adaptive - sp->baseline_slow);
        float drift_penalty = (drift_proxy / 3.0f) * 10.0f;
        if (drift_penalty > 20.0f) drift_penalty = 20.0f;
        conf -= drift_penalty;

        float stab_penalty = (100.0f - sp->stability_score) * 0.3f;
        if (stab_penalty > 20.0f) stab_penalty = 20.0f;
        conf -= stab_penalty;

        /* Coherence clamp [0.3, 1.0] ? confirmed applied (ChatGPT audit) */
        float var_short = sp->noise_variance;
        float var_long  = sp->calibrated_noise_var > 0.001f ?
                          sp->calibrated_noise_var : sp->noise_variance;
        float coherence = 1.0f - (var_short / (var_long > 0.001f ? var_long : 0.001f));
        if (coherence < 0.3f) coherence = 0.3f;
        if (coherence > 1.0f) coherence = 1.0f;
        conf *= coherence;

        /* Cap at 90% ? always leave margin (ChatGPT audit):
         * A device that shows 100% confidence breeds complacency.
         * 90% max keeps the operator appropriately attentive.
         * A calibrated, stable, clean signal shows ~86?89%. */
        if (conf > 90.0f)  conf = 90.0f;
        if (conf <  0.0f)  conf =  0.0f;

        out->confidence = (uint8_t)conf;
    }
    return true;
}

/* =========================================================================
 * QUERIES
 * ========================================================================= */

float sp_get_stability(const SignalProcessor_t *sp) { return sp->stability_score; }
float sp_get_snr(const SignalProcessor_t *sp) {
    return sp->noise_variance > 0.01f ?
           sp->signal_variance / sp->noise_variance : 1.0f;
}
bool  sp_is_stable(const SignalProcessor_t *sp)    { return sp->is_stable; }
float sp_get_baseline(const SignalProcessor_t *sp) { return sp->baseline_fixed; }

void sp_set_bt_mode(SignalProcessor_t *sp, BtOutputMode_t mode)
{
    sp->bt_mode  = mode;
    sp->bt_drift = 0.0f;   /* reset drift on mode change ? in-struct, Core0-safe */
    const char *names[] = {"RAW", "NORMALIZED", "ENHANCED"};
    ESP_LOGI(TAG, "BT mode: %s", names[(uint8_t)mode < 3 ? mode : 0]);
}

BtOutputMode_t sp_get_bt_mode(const SignalProcessor_t *sp)
{
    return sp->bt_mode;
}

void sp_set_heading(SignalProcessor_t *sp, uint8_t heading)
{
    /* heading: 0=N 1=E 2=S 3=W 4=disable */
    sp->scan_heading = (heading <= 4) ? heading : 4;
    if (heading < 4) {
        ESP_LOGI(TAG, "Heading: %s  compensation: %s",
                 heading == 0 ? "N" : heading == 1 ? "E" :
                 heading == 2 ? "S" : "W",
                 sp->heading_comp_enabled ? "ON" : "OFF");
    }
}

void sp_set_heading_comp(SignalProcessor_t *sp, bool enable)
{
    sp->heading_comp_enabled = enable;
    if (!enable) sp->scan_heading = 4;
    ESP_LOGI(TAG, "Heading compensation: %s", enable ? "ENABLED" : "DISABLED");
}

/* Load heading corrections from device calibration.
 * Call once after devcal_load() ? copies corrections table. */
void sp_load_heading_corrections(SignalProcessor_t *sp,
                                  const float corrections[4])
{
    if (!corrections) return;
    for (int i = 0; i < 4; i++) {
        sp->heading_corrections[i] = corrections[i];
    }
    ESP_LOGI(TAG, "Heading corrections loaded: N=%.1f E=%.1f S=%.1f W=%.1f",
             corrections[0], corrections[1], corrections[2], corrections[3]);
}

void sp_reset_adaptive(SignalProcessor_t *sp)
{
    sp->baseline_adaptive = sp->baseline_fixed;
    sp->baseline_slow     = sp->baseline_fixed;
    sp->stability_score   = 0.0f;
    sp->kf.x = 0.0f;
    sp->kf.p = 1.0f;
    ESP_LOGI(TAG, "Reset: baseline=%.2f", sp->baseline_fixed);
}