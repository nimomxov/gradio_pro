/**
 * @file calib_engine.c
 * @brief Two-phase soil calibration engine implementation.
 */

#include "calib_engine.h"
#include "system_monitor/system_monitor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

static const char *TAG = "CalibEng";

/* =========================================================================
 * SOIL CLASSIFICATION TABLE
 *
 * Maps std_dev ranges to:
 *  - SoilType
 *  - Recommended sensitivity
 *  - Filter window size
 *  - EMA alpha
 *  - Outlier rejection sigma
 *
 * Values derived from field gradiometer literature and FLC100 datasheet:
 *  Clean soil:     std < 2     → very little mineralization
 *  Mineral soil:   std 2-8     → common agricultural / field condition
 *  Noisy soil:     std 8-25    → high iron content, volcanic soil
 *  Extreme:        std > 25    → industrial area, power lines, very wet clay
 * ========================================================================= */

/* =========================================================================
 * SOIL CLASSIFICATION TABLE — 5 مستويات
 *
 * كل صف يحدد:
 *  std_max      : الحد الأعلى لـ std_dev لهذا التصنيف
 *  soil_type    : نوع التربة المُكتشَف
 *  recommended  : مستوى الحساسية الموصى به تلقائياً
 *  window_size  : حجم نافذة المتوسط المتحرك (عينات)
 *  alpha_smooth : معامل EMA [0.0-1.0] — كلما ارتفع كلما كان أسرع استجابةً
 *  outlier_sigma: حد رفض الشواذ بعدد الانحرافات المعيارية
 *
 * SCIENTIFIC BASIS (FLC100 + ADS1115 @ ±2.048V, 16-bit):
 *  1 ADC count ≈ 62.5 µV → مع حساسية FLC100 ~10 V/T → 1 count ≈ 6.25 µT
 *  Pristine:  std < 1.0  → noise floor < 6.25 µT  → يكشف أهداف > 2m
 *  Clean:     std 1-4    → noise floor < 25 µT     → يكشف أهداف > 1.5m
 *  Mineral:   std 4-12   → noise floor < 75 µT     → يكشف أهداف > 1m
 *  Noisy:     std 12-30  → noise floor < 190 µT    → يكشف أهداف > 0.6m
 *  Extreme:   std > 30   → noise floor > 190 µT    → أهداف سطحية فقط
 * ========================================================================= */


/* =========================================================================
 * SOIL CLASSIFICATION TABLE
 * ========================================================================= */

typedef struct {
    float             std_max;
    SoilType_t        soil_type;
    SensitivityMode_t recommended;
    uint8_t           window_size;
    float             alpha_smooth;
    float             outlier_sigma;
} SoilClass_t;

static const SoilClass_t SOIL_TABLE[] = {
    {   1.0f, SOIL_TYPE_PRISTINE, SENS_MODE_VERY_HIGH,  4, 0.60f, 1.8f },
    {   4.0f, SOIL_TYPE_CLEAN,    SENS_MODE_HIGH,        8, 0.45f, 2.0f },
    {  12.0f, SOIL_TYPE_MINERAL,  SENS_MODE_MEDIUM,     20, 0.28f, 2.5f },
    {  30.0f, SOIL_TYPE_NOISY,    SENS_MODE_LOW,        36, 0.15f, 3.0f },
    { 999.0f, SOIL_TYPE_EXTREME,  SENS_MODE_VERY_LOW,   56, 0.08f, 3.8f },
};
#define SOIL_TABLE_COUNT (sizeof(SOIL_TABLE) / sizeof(SOIL_TABLE[0]))

/* =========================================================================
 * LIFECYCLE
 * ========================================================================= */

void calib_engine_init(CalibEngine_t *eng)
{
    memset(eng, 0, sizeof(CalibEngine_t));
    eng->status        = CALIB_STATUS_IDLE;
    eng->current_phase = CALIB_PHASE_STATIC;
    ESP_LOGI(TAG, "Calib engine init. Static=%us  Dynamic=%us  Total=%us",
             CALIB_STATIC_MS/1000, CALIB_DYNAMIC_MS/1000, CALIB_DURATION_MS/1000);
}

void calib_engine_start(CalibEngine_t *eng)
{
    memset(eng->ring,          0, sizeof(eng->ring));
    memset(eng->dyn_local_buf, 0, sizeof(eng->dyn_local_buf));
    eng->ring_head        = 0;
    eng->ring_count       = 0;
    eng->welford_mean     = 0.0;
    eng->welford_M2       = 0.0;
    eng->welford_n        = 0;
    eng->dyn_local_head   = 0;
    eng->dyn_local_count  = 0;
    eng->dyn_welford_mean = 0.0;
    eng->dyn_welford_M2   = 0.0;
    eng->dyn_welford_n    = 0;
    eng->dyn_outliers     = 0;
    eng->total_fed        = 0;
    eng->rejected_count   = 0;
    eng->noise_static     = 0.0f;
    eng->noise_dynamic    = 0.0f;
    eng->current_phase    = CALIB_PHASE_STATIC;
    eng->phase2_notified  = false;
    eng->start_tick       = xTaskGetTickCount();
    eng->status           = CALIB_STATUS_IN_PROGRESS;
    eng->started          = true;
    memset(&eng->result, 0, sizeof(CalibResult_t));

    ESP_LOGI(TAG, "Calibration STARTED");
    ESP_LOGI(TAG, "  Phase 1 (still):   0-10s");
    ESP_LOGI(TAG, "  Phase 2 (walking): 10-30s");
}

/* =========================================================================
 * WELFORD'S ONLINE ALGORITHM
 * ========================================================================= */

static void welford_update(double *mean, double *M2, uint32_t *n, double value)
{
    (*n)++;
    double delta  = value - *mean;
    *mean += delta / (double)(*n);
    double delta2 = value - *mean;
    *M2   += delta * delta2;
}

static double welford_variance(double M2, uint32_t n)
{
    return (n < 2) ? 0.0 : M2 / (double)n;
}

/* =========================================================================
 * OUTLIER REMOVAL
 * ========================================================================= */

static uint16_t remove_outliers(int16_t *samples, uint16_t n,
                                 float mean, float std_dev,
                                 float *out_mean, float *out_std)
{
    if (std_dev < 0.01f) { *out_mean = mean; *out_std = std_dev; return 0; }
    if (n == 0)          { *out_mean = mean; *out_std = std_dev; return 0; }

    /* FIX: Use a separate valid[] boolean array instead of INT16_MIN sentinel.
     * The sentinel approach is broken because ADS1115 CAN legitimately return
     * -32768 (INT16_MIN) when the input is at full negative swing (e.g., strong
     * ferrous target very close to sensor). Marking that as an outlier silently
     * corrupts calibration — it would make the system think there's no signal
     * in saturation conditions, the exact moment we need accurate noise floor.
     * A separate bool array costs N bytes on the stack but is always correct. */

    /* Stack-allocate for CALIB_RING_SIZE (512 bytes max) — safe on signal_task 6KB stack */
    bool valid[GRAD_CALIB_RING_SIZE];
    if (n > GRAD_CALIB_RING_SIZE) n = GRAD_CALIB_RING_SIZE;  /* safety clamp */

    float threshold = CALIB_OUTLIER_SIGMA * std_dev;
    uint16_t removed = 0;

    for (uint16_t i = 0; i < n; i++) {
        valid[i] = (fabsf((float)samples[i] - mean) <= threshold);
        if (!valid[i]) removed++;
    }

    double sum = 0.0, sum2 = 0.0;
    uint16_t v_count = 0;
    for (uint16_t i = 0; i < n; i++) {
        if (valid[i]) {
            sum  += (double)samples[i];
            sum2 += (double)samples[i] * (double)samples[i];
            v_count++;
        }
    }

    if (v_count == 0) { *out_mean = mean; *out_std = std_dev; return removed; }

    *out_mean = (float)(sum / (double)v_count);
    float var = (float)(sum2 / (double)v_count - (double)(*out_mean) * (double)(*out_mean));
    *out_std  = sqrtf(var > 0.0f ? var : 0.0f);
    return removed;
}

/* =========================================================================
 * DRIFT ESTIMATION (linear regression)
 * ========================================================================= */

static float estimate_drift(const int16_t *samples, uint16_t n)
{
    if (n < 2) return 0.0f;
    double sum_x = 0, sum_y = 0;
    for (uint16_t i = 0; i < n; i++) { sum_x += i; sum_y += samples[i]; }
    double mx = sum_x / n, my = sum_y / n;
    double num = 0, den = 0;
    for (uint16_t i = 0; i < n; i++) {
        double dx = i - mx, dy = samples[i] - my;
        num += dx * dy; den += dx * dx;
    }
    return (den < 0.001) ? 0.0f : (float)(num / den);
}

/* =========================================================================
 * PHASE 1 ANALYSIS
 * ========================================================================= */

static float analyze_static(CalibEngine_t *eng, float *out_mean)
{
    float raw_mean = (float)eng->welford_mean;
    float raw_std  = sqrtf((float)welford_variance(eng->welford_M2, eng->welford_n));

    static int16_t work[CALIB_RING_SIZE];
    uint16_t n = eng->ring_count;
    memcpy(work, eng->ring, n * sizeof(int16_t));

    float clean_mean, clean_std;
    uint16_t removed = remove_outliers(work, n, raw_mean, raw_std,
                                       &clean_mean, &clean_std);

    ESP_LOGI(TAG, "P1 static: mean=%.2f  std=%.3f  removed=%u", clean_mean, clean_std, removed);
    *out_mean = clean_mean;
    return clean_std;
}

/* =========================================================================
 * PHASE 2 FEED — Dynamic noise estimation
 *
 * Algorithm:
 *  1. Maintain a local circular buffer of CALIB_DYN_LOCAL_WIN samples
 *  2. local_mean = average of last N samples  (tracks spatial drift)
 *  3. residual = sample - local_mean          (removes spatial variation)
 *  4. Feed residual into Welford               (measures true noise)
 *  5. Outlier rejection: |residual| > 3σ      (excludes real targets)
 * ========================================================================= */

static void feed_dynamic(CalibEngine_t *eng, int16_t value)
{
    /* Update local circular buffer */
    eng->dyn_local_buf[eng->dyn_local_head] = (float)value;
    eng->dyn_local_head = (eng->dyn_local_head + 1) % CALIB_DYN_LOCAL_WIN;
    if (eng->dyn_local_count < CALIB_DYN_LOCAL_WIN) eng->dyn_local_count++;

    /* Compute local mean */
    float local_sum = 0;
    for (uint8_t i = 0; i < eng->dyn_local_count; i++)
        local_sum += eng->dyn_local_buf[i];
    float local_mean = local_sum / (float)eng->dyn_local_count;

    /* Residual = sample - local mean */
    float residual = (float)value - local_mean;

    /* Outlier rejection: skip if residual > 3σ (likely a real target) */
    if (eng->dyn_welford_n >= 8) {
        float dyn_std = sqrtf((float)welford_variance(eng->dyn_welford_M2,
                                                       eng->dyn_welford_n));
        if (dyn_std > 0.01f && fabsf(residual) > CALIB_OUTLIER_SIGMA * dyn_std) {
            eng->dyn_outliers++;
            return;  /* skip — likely a real target or spike */
        }
    }

    /* Feed residual into Welford */
    welford_update(&eng->dyn_welford_mean, &eng->dyn_welford_M2,
                   &eng->dyn_welford_n, (double)residual);
}

/* =========================================================================
 * FINAL ANALYSIS (after both phases)
 * ========================================================================= */

static CalibEngineStatus_t run_final_analysis(CalibEngine_t *eng)
{
    eng->status = CALIB_STATUS_COMPUTING;

    /* ── Phase 1 results ── */
    float baseline;
    float noise_s = analyze_static(eng, &baseline);
    eng->noise_static = noise_s;

    /* ── Phase 2 results ── */
    float noise_d = 0.0f;
    if (eng->dyn_welford_n >= 8) {
        noise_d = sqrtf((float)welford_variance(eng->dyn_welford_M2,
                                                 eng->dyn_welford_n));
    }
    eng->noise_dynamic = noise_d;

    ESP_LOGI(TAG, "P2 dynamic: noise=%.3f  outliers_rejected=%lu",
             noise_d, (unsigned long)eng->dyn_outliers);

    /* ── Combine ── */
    float noise_final = (noise_d > noise_s) ? noise_d : noise_s;

    ESP_LOGI(TAG, "Final: baseline=%.2f  noise_static=%.3f  noise_dynamic=%.3f  → noise_final=%.3f",
             baseline, noise_s, noise_d, noise_final);

    /* ── Soil classification uses noise_final ── */
    SoilType_t        soil    = calib_classify_soil(noise_final);
    SensitivityMode_t recommend = calib_recommend_sensitivity(soil);

    CalibResult_t r;
    memset(&r, 0, sizeof(r));
    calib_compute_filter_params(soil, recommend, &r);

    r.mean              = baseline;
    r.std_dev           = noise_final;
    r.variance          = noise_final * noise_final;
    r.soil_type         = soil;
    r.recommended_mode  = recommend;
    r.samples_collected = (uint16_t)(eng->welford_n + eng->dyn_welford_n);
    r.timestamp_ms      = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    r.is_valid          = true;

    /* Drift from Phase 1 */
    static int16_t work[CALIB_RING_SIZE];
    memcpy(work, eng->ring, eng->ring_count * sizeof(int16_t));
    float drift_ps = estimate_drift(work, eng->ring_count)
                     * (float)GRAD_ADC_SAMPLE_RATE_HZ;
    r.drift_rate = drift_ps;

    if (fabsf(drift_ps) > CALIB_MAX_DRIFT_RATE) {
        ESP_LOGW(TAG, "Drift %.3f counts/s — device not fully still in Phase 1", drift_ps);
    }

    const char *soil_names[] = {"UNKNOWN","PRISTINE","CLEAN","MINERAL","NOISY","EXTREME"};
    ESP_LOGI(TAG, "=== CALIBRATION COMPLETE ===");
    ESP_LOGI(TAG, "  Baseline:    %.2f", r.mean);
    ESP_LOGI(TAG, "  Noise:       %.3f (static=%.3f  dynamic=%.3f)", noise_final, noise_s, noise_d);
    ESP_LOGI(TAG, "  Soil:        %s", soil_names[(int)soil]);
    ESP_LOGI(TAG, "  Window:      %d  Alpha:%.2f  Sigma:%.1f",
             r.window_size, r.alpha_smooth, r.outlier_sigma);

    eng->result = r;
    eng->status = CALIB_STATUS_DONE;
    return CALIB_STATUS_DONE;
}

/* =========================================================================
 * FEED
 * ========================================================================= */

CalibEngineStatus_t calib_engine_feed(CalibEngine_t *eng, const AdcSample_t *sample)
{
    if (eng->status != CALIB_STATUS_IN_PROGRESS) return eng->status;

    eng->total_fed++;

    if (sample->quality < CALIB_MIN_SAMPLE_QUALITY) {
        eng->rejected_count++;
        return CALIB_STATUS_IN_PROGRESS;
    }

    uint32_t elapsed_ms = (xTaskGetTickCount() - eng->start_tick) * portTICK_PERIOD_MS;
    int16_t  value      = sample->differential;

    if (elapsed_ms < CALIB_STATIC_MS) {
        /* ── Phase 1: Static ── */
        eng->current_phase = CALIB_PHASE_STATIC;

        if (eng->ring_count < CALIB_RING_SIZE) {
            eng->ring[eng->ring_head] = value;
            eng->ring_head = (eng->ring_head + 1) % CALIB_RING_SIZE;
            eng->ring_count++;
        }
        welford_update(&eng->welford_mean, &eng->welford_M2, &eng->welford_n, (double)value);

    } else if (elapsed_ms < CALIB_DURATION_MS) {
        /* ── Phase 2: Dynamic ── */
        if (eng->current_phase != CALIB_PHASE_DYNAMIC) {
            eng->current_phase = CALIB_PHASE_DYNAMIC;
            ESP_LOGI(TAG, "→ Phase 2 (dynamic) started at %lu ms", (unsigned long)elapsed_ms);
        }
        feed_dynamic(eng, value);

    } else {
        /* ── Done ── */
        return run_final_analysis(eng);
    }

    return CALIB_STATUS_IN_PROGRESS;
}

/* =========================================================================
 * QUERIES
 * ========================================================================= */

uint8_t calib_engine_get_progress(const CalibEngine_t *eng)
{
    if (!eng->started)                           return 0;
    if (eng->status == CALIB_STATUS_DONE)        return 100;
    if (eng->status == CALIB_STATUS_ERROR)       return 100;

    uint32_t elapsed = (xTaskGetTickCount() - eng->start_tick) * portTICK_PERIOD_MS;
    if (elapsed >= CALIB_DURATION_MS) return 99;
    return (uint8_t)((elapsed * 100u) / CALIB_DURATION_MS);
}

bool calib_engine_phase2_started(CalibEngine_t *eng)
{
    if (eng->current_phase == CALIB_PHASE_DYNAMIC && !eng->phase2_notified) {
        eng->phase2_notified = true;
        return true;
    }
    return false;
}

CalibPhaseInternal_t calib_engine_get_phase(const CalibEngine_t *eng)
{
    return eng->current_phase;
}

esp_err_t calib_engine_get_result(const CalibEngine_t *eng, CalibResult_t *out_result)
{
    if (eng->status != CALIB_STATUS_DONE) return ESP_ERR_INVALID_STATE;
    *out_result = eng->result;
    return ESP_OK;
}

CalibEngineStatus_t calib_engine_get_status(const CalibEngine_t *eng)
{
    return eng->status;
}

/* =========================================================================
 * SOIL CLASSIFICATION
 * ========================================================================= */

SoilType_t calib_classify_soil(float std_dev)
{
    for (size_t i = 0; i < SOIL_TABLE_COUNT; i++) {
        if (std_dev < SOIL_TABLE[i].std_max) return SOIL_TABLE[i].soil_type;
    }
    return SOIL_TYPE_EXTREME;
}

SensitivityMode_t calib_recommend_sensitivity(SoilType_t soil)
{
    for (size_t i = 0; i < SOIL_TABLE_COUNT; i++) {
        if (SOIL_TABLE[i].soil_type == soil) return SOIL_TABLE[i].recommended;
    }
    return SENS_MODE_MEDIUM;
}

void calib_compute_filter_params(SoilType_t soil, SensitivityMode_t mode,
                                  CalibResult_t *out_result)
{
    uint8_t window = 16;
    float   alpha  = 0.30f;
    float   sigma  = 2.5f;

    for (size_t i = 0; i < SOIL_TABLE_COUNT; i++) {
        if (SOIL_TABLE[i].soil_type == soil) {
            window = SOIL_TABLE[i].window_size;
            alpha  = SOIL_TABLE[i].alpha_smooth;
            sigma  = SOIL_TABLE[i].outlier_sigma;
            break;
        }
    }

    switch (mode) {
        case SENS_MODE_VERY_HIGH:
            window = 4;   alpha = fminf(alpha + 0.15f, 0.60f); sigma = fmaxf(sigma - 0.5f, 1.5f); break;
        case SENS_MODE_HIGH:
            window = (window > 4) ? window - 4 : 4;
            alpha  = fminf(alpha + 0.08f, 0.50f); sigma = fmaxf(sigma - 0.3f, 1.8f); break;
        case SENS_MODE_LOW:
            window = (uint8_t)((window + 8 <= GRAD_FILTER_MAX_WINDOW) ? window + 8 : GRAD_FILTER_MAX_WINDOW);
            alpha  = fmaxf(alpha - 0.08f, 0.08f); sigma = fminf(sigma + 0.5f, 4.5f); break;
        case SENS_MODE_VERY_LOW:
            window = (uint8_t)((window + 16 <= GRAD_FILTER_MAX_WINDOW) ? window + 16 : GRAD_FILTER_MAX_WINDOW);
            alpha  = fmaxf(alpha - 0.15f, 0.03f); sigma = fminf(sigma + 0.8f, 5.0f); break;
        default: break;
    }

    out_result->window_size   = window;
    out_result->alpha_smooth  = alpha;
    out_result->outlier_sigma = sigma;
}
