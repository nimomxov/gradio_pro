/**
 * @file calib_engine.c
 * @brief 10-second calibration engine — implementation.
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

typedef struct {
    float             std_max;
    SoilType_t        soil_type;
    SensitivityMode_t recommended;
    uint8_t           window_size;   /* moving average window */
    float             alpha_smooth;  /* EMA alpha */
    float             outlier_sigma; /* rejection threshold */
} SoilClass_t;

static const SoilClass_t SOIL_TABLE[] = {
    /*  std_max   soil_type              recommended          window  alpha   sigma */
    {   1.0f,  SOIL_TYPE_PRISTINE,  SENS_MODE_VERY_HIGH,      4,   0.60f,  1.8f },
    {   4.0f,  SOIL_TYPE_CLEAN,     SENS_MODE_HIGH,            8,   0.45f,  2.0f },
    {  12.0f,  SOIL_TYPE_MINERAL,   SENS_MODE_MEDIUM,         20,   0.28f,  2.5f },
    {  30.0f,  SOIL_TYPE_NOISY,     SENS_MODE_LOW,            36,   0.15f,  3.0f },
    { 999.0f,  SOIL_TYPE_EXTREME,   SENS_MODE_VERY_LOW,       56,   0.08f,  3.8f },
};

#define SOIL_TABLE_COUNT (sizeof(SOIL_TABLE) / sizeof(SOIL_TABLE[0]))

/* =========================================================================
 * LIFECYCLE
 * ========================================================================= */

void calib_engine_init(CalibEngine_t *eng)
{
    memset(eng, 0, sizeof(CalibEngine_t));
    eng->status  = CALIB_STATUS_IDLE;
    eng->started = false;
    ESP_LOGI(TAG, "Calibration engine initialized. Ring size: %u samples",
             CALIB_RING_SIZE);
}

void calib_engine_start(CalibEngine_t *eng)
{
    memset(eng->ring, 0, sizeof(eng->ring));
    eng->ring_head      = 0;
    eng->ring_count     = 0;
    eng->welford_mean   = 0.0;
    eng->welford_M2     = 0.0;
    eng->welford_n      = 0;
    eng->total_fed      = 0;
    eng->rejected_count = 0;
    eng->start_tick     = xTaskGetTickCount();
    eng->status         = CALIB_STATUS_IN_PROGRESS;
    eng->started        = true;
    memset(&eng->result, 0, sizeof(CalibResult_t));

    ESP_LOGI(TAG, "Calibration STARTED — collecting for %u ms (%u s)",
             CALIB_DURATION_MS, CALIB_DURATION_MS / 1000u);
    ESP_LOGI(TAG, "Expected samples: ~%u at %uHz",
             (CALIB_DURATION_MS / 1000u) * GRAD_ADC_SAMPLE_RATE_HZ,
             GRAD_ADC_SAMPLE_RATE_HZ);
}

/* =========================================================================
 * WELFORD'S ONLINE ALGORITHM
 *
 * Computes mean and variance in a single pass, numerically stable.
 * Reference: Welford (1962), Knuth TAOCP Vol 2.
 *
 * Classic formula: variance = Σ(x - mean)² / n
 * Problem: requires two passes OR accumulates catastrophic cancellation.
 *
 * Welford's update:
 *   n    += 1
 *   delta = x - mean
 *   mean += delta / n
 *   M2   += delta * (x - mean)   ← uses NEW mean
 *   variance = M2 / n
 * ========================================================================= */

static void welford_update(CalibEngine_t *eng, int16_t value)
{
    eng->welford_n++;
    double delta  = (double)value - eng->welford_mean;
    eng->welford_mean += delta / (double)eng->welford_n;
    double delta2 = (double)value - eng->welford_mean;
    eng->welford_M2 += delta * delta2;
}

static double welford_get_variance(const CalibEngine_t *eng)
{
    if (eng->welford_n < 2) return 0.0;
    return eng->welford_M2 / (double)eng->welford_n;
}

/* =========================================================================
 * DRIFT ESTIMATION (linear regression)
 *
 * Estimates how much the baseline is drifting over the calibration window.
 * If drift is significant, the calibration may be unreliable.
 *
 * Uses simple OLS: fits y = a + b*x where x = sample index, y = sample value.
 * Returns b (slope) in ADC counts per sample.
 * ========================================================================= */

static float estimate_drift(const int16_t *samples, uint16_t n)
{
    if (n < 2) return 0.0f;

    /* Compute means */
    double sum_x = 0.0, sum_y = 0.0;
    for (uint16_t i = 0; i < n; i++) {
        sum_x += i;
        sum_y += samples[i];
    }
    double mean_x = sum_x / n;
    double mean_y = sum_y / n;

    /* Compute slope: b = Σ(xi - mean_x)(yi - mean_y) / Σ(xi - mean_x)² */
    double num = 0.0, den = 0.0;
    for (uint16_t i = 0; i < n; i++) {
        double dx = (double)i - mean_x;
        double dy = (double)samples[i] - mean_y;
        num += dx * dy;
        den += dx * dx;
    }

    if (den < 0.001) return 0.0f;
    return (float)(num / den);  /* ADC counts per sample */
}

/* =========================================================================
 * OUTLIER REMOVAL (Chauvenet criterion)
 *
 * Removes samples that are statistically unlikely to belong to the
 * main distribution. This handles:
 *  - Operator accidentally sweeping over a target during calibration
 *  - Electromagnetic interference spikes
 *  - Hardware glitch samples that passed the quality filter
 *
 * Algorithm:
 *  1. Compute mean and std from ALL collected samples (Welford result)
 *  2. Remove any sample where |x - mean| > CALIB_OUTLIER_SIGMA × std
 *  3. Recompute mean from remaining samples
 *
 * Returns number of outliers removed.
 * ========================================================================= */

static uint16_t remove_outliers(int16_t *samples, uint16_t n,
                                float mean, float std_dev,
                                float *out_clean_mean, float *out_clean_std)
{
    if (std_dev < 0.01f) {
        *out_clean_mean = mean;
        *out_clean_std  = std_dev;
        return 0;
    }

    float threshold = CALIB_OUTLIER_SIGMA * std_dev;
    uint16_t removed = 0;

    /* Mark outliers by setting to INT16_MIN (sentinel) */
    for (uint16_t i = 0; i < n; i++) {
        if (fabsf((float)samples[i] - mean) > threshold) {
            samples[i] = INT16_MIN;
            removed++;
        }
    }

    /* Recompute mean and variance from clean samples */
    double sum = 0.0, sum_sq = 0.0;
    uint16_t valid = 0;
    for (uint16_t i = 0; i < n; i++) {
        if (samples[i] != INT16_MIN) {
            sum += samples[i];
            valid++;
        }
    }
    if (valid == 0) {
        *out_clean_mean = mean;
        *out_clean_std  = std_dev;
        return removed;
    }

    double clean_mean = sum / valid;
    for (uint16_t i = 0; i < n; i++) {
        if (samples[i] != INT16_MIN) {
            double d = samples[i] - clean_mean;
            sum_sq += d * d;
        }
    }

    *out_clean_mean = (float)clean_mean;
    *out_clean_std  = (valid > 1) ? sqrtf((float)(sum_sq / valid)) : std_dev;

    return removed;
}

/* =========================================================================
 * SOIL CLASSIFICATION & SENSITIVITY RECOMMENDATION
 * ========================================================================= */

SoilType_t calib_classify_soil(float std_dev)
{
    for (size_t i = 0; i < SOIL_TABLE_COUNT; i++) {
        if (std_dev <= SOIL_TABLE[i].std_max) {
            return SOIL_TABLE[i].soil_type;
        }
    }
    return SOIL_TYPE_EXTREME;
}

SensitivityMode_t calib_recommend_sensitivity(SoilType_t soil)
{
    for (size_t i = 0; i < SOIL_TABLE_COUNT; i++) {
        if (SOIL_TABLE[i].soil_type == soil) {
            return SOIL_TABLE[i].recommended;
        }
    }
    return SENS_MODE_MEDIUM;
}

void calib_compute_filter_params(SoilType_t soil, SensitivityMode_t mode,
                                 CalibResult_t *out_result)
{
    /*
     * نبدأ من معاملات التربة الافتراضية من SOIL_TABLE،
     * ثم نُطبِّق تعديل المستخدم (إذا اختار غير AUTO).
     *
     * جدول التعديلات النسبية للمستخدم مقارنةً بالتوصية الافتراضية:
     *
     *  VERY_HIGH: window-=6, alpha+=0.15, sigma-=0.8  ← أقصى حساسية
     *  HIGH:      window-=3, alpha+=0.08, sigma-=0.4
     *  MEDIUM:    بدون تغيير — نفس قيم التربة
     *  LOW:       window+=8, alpha-=0.08, sigma+=0.5
     *  VERY_LOW:  window+=16, alpha-=0.15, sigma+=0.8 ← أقصى تصفية
     *  AUTO:      نفس التوصية (MEDIUM of soil)
     */

    /* قيم افتراضية من SOIL_TABLE */
    uint8_t window = 20;
    float   alpha  = 0.28f;
    float   sigma  = 2.5f;

    for (size_t i = 0; i < SOIL_TABLE_COUNT; i++) {
        if (SOIL_TABLE[i].soil_type == soil) {
            window = SOIL_TABLE[i].window_size;
            alpha  = SOIL_TABLE[i].alpha_smooth;
            sigma  = SOIL_TABLE[i].outlier_sigma;
            break;
        }
    }

    /* تطبيق تعديل مستوى الحساسية */
    switch (mode) {

        case SENS_MODE_VERY_HIGH:
            /* أقصى حساسية: استجابة سريعة، رفض قليل */
            window = (window > 6) ? window - 6 : 4;
            alpha  = fminf(alpha + 0.15f, 0.75f);
            sigma  = fmaxf(sigma - 0.8f,  1.5f);
            break;

        case SENS_MODE_HIGH:
            /* حساسية عالية */
            window = (window > 3) ? window - 3 : 4;
            alpha  = fminf(alpha + 0.08f, 0.65f);
            sigma  = fmaxf(sigma - 0.4f,  1.8f);
            break;

        case SENS_MODE_MEDIUM:
        case SENS_MODE_AUTO:
        default:
            /* نفس قيم التربة — بدون تعديل */
            break;

        case SENS_MODE_LOW:
            /* حساسية منخفضة: تصفية أقوى */
            window = ((uint16_t)window + 8 <= GRAD_FILTER_MAX_WINDOW)
                     ? window + 8 : GRAD_FILTER_MAX_WINDOW;
            alpha  = fmaxf(alpha - 0.08f, 0.05f);
            sigma  = fminf(sigma + 0.5f,  4.5f);
            break;

        case SENS_MODE_VERY_LOW:
            /* أدنى حساسية: تصفية قصوى للبيئات الصعبة جداً */
            window = ((uint16_t)window + 16 <= GRAD_FILTER_MAX_WINDOW)
                     ? window + 16 : GRAD_FILTER_MAX_WINDOW;
            alpha  = fmaxf(alpha - 0.15f, 0.03f);
            sigma  = fminf(sigma + 0.8f,  5.0f);
            break;
    }

    out_result->window_size   = window;
    out_result->alpha_smooth  = alpha;
    out_result->outlier_sigma = sigma;
}

/* =========================================================================
 * ANALYSIS PHASE (runs synchronously after collection ends)
 * ========================================================================= */

static CalibEngineStatus_t run_analysis(CalibEngine_t *eng)
{
    eng->status = CALIB_STATUS_COMPUTING;
    ESP_LOGI(TAG, "Running analysis on %u samples (%u rejected during collection)",
             eng->ring_count, eng->rejected_count);

    /* --- Validity check: enough samples? --- */
    uint32_t elapsed_ms = (xTaskGetTickCount() - eng->start_tick) * portTICK_PERIOD_MS;
    uint32_t expected_samples = (elapsed_ms * GRAD_ADC_SAMPLE_RATE_HZ) / 1000u;
    float    valid_ratio = (expected_samples > 0)
                           ? (float)eng->ring_count / (float)expected_samples
                           : 0.0f;

    if (valid_ratio < CALIB_MIN_VALID_RATIO) {
        ESP_LOGE(TAG, "Insufficient samples: %u collected, %lu expected (%.0f%% valid)",
                 eng->ring_count, (unsigned long)expected_samples, valid_ratio * 100.0f);
        eng->result.is_valid = false;
        return CALIB_STATUS_ERROR;
    }

    /* --- Step 1: Get Welford statistics --- */
    float raw_mean    = (float)eng->welford_mean;
    float raw_var     = (float)welford_get_variance(eng);
    float raw_std_dev = sqrtf(raw_var);

    ESP_LOGI(TAG, "Raw stats — Mean: %.2f, Std: %.3f, N: %lu",
             raw_mean, raw_std_dev, (unsigned long)eng->welford_n);

    /* --- Step 2: Outlier removal on ring buffer copy --- */
    /* Work on a local copy so we don't corrupt the ring */
    static int16_t work_buf[CALIB_RING_SIZE];
    uint16_t       work_n = eng->ring_count;
    memcpy(work_buf, eng->ring, work_n * sizeof(int16_t));

    float clean_mean, clean_std;
    uint16_t outliers_removed = remove_outliers(
        work_buf, work_n, raw_mean, raw_std_dev,
        &clean_mean, &clean_std
    );

    ESP_LOGI(TAG, "After outlier removal: Mean: %.2f, Std: %.3f, Removed: %u",
             clean_mean, clean_std, outliers_removed);

    /* --- Step 3: Drift estimation --- */
    /* Rebuild clean sample array (skip sentinels) */
    static int16_t clean_buf[CALIB_RING_SIZE];
    uint16_t clean_n = 0;
    for (uint16_t i = 0; i < work_n; i++) {
        if (work_buf[i] != INT16_MIN) {
            clean_buf[clean_n++] = work_buf[i];
        }
    }

    float drift_per_sample = estimate_drift(clean_buf, clean_n);
    float drift_per_second = drift_per_sample * (float)GRAD_ADC_SAMPLE_RATE_HZ;

    if (fabsf(drift_per_second) > CALIB_MAX_DRIFT_RATE) {
        ESP_LOGW(TAG, "WARNING: Significant drift detected: %.3f counts/sec",
                 drift_per_second);
        ESP_LOGW(TAG, "  → Calibration may be unreliable. Hold device still!");
        /* Don't fail — just warn. Operator may accept this. */
    } else {
        ESP_LOGI(TAG, "Drift: %.4f counts/sec (acceptable)", drift_per_second);
    }

    /* --- Step 4: Compute min/max/peak-to-peak --- */
    int16_t min_val = clean_buf[0], max_val = clean_buf[0];
    for (uint16_t i = 1; i < clean_n; i++) {
        if (clean_buf[i] < min_val) min_val = clean_buf[i];
        if (clean_buf[i] > max_val) max_val = clean_buf[i];
    }

    /* --- Step 5: Soil classification & recommendations --- */
    SoilType_t        soil      = calib_classify_soil(clean_std);
    SensitivityMode_t recommend = calib_recommend_sensitivity(soil);

    const char *soil_names[] = {"UNKNOWN", "PRISTINE", "CLEAN", "MINERAL", "NOISY", "EXTREME"};
    const char *mode_names[] = {"AUTO", "VERY_HIGH", "HIGH", "MEDIUM", "LOW", "VERY_LOW"};

    ESP_LOGI(TAG, "=== SOIL ANALYSIS ===");
    ESP_LOGI(TAG, "  Noise floor (std): %.3f ADC counts", clean_std);
    ESP_LOGI(TAG, "  Classification:    %s", soil_names[(int)soil]);
    ESP_LOGI(TAG, "  Recommendation:    SENSITIVITY_%s", mode_names[(int)recommend]);

    /* --- Step 6: Compute filter parameters --- */
    CalibResult_t result;
    memset(&result, 0, sizeof(result));

    calib_compute_filter_params(soil, recommend, &result);

    ESP_LOGI(TAG, "  Filter window:     %d samples", result.window_size);
    ESP_LOGI(TAG, "  EMA alpha:         %.2f", result.alpha_smooth);
    ESP_LOGI(TAG, "  Outlier sigma:     %.1f", result.outlier_sigma);

    /* --- Step 7: Fill result struct --- */
    result.mean               = clean_mean;
    result.std_dev            = clean_std;
    result.variance           = clean_std * clean_std;
    result.min_val            = (float)min_val;
    result.max_val            = (float)max_val;
    result.peak_to_peak       = (float)(max_val - min_val);
    result.drift_rate         = drift_per_second;
    result.soil_type          = soil;
    result.recommended_mode   = recommend;
    result.samples_collected  = clean_n;
    result.timestamp_ms       = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    result.is_valid           = true;

    eng->result = result;
    eng->status = CALIB_STATUS_DONE;

    ESP_LOGI(TAG, "=== CALIBRATION COMPLETE ===");
    ESP_LOGI(TAG, "  Baseline:    %.2f ADC counts", result.mean);
    ESP_LOGI(TAG, "  Noise:       %.3f (std)", result.std_dev);
    ESP_LOGI(TAG, "  Peak-peak:   %.1f ADC counts", result.peak_to_peak);
    ESP_LOGI(TAG, "  Valid ratio: %.0f%%", valid_ratio * 100.0f);
    ESP_LOGI(TAG, "  Drift:       %.4f counts/sec", result.drift_rate);

    return CALIB_STATUS_DONE;
}

/* =========================================================================
 * FEED (called per-sample from signal_task)
 * ========================================================================= */

CalibEngineStatus_t calib_engine_feed(CalibEngine_t *eng, const AdcSample_t *sample)
{
    if (eng->status != CALIB_STATUS_IN_PROGRESS) {
        return eng->status;
    }

    eng->total_fed++;

    /* Reject low-quality samples */
    if (sample->quality < CALIB_MIN_SAMPLE_QUALITY) {
        eng->rejected_count++;
        return CALIB_STATUS_IN_PROGRESS;
    }

    /* Accept sample */
    int16_t value = sample->differential;

    /* Store in ring buffer */
    if (eng->ring_count < CALIB_RING_SIZE) {
        eng->ring[eng->ring_head] = value;
        eng->ring_head = (eng->ring_head + 1) % CALIB_RING_SIZE;
        eng->ring_count++;
    }
    /* If ring full: this is unlikely (500 slots for 10s), but safe anyway */

    /* Update Welford running statistics */
    welford_update(eng, value);

    /* Check if collection window is complete */
    uint32_t elapsed_ms = (xTaskGetTickCount() - eng->start_tick) * portTICK_PERIOD_MS;

    if (elapsed_ms >= CALIB_DURATION_MS) {
        ESP_LOGI(TAG, "Collection window complete: %u ms, %u samples",
                 elapsed_ms, eng->ring_count);
        return run_analysis(eng);
    }

    return CALIB_STATUS_IN_PROGRESS;
}

/* =========================================================================
 * QUERIES
 * ========================================================================= */

uint8_t calib_engine_get_progress(const CalibEngine_t *eng)
{
    if (!eng->started) return 0;
    if (eng->status == CALIB_STATUS_DONE) return 100;
    if (eng->status == CALIB_STATUS_ERROR) return 100;

    uint32_t elapsed = (xTaskGetTickCount() - eng->start_tick) * portTICK_PERIOD_MS;
    if (elapsed >= CALIB_DURATION_MS) return 99;

    return (uint8_t)((elapsed * 100u) / CALIB_DURATION_MS);
}

esp_err_t calib_engine_get_result(const CalibEngine_t *eng, CalibResult_t *out_result)
{
    if (eng->status != CALIB_STATUS_DONE) {
        return ESP_ERR_INVALID_STATE;
    }
    *out_result = eng->result;
    return ESP_OK;
}

CalibEngineStatus_t calib_engine_get_status(const CalibEngine_t *eng)
{
    return eng->status;
}
