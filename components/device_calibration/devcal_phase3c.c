/**
 * @file devcal_phase3c.c
 * @brief Phase 3C — Static Sensor Matching (4000 gated samples per channel)
 *
 * ═══════════════════════════════════════════════════════════════════
 * DESIGN DECISIONS (based on multi-AI review):
 *
 * 1. FORMULA — why we use precomputed addend, not s1-based formula:
 *
 *    Ideal: corrected = s1 - (s2 - offset2) × gain2
 *    = gain2×raw_diff + (1-gain2)×s1 + offset2×gain2
 *
 *    Problem: s1 (AIN0 alone) is NOT available during differential
 *    operation — ADS1115 computes AIN0-AIN1 internally in the MUX.
 *    Switching MUX per-sample would double I2C traffic (unacceptable).
 *
 *    We use precomputed constant from Phase 3C calibration means:
 *      match_addend = (ain0_dc - ain1_dc) - (gain2-1)*ain1_dc + offset2*gain2
 *                   = ain0_dc - ain1_dc*gain2 + offset2*gain2
 *
 *    Residual error = (1-gain2) × Δs2(t)
 *    With gain2=1.01 and typical Δs2=±200 LSB: error ≈ ±2 LSB.
 *    This is acceptable (< threshold for deep targets at ~5-10 LSB).
 *    The offset correction (dominant term) is exact in both approaches.
 *
 * 2. GATING during collection (ChatGPT/DeepSeek recommendation):
 *    Only accumulate samples where |current_diff| < 3×noise_floor.
 *    Prevents targets or motion artifacts from biasing offset2/gain2.
 *    noise_floor estimated from first 100 samples (bootstrap phase).
 *
 * 3. DEAD-BAND reduced to 0.2% (from 0.5%):
 *    Detects smaller gain mismatches for higher baseline accuracy.
 *
 * 4. FALLBACK: if Phase 3C not run → apply offset only, gain=1.0.
 *    Never apply an uncalibrated gain (could worsen signal).
 *
 * 5. TIMING: 4000 target samples per channel + gating overhead ≈ 30s.
 *    Actual time depends on soil noise level (gate reject rate).
 * ═══════════════════════════════════════════════════════════════════
 */

#include "devcal_phase3c.h"
#include "devcal_common.h"
#include "adc_task.h"
#include "ads1115_driver.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "DevCal_P3C";

/* ── Configuration ──────────────────────────────────────────────── */
#define P3C_SAMPLES_TARGET   4000u    /* gated samples per channel    */
#define P3C_BOOTSTRAP_N      100u     /* samples for noise estimation */
#define P3C_GATE_SIGMA       3.0f     /* reject |x - mean| > N×σ     */
#define P3C_SAMPLE_DELAY_MS  3u       /* 3ms between reads @ 860SPS   */
#define P3C_PROGRESS_STEP    40u      /* update UI every N accepted   */
#define P3C_MAX_GAIN_RATIO   1.15f    /* reject gain > 15% mismatch   */
#define P3C_GAIN_DEADBAND    0.002f   /* ignore gain correction <0.2% */
#define P3C_MIN_VALID_RATIO  0.90f    /* require ≥90% acceptance      */
#define P3C_MAX_OVERSAMPLE   3u       /* max extra rounds for gating  */

/* ══════════════════════════════════════════════════════════════════
 * Welford's Online Algorithm — double precision
 * Numerically stable for large N and large absolute values (~20kLSB)
 * ══════════════════════════════════════════════════════════════════ */
typedef struct {
    double   mean;
    double   M2;
    uint32_t n;
} Welford_t;

static void w_init(Welford_t *w) { w->mean=0.0; w->M2=0.0; w->n=0; }

static void w_update(Welford_t *w, double x)
{
    w->n++;
    double delta = x - w->mean;
    w->mean     += delta / (double)w->n;
    w->M2       += delta * (x - w->mean);
}

static double w_std(const Welford_t *w)
{
    if (w->n < 2) return 0.0;
    return sqrt(w->M2 / (double)(w->n));   /* population std dev */
}

/* ══════════════════════════════════════════════════════════════════
 * bootstrap_noise: collect P3C_BOOTSTRAP_N samples to estimate
 * noise floor for gate threshold. Fast, ungated (first N samples).
 * ══════════════════════════════════════════════════════════════════ */
static float bootstrap_noise(ADS1115Driver_t *drv, uint8_t channel)
{
    Welford_t w;
    w_init(&w);
    for (uint32_t i = 0; i < P3C_BOOTSTRAP_N; i++) {
        int16_t v = 0;
        if (ads1115_read_single_ended(drv, channel, &v) == ESP_OK) {
            w_update(&w, (double)v);
        }
        vTaskDelay(pdMS_TO_TICKS(P3C_SAMPLE_DELAY_MS));
    }
    float noise = (float)w_std(&w);
    ESP_LOGI(TAG, "AIN%u bootstrap: mean=%.1f std=%.2f", channel, w.mean, noise);
    return (noise > 0.1f) ? noise : 1.0f;   /* floor: never zero */
}

/* ══════════════════════════════════════════════════════════════════
 * collect_gated: accumulate exactly n_target ACCEPTED samples.
 *
 * Gating rule (ChatGPT/DeepSeek recommendation):
 *   Accept sample only if |sample - running_mean| < gate_threshold
 *   gate_threshold = P3C_GATE_SIGMA × noise_floor
 *
 * Why: any target or motion spike would bias the mean and std
 * of the per-channel absolute DC level. We only want quiet soil.
 *
 * Returns actual accepted count (may be < n_target on I2C errors).
 * ══════════════════════════════════════════════════════════════════ */
static uint32_t collect_gated(ADS1115Driver_t *drv,
                               uint8_t          channel,
                               uint32_t          n_target,
                               float             noise_floor,
                               Welford_t        *w)
{
    w_init(w);
    uint32_t accepted  = 0;
    uint32_t rejected  = 0;
    uint32_t i2c_err   = 0;
    float    gate_thr  = P3C_GATE_SIGMA * noise_floor;
    bool     gate_ready = false;  /* active only after ≥10 accepted samples */

    uint64_t t_start = (uint64_t)(esp_timer_get_time() / 1000LL);

    while (accepted < n_target) {
        int16_t raw = 0;
        esp_err_t ret = ads1115_read_single_ended(drv, channel, &raw);
        if (ret != ESP_OK) {
            i2c_err++;
            if (i2c_err > n_target / 5u) break;
            vTaskDelay(pdMS_TO_TICKS(P3C_SAMPLE_DELAY_MS));
            continue;
        }

        /* ── Gating (ChatGPT Production Audit — improved formula) ──────
         * Use |sample - running_mean| < gate_threshold
         * NOT |sample| < gate_threshold.
         *
         * Why: in non-homogeneous soil or with any DC offset, the absolute
         * value of a single-ended reading can be large (8000–20000 LSB).
         * Gating on raw |sample| would reject valid quiet-soil readings.
         * Gating on deviation from running_mean correctly rejects only
         * sudden changes (target, motion, EMI spike), regardless of the
         * absolute DC level of the sensor.
         *
         * Example: mean_AIN0 = 15200 LSB (Earth field DC), noise σ = 6 LSB.
         *   Old gate: |15206| < 18 → FALSE → reject (wrong!)
         *   New gate: |15206 - 15200| < 18 → TRUE → accept (correct)
         * ─────────────────────────────────────────────────────────── */
        if (gate_ready) {
            double deviation = fabs((double)raw - w->mean);  /* deviation from running mean */
            if (deviation > (double)gate_thr) {
                rejected++;
                vTaskDelay(pdMS_TO_TICKS(P3C_SAMPLE_DELAY_MS));
                continue;
            }
        }

        w_update(w, (double)raw);
        accepted++;
        if (accepted == 10u) gate_ready = true;

        /* UI update */
        if ((accepted % P3C_PROGRESS_STEP) == 0) {
            uint32_t pct = accepted * 100u / n_target;
            uint32_t elapsed = (uint32_t)((uint64_t)(esp_timer_get_time()/1000LL) - t_start);
            uint32_t rate    = accepted > 0 ? (accepted * 1000u / (elapsed > 0 ? elapsed : 1)) : 1;
            uint32_t remain_ms = (n_target - accepted) * 1000u / (rate > 0 ? rate : 1);

            char buf[64];
            snprintf(buf, sizeof(buf),
                     "AIN%u | mean=%.1f  σ=%.2f\nOK:%lu  Skip:%lu",
                     (unsigned)channel, w->mean, w_std(w),
                     (unsigned long)accepted, (unsigned long)rejected);
            devcal_ui_val(buf, lv_color_hex(0x00D4FFu));
            devcal_ui_prog((int32_t)pct, "");
            devcal_ui_timer(remain_ms);
        }

        vTaskDelay(pdMS_TO_TICKS(P3C_SAMPLE_DELAY_MS));
    }

    ESP_LOGI(TAG, "AIN%u: accepted=%lu rejected=%lu i2c_err=%lu | mean=%.3f std=%.3f",
             (unsigned)channel,
             (unsigned long)accepted, (unsigned long)rejected, (unsigned long)i2c_err,
             w->mean, w_std(w));
    return accepted;
}

/* ══════════════════════════════════════════════════════════════════
 * PUBLIC ENTRY POINT
 * ══════════════════════════════════════════════════════════════════ */
void devcal_run_phase3c(DeviceProfile_t *p)
{
    ESP_LOGI(TAG, "=== Phase 3C: Static Sensor Matching (%u gated samples/ch) ===",
             (unsigned)P3C_SAMPLES_TARGET);

    /* ── Intro ────────────────────────────────────────────────────── */
    devcal_ui_set(
        "Phase 3C / 5  —  Sensor Matching",
        "Static DC Offset + Gain  (4000 gated samples)",
        "Place rod on flat NON-METALLIC surface.\n"
        "Point AWAY from electronics and metal.\n"
        "Must be COMPLETELY STILL — ~30 seconds.\n"
        "Motion samples are auto-rejected (gated).",
        "Measures DC offset + sensitivity ratio between\n"
        "the two FLC100 sensors for baseline accuracy.\n"
        "Hold SCAN 2s to skip."
    );
    devcal_ui_btn_prompt(LV_SYMBOL_PLAY " Place rod still — Press SCAN to start");

    bool go = devcal_btn_wait(0);
    if (!go) {
        /* Fallback: offset from Phase 1, gain disabled */
        p->sensor_offset2      = (float)p->nominal_offset;
        p->sensor_gain2        = 1.0f;
        p->ain1_dc_level       = 0.0f;
        p->sensor_match_addend = (float)p->nominal_offset;
        p->p3c_std_ain0 = p->p3c_std_ain1 = 0.0f;
        p->p3c_samples  = 0u;
        ESP_LOGI(TAG, "Phase 3C skipped — offset=%.1f (P1), gain disabled",
                 p->sensor_offset2);
        devcal_ui_result_fail("3C skipped — Phase 1 offset used, no gain");
        return;
    }

    ADS1115Driver_t *drv = adc_task_get_driver();
    if (!drv) {
        ESP_LOGE(TAG, "ADC driver unavailable");
        devcal_ui_result_fail("3C failed — ADC driver NULL");
        p->sensor_offset2 = 0.0f; p->sensor_gain2 = 1.0f;
        p->sensor_match_addend = 0.0f;
        return;
    }

    /* ── Bootstrap noise estimate per channel ─────────────────────── */
    devcal_ui_set("Phase 3C  —  Noise Estimation",
                  "Estimating noise floor (100 samples)...",
                  "Hold still — takes ~1 second.", "");
    devcal_ui_btn_prompt("");

    float noise0 = bootstrap_noise(drv, 0);
    float noise1 = bootstrap_noise(drv, 1);
    float noise_avg = (noise0 + noise1) * 0.5f;

    ESP_LOGI(TAG, "Gate threshold: ±%.1f LSB (%.1f × %.2f σ)",
             P3C_GATE_SIGMA * noise_avg, (float)P3C_GATE_SIGMA, noise_avg);

    /* ── Collect AIN0 ─────────────────────────────────────────────── */
    devcal_ui_set(
        "Phase 3C-A / 5  —  Upper Sensor (AIN0)",
        "Collecting 4000 gated samples...",
        "HOLD COMPLETELY STILL\n"
        "Motion samples are automatically skipped.\n"
        "Progress shows accepted samples only.",
        "Measuring upper FLC100 sensor absolute level."
    );
    devcal_ui_btn_prompt("");

    Welford_t w0;
    uint32_t acc0 = collect_gated(drv, 0, P3C_SAMPLES_TARGET, noise0, &w0);

    /* ── Collect AIN1 ─────────────────────────────────────────────── */
    devcal_ui_set(
        "Phase 3C-B / 5  —  Lower Sensor (AIN1)",
        "Collecting 4000 gated samples...",
        "KEEP HOLDING STILL — almost done!\n"
        "Measuring lower FLC100 sensor now.",
        "Measuring lower FLC100 sensor absolute level."
    );
    devcal_ui_btn_prompt("");

    Welford_t w1;
    uint32_t acc1 = collect_gated(drv, 1, P3C_SAMPLES_TARGET, noise1, &w1);

    /* ── Restore differential mode ────────────────────────────────── */
    for (int attempt = 0; attempt < 3; attempt++) {
        if (ads1115_driver_start(drv) == ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "ADS1115 restored to differential mode");

    /* ── Validate ─────────────────────────────────────────────────── */
    float r0 = (float)acc0 / (float)P3C_SAMPLES_TARGET;
    float r1 = (float)acc1 / (float)P3C_SAMPLES_TARGET;

    if (r0 < P3C_MIN_VALID_RATIO || r1 < P3C_MIN_VALID_RATIO) {
        ESP_LOGE(TAG, "Insufficient accepted samples: AIN0=%lu AIN1=%lu",
                 (unsigned long)acc0, (unsigned long)acc1);
        /* Still store partial results with gain=1 fallback */
        p->sensor_offset2      = (float)(w0.mean - w1.mean);
        p->sensor_gain2        = 1.0f;
        /* Partial: store partial results, addend = offset only (no gain) */
        p->ain1_dc_level       = (float)w1.mean;
        p->sensor_match_addend = (float)(w0.mean - w1.mean);  /* offset only, gain=1 */
        p->p3c_samples         = (acc0 + acc1) / 2u;
        devcal_ui_result_fail("3C partial — too many rejections, retry in open area");
        return;
    }

    /* ── Compute offset2 ──────────────────────────────────────────
     * offset2 = mean_AIN0 - mean_AIN1
     * In uniform Earth field with perfect sensors → 0.
     * Non-zero value = DC mismatch between the two coils.
     * Standard Error = σ/√N ≈ 8/√4000 = 0.13 LSB (6× better than P1)
     * ─────────────────────────────────────────────────────────── */
    float offset2 = (float)(w0.mean - w1.mean);

    /* ── Compute gain2 ────────────────────────────────────────────
     * gain2 = std_AIN0 / std_AIN1
     * Both sensors see same Earth field noise → std ratio = sensitivity ratio.
     * Dead-band 0.2%: below this, sensors are effectively matched.
     * ─────────────────────────────────────────────────────────── */
    double std0 = w_std(&w0);
    double std1 = w_std(&w1);
    float  gain2 = 1.0f;

    if (std1 > 0.3 && std0 > 0.3) {
        float raw_ratio = (float)(std0 / std1);
        if (raw_ratio > P3C_MAX_GAIN_RATIO || raw_ratio < (1.0f / P3C_MAX_GAIN_RATIO)) {
            ESP_LOGW(TAG, "gain ratio %.4f out of bounds [%.3f..%.3f] — rejected",
                     raw_ratio, 1.0f/P3C_MAX_GAIN_RATIO, P3C_MAX_GAIN_RATIO);
            gain2 = 1.0f;
        } else if (fabsf(raw_ratio - 1.0f) < P3C_GAIN_DEADBAND) {
            ESP_LOGI(TAG, "gain ratio %.4f within %.1f%% deadband — no correction",
                     raw_ratio, P3C_GAIN_DEADBAND * 100.0f);
            gain2 = 1.0f;
        } else {
            gain2 = raw_ratio;
        }
    }

    /* ── Precompute sensor_match_addend ───────────────────────────
     * Runtime correction formula (O(1) per sample):
     *   corrected_diff = raw_diff + match_addend
     *
     * Derivation (using P3C calibration means as DC reference):
     *   s1 ≈ ain0_dc + Δcommon  (Earth field + common motion)
     *   s2 ≈ ain1_dc + Δcommon  (same common component)
     *   true_diff = s1 - (s2 - offset2) × gain2
     *             = (s1-s2) + (1-gain2)*s2 + offset2*gain2
     *   Using s2 ≈ ain1_dc:
     *   match_addend = (1-gain2)*ain1_dc + offset2*gain2
     *
     * Residual error = (1-gain2) × Δs2(t)
     * With gain2=1.01, Δs2=±200 LSB → error ≈ ±2 LSB (acceptable).
     * The dominant offset term (offset2) is corrected exactly.
     *
     * NOTE: s1-based formula (gain2*diff + (1-gain2)*s1 + b) would be
     * more accurate but requires s1 independently — not available in
     * ADS1115 differential mode without MUX switching (unacceptable cost).
     * ─────────────────────────────────────────────────────────── */
    float ain1_dc = (float)w1.mean;
    float addend  = (1.0f - gain2) * ain1_dc + offset2 * gain2;

    /* ── Clamp addend (ChatGPT Production Audit recommendation) ─────
     * In extreme edge cases (strong tilt, steep natural gradient,
     * ferrous soil), Δs2(t) can reach 300–500 LSB, causing the
     * residual error (1-gain2)×Δs2 to reach 3–5 LSB — same order as
     * deep target signals.
     *
     * Clamping addend to ±200 LSB prevents catastrophic bias in these
     * cases without affecting normal operation (typical addend < 30 LSB).
     * The ±200 LSB limit covers any realistic FLC100 DC mismatch.
     * ─────────────────────────────────────────────────────────────── */
    if      (addend >  200.0f) { addend =  200.0f; ESP_LOGW(TAG, "addend clamped to +200"); }
    else if (addend < -200.0f) { addend = -200.0f; ESP_LOGW(TAG, "addend clamped to -200"); }

    /* ── Store results ────────────────────────────────────────────── */
    p->sensor_offset2      = offset2;
    p->sensor_gain2        = gain2;
    p->ain1_dc_level       = ain1_dc;
    p->sensor_match_addend = addend;
    p->p3c_std_ain0        = (float)std0;
    p->p3c_std_ain1        = (float)std1;
    p->p3c_samples         = (acc0 + acc1) / 2u;
    p->nominal_offset      = (int32_t)(offset2 + 0.5f);   /* update P1 value */
    p->phases_completed   |= (1u << 5u);

    /* ── Quality strings ──────────────────────────────────────────── */
    float a = fabsf(offset2);
    const char *oq = a<2.0f ? "Excellent" : a<8.0f ? "Good" :
                     a<20.0f ? "Acceptable" : "High — check sensor";
    float ge = fabsf(gain2 - 1.0f) * 100.0f;
    const char *gq = ge<0.2f ? "Matched" : ge<1.0f ? "Good" :
                     ge<5.0f ? "Acceptable" : "Significant mismatch";

    float se = (float)(std0 / sqrt((double)acc0));   /* standard error of mean */

    ESP_LOGI(TAG, "RESULTS: offset=%.3f LSB [%s] | gain=%.4f [%s]",
             offset2, oq, gain2, gq);
    ESP_LOGI(TAG, "SE_mean=%.4f LSB | std0=%.2f std1=%.2f",
             se, (float)std0, (float)std1);
    ESP_LOGI(TAG, "match_addend=%.3f (additive O(1) correction)", addend);

    char result[160];
    snprintf(result, sizeof(result),
             "offset: %.1f LSB  %s\n"
             "gain:   %.4f      %s\n"
             "SE_mean: %.3f LSB  (σ/√%lu)\n"
             "gated: %lu+%lu accepted / %u each",
             offset2, oq, gain2, gq,
             se, (unsigned long)acc0,
             (unsigned long)acc0, (unsigned long)acc1,
             (unsigned)P3C_SAMPLES_TARGET);

    devcal_ui_val(result, lv_color_hex(0x00D4FFu));
    devcal_ui_result_ok("Phase 3C complete " LV_SYMBOL_OK);
}
 /*
 * ═══════════════════════════════════════════════════════════════════
 * PHYSICS BACKGROUND
 * ═══════════════════════════════════════════════════════════════════
 *
 * The ADS1115 in normal operation measures: diff = AIN0 − AIN1
 *
 * In a perfect sensor pair (identical FLC100 coils):
 *   diff in uniform Earth field = 0 (no anomaly present)
 *
 * In reality, two physical sensors always have:
 *   a) DC offset mismatch: different zero-field output voltages
 *      → diff in uniform Earth field ≠ 0 → false baseline gradient
 *   b) Gain mismatch: different sensitivity (LSB/nT)
 *      → same anomaly produces slightly different response per sensor
 *      → differential output distorted at strong field angles
 *
 * Phase 3C measures each sensor independently against GND:
 *   mean_AIN0, std_AIN0 (4000 samples, Welford algorithm)
 *   mean_AIN1, std_AIN1 (4000 samples, Welford algorithm)
 *
 * Then computes corrections:
*/