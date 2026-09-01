/**
 * @file devcal_phase2.c
 * @brief Phase 2 — Sensor Matching + Dynamic Range
 *
 * PROCEDURE:
 *   Step A: Hold bar magnet BELOW rod, North pole UP (toward sensors)
 *           → gradient_N (should be POSITIVE — metal-like response)
 *   Step B: FLIP magnet, South pole UP
 *           → gradient_S (should be NEGATIVE — void-like response)
 *   Step C: Remove magnet → verify baseline returns
 *
 * PHYSICS:
 *   asymmetry = (grad_N + grad_S) / 2
 *   If asymmetry ≠ 0 → sensors are mismatched.
 *   We subtract asymmetry from every reading to compensate.
 *
 *   dynamic_range = |grad_N| + |grad_S|
 *   kalman_Q = (noise_floor / range)² × 0.1
 *   → Smaller range relative to noise → larger Q (less trust in model)
 */

#include "devcal_phase2.h"
#include "devcal_common.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>

static const char *TAG = "DevCal_P2";

#define P2_SETTLE_MS   3000u    /* Wait after placing magnet */
#define P2_MEASURE_N   160u     /* was 80 — doubled: 8s per measurement step */

void devcal_run_phase2(DeviceProfile_t *p)
{
    ESP_LOGI(TAG, "=== Phase 2: Sensor Matching + Dynamic Range ===");

    /* ────────────────────────────────────────────────
     * STEP A — North pole
     * ──────────────────────────────────────────────── */
    devcal_ui_set(
        "Phase 2A / 5  —  Magnet North",
        "Sensor Matching — North Pole",

        "Hold BAR MAGNET below the rod\n"
        "North pole facing UP toward sensors\n"
        "Distance: ~5 to 10 cm from rod bottom",

        "PRO TIP: Use a strong bar magnet.\n"
        "Ferrite fridge magnets are too weak.\n"
        "Hold 2s to skip this phase"
    );
    devcal_ui_btn_prompt(LV_SYMBOL_PLAY " Press SCAN when magnet is in position");

    bool go = devcal_btn_wait(0);
    if (!go) {
        /* Skip — safe defaults */
        p->gradient_N    = 200.0f;
        p->gradient_S    = -200.0f;
        p->dynamic_range = 400.0f;
        p->asymmetry     = 0.0f;
        p->sensitivity   = 200.0f;
        p->kalman_Q = (p->noise_floor > 0 && p->dynamic_range > 0) ?
                      (p->noise_floor / p->dynamic_range) *
                      (p->noise_floor / p->dynamic_range) * 0.1f : 0.001f;
        p->phases_completed |= (1 << 1);
        devcal_ui_result_fail("Phase 2 skipped — using defaults");
        return;
    }

    /* Settle */
    devcal_ui_set("Phase 2A / 5", "Settling...",
                  "HOLD STILL — magnet in place", "");
    devcal_ui_btn_prompt("");

    for (uint32_t i = 0; i < P2_SETTLE_MS / DCAL_SAMPLE_MS; i++) {
        devcal_ui_prog(i * 100 / (P2_SETTLE_MS / DCAL_SAMPLE_MS),
                       "Settling...");
        devcal_read_raw();   /* discard settle samples */
        vTaskDelay(pdMS_TO_TICKS(DCAL_SAMPLE_MS));
    }

    devcal_ui_set("Phase 2A / 5", "Measuring North Pole Response",
                  "KEEP STILL — magnet and device",
                  "Expected: POSITIVE value (metal response)");

    float mean_N, std_N;
    devcal_collect_stats(P2_MEASURE_N, &mean_N, &std_N, g_ui.bar_prog);

    char vbuf[32];
    snprintf(vbuf, sizeof(vbuf), "%.0f LSB", mean_N);
    devcal_ui_val(vbuf,
                  mean_N > 0 ? lv_color_hex(DCOL_GREEN) : lv_color_hex(DCOL_RED));

    const char *polarity_hint = (mean_N > 10.0f) ?
        "Good — positive response as expected" :
        (mean_N < -10.0f) ? "Inverted — check wiring (AIN0/AIN1)" :
        "Weak signal — move magnet closer";
    devcal_ui_val2(polarity_hint,
                   fabsf(mean_N) > 10.0f ?
                   lv_color_hex(DCOL_GREEN) : lv_color_hex(DCOL_AMBER));

    ESP_LOGI(TAG, "P2A: gradient_N = %.1f  std=%.2f", mean_N, std_N);
    vTaskDelay(pdMS_TO_TICKS(1500));

    /* ────────────────────────────────────────────────
     * STEP B — South pole (flip magnet)
     * ──────────────────────────────────────────────── */
    devcal_ui_set(
        "Phase 2B / 5  —  Magnet South",
        "Sensor Matching — South Pole",

        "FLIP the magnet\n"
        "South pole now facing UP toward sensors\n"
        "Same position, same distance",

        "PRO TIP: Same spot, just flip.\n"
        "Expected: NEGATIVE value (void response)"
    );
    devcal_ui_btn_prompt(LV_SYMBOL_PLAY " Press SCAN when magnet is flipped");
    devcal_btn_wait(0);

    devcal_ui_set("Phase 2B / 5", "Settling...",
                  "HOLD STILL — flipped magnet in place", "");
    devcal_ui_btn_prompt("");

    for (uint32_t i = 0; i < P2_SETTLE_MS / DCAL_SAMPLE_MS; i++) {
        devcal_ui_prog(i * 100 / (P2_SETTLE_MS / DCAL_SAMPLE_MS), "");
        devcal_read_raw();
        vTaskDelay(pdMS_TO_TICKS(DCAL_SAMPLE_MS));
    }

    devcal_ui_set("Phase 2B / 5", "Measuring South Pole Response",
                  "KEEP STILL",
                  "Expected: NEGATIVE value (void response)");

    float mean_S, std_S;
    devcal_collect_stats(P2_MEASURE_N, &mean_S, &std_S, g_ui.bar_prog);

    snprintf(vbuf, sizeof(vbuf), "%.0f LSB", mean_S);
    devcal_ui_val(vbuf,
                  mean_S < 0 ? lv_color_hex(DCOL_GREEN) : lv_color_hex(DCOL_AMBER));
    ESP_LOGI(TAG, "P2B: gradient_S = %.1f  std=%.2f", mean_S, std_S);
    vTaskDelay(pdMS_TO_TICKS(1500));

    /* ────────────────────────────────────────────────
     * STEP C — Remove magnet + verify
     * ──────────────────────────────────────────────── */
    devcal_ui_set(
        "Phase 2C / 5  —  Verify Baseline",
        "Remove Magnet — Baseline Check",
        "Remove magnet completely\nMove it at least 1m away",
        "Verifying baseline returns to offset"
    );
    devcal_ui_btn_prompt(LV_SYMBOL_PLAY " Press SCAN after removing magnet");
    devcal_btn_wait(0);

    float mean_bl, std_bl;
    devcal_collect_stats(40, &mean_bl, &std_bl, g_ui.bar_prog);

    float baseline_error = fabsf(mean_bl - (float)p->nominal_offset);
    snprintf(vbuf, sizeof(vbuf), "%.0f LSB  (err:%.0f)", mean_bl, baseline_error);
    devcal_ui_val(vbuf,
                  baseline_error < p->noise_floor * 3.0f ?
                  lv_color_hex(DCOL_GREEN) : lv_color_hex(DCOL_AMBER));
    ESP_LOGI(TAG, "P2C: baseline=%.1f  error=%.1f", mean_bl, baseline_error);
    vTaskDelay(pdMS_TO_TICKS(1200));

    /* ────────────────────────────────────────────────
     * COMPUTE RESULTS
     * ──────────────────────────────────────────────── */
    float range     = fabsf(mean_N) + fabsf(mean_S);
    float asymmetry = (mean_N + mean_S) / 2.0f;

    p->gradient_N    = mean_N;
    p->gradient_S    = mean_S;
    p->dynamic_range = range;
    p->asymmetry     = asymmetry;
    p->sensitivity   = range / 2.0f;

    if (range > 1.0f && p->noise_floor > 0.0f) {
        float ratio  = p->noise_floor / range;
        p->kalman_Q  = ratio * ratio * 0.1f;
    } else {
        p->kalman_Q  = 0.001f;
    }

    p->phases_completed |= (1 << 1);

    ESP_LOGI(TAG, "P2 done: range=%.1f  asym=%.1f  Q=%.6f",
             range, asymmetry, p->kalman_Q);

    char res[80];
    snprintf(res, sizeof(res),
             "Range:%.0f  Asym:%.1f  SNR:%.0fdB",
             range, asymmetry,
             range > 0 ? 20.0f * log10f(range / (p->noise_floor + 0.01f)) : 0.0f);

    devcal_ui_val2(res, lv_color_hex(DCOL_CYAN));
    devcal_ui_result_ok("Phase 2 complete  " LV_SYMBOL_OK);
}
