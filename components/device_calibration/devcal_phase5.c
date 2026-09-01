/**
 * @file devcal_phase5.c
 * @brief Phase 5 — Final Noise Floor + Kalman Q/R Optimisation
 *
 * PROCEDURE:
 *   15 seconds of measurement with ALL corrections mentally applied.
 *   Device completely still, same conditions as Phase 1.
 *
 * OUTPUT:
 *   detection_limit = 3 × std_dev  (3-sigma detection criterion)
 *   kalman_R_final  = std_dev²
 *   kalman_Q_final  = refined Q accounting for directional variation
 *
 * KALMAN OPTIMISATION:
 *   If dir_variation is large → process noise Q is increased slightly
 *   to allow the filter to track environmental variation.
 *   If dir_variation is small → Q from Phase 2 is used directly.
 *
 * DETECTION LIMIT DISPLAY:
 *   Tells user: "Minimum detectable target = X LSB"
 *   Contextual message: depth estimate if sensitivity known.
 */

#include "devcal_phase5.h"
#include "devcal_common.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>

static const char *TAG = "DevCal_P5";

#define P5_DURATION_MS   30000u   /* was 15000 — doubled for accuracy */

void devcal_run_phase5(DeviceProfile_t *p)
{
    ESP_LOGI(TAG, "=== Phase 5: Final Noise Floor ===");

    devcal_ui_set(
        "Phase 5 / 5  —  Final Noise",
        "Final Noise Floor Measurement",

        "1. Hold rod VERTICAL\n"
        "2. Move 3m from ALL metal sources\n"
        "3. Absolutely STILL — 15 seconds",

        "PRO TIP: This determines your minimum\n"
        "detectable target depth.\n"
        "Best conditions = best field performance"
    );
    devcal_ui_btn_prompt(LV_SYMBOL_PLAY " Press SCAN to start final 15s measurement");

    bool go = devcal_btn_wait(0);
    if (!go) {
        /* Use Phase 1 values as fallback */
        p->detection_limit = p->noise_floor * 3.0f;
        p->kalman_R_final  = p->kalman_R;
        p->kalman_Q_final  = p->kalman_Q;
        p->phases_completed |= (1 << 4);
        devcal_ui_result_fail("Phase 5 skipped — using Phase 1 noise floor");
        return;
    }

    devcal_ui_set(
        "Phase 5 / 5  —  Measuring",
        "Final Noise Measurement",
        "DO NOT MOVE — 15 seconds\n"
        "Last step of calibration",
        "Almost done!"
    );
    devcal_ui_btn_prompt("");

    float mean, std_dev;
    devcal_collect_timed(P5_DURATION_MS, &mean, &std_dev);

    /* ── Detection limit ── */
    float det_limit = std_dev * 3.0f;  /* 3-sigma criterion */

    /* ── Kalman R final ── */
    float kalman_R_final = std_dev * std_dev;

    /* ── Kalman Q final ── */
    /* Base: from Phase 2 */
    float kalman_Q_final = p->kalman_Q;

    /* Increase Q if directional variation significant */
    if (p->dir_variation > std_dev * 5.0f) {
        float dir_factor = p->dir_variation / (std_dev * 5.0f);
        if (dir_factor > 3.0f) dir_factor = 3.0f;
        kalman_Q_final *= dir_factor;
        ESP_LOGI(TAG, "Q boosted by dir_factor=%.2f", dir_factor);
    }

    /* Increase Q if tilt is significant */
    float tilt_max = (p->tilt_ns > p->tilt_ew) ? p->tilt_ns : p->tilt_ew;
    if (tilt_max > std_dev * 2.0f) {
        kalman_Q_final *= 1.5f;
        ESP_LOGI(TAG, "Q boosted by tilt factor");
    }

    p->detection_limit  = det_limit;
    p->kalman_R_final   = kalman_R_final;
    p->kalman_Q_final   = kalman_Q_final;
    p->phases_completed |= (1 << 4);

    ESP_LOGI(TAG, "P5 done: std=%.2f  det_limit=%.1f  Q=%.6f  R=%.4f",
             std_dev, det_limit, kalman_Q_final, kalman_R_final);

    /* ── Quality feedback ── */
    const char *quality_msg;
    if      (det_limit < 5.0f)   quality_msg = "Excellent sensitivity";
    else if (det_limit < 15.0f)  quality_msg = "Good sensitivity";
    else if (det_limit < 40.0f)  quality_msg = "Average sensitivity";
    else                          quality_msg = "High noise — check environment";

    char vbuf[32], v2buf[64];
    snprintf(vbuf, sizeof(vbuf), "%.1f LSB", det_limit);
    snprintf(v2buf, sizeof(v2buf),
             "Min target: %.0f LSB  —  %s", det_limit, quality_msg);

    devcal_ui_val(vbuf,
        det_limit < 15.0f ? lv_color_hex(DCOL_GREEN) : lv_color_hex(DCOL_AMBER));
    devcal_ui_val2(v2buf, lv_color_hex(DCOL_CYAN));

    /* Show Kalman values */
    char kbuf[48];
    snprintf(kbuf, sizeof(kbuf), "Q=%.5f  R=%.3f", kalman_Q_final, kalman_R_final);
    devcal_ui_val2(kbuf, lv_color_hex(DCOL_GREY));

    devcal_ui_result_ok("Phase 5 complete  " LV_SYMBOL_OK);
}
