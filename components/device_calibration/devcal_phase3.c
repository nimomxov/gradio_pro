/**
 * @file devcal_phase3.c
 * @brief Phase 3 — Directional Calibration
 *
 * ═══════════════════════════════════════════════════════════════════
 *  PHASE 3A — Horizontal Compass (Earth Field Compensation)
 * ═══════════════════════════════════════════════════════════════════
 *
 *  المستخدم يحمل العصا أفقياً كالبندقية ويصوّب نحو كل اتجاه.
 *
 *  الفيزياء:
 *    الجهاز أفقي → الحساسان العلوي والسفلي على نفس المستوى
 *    الحقل المغناطيسي للأرض يدخل بزوايا مختلفة حسب الاتجاه
 *
 *    شمال: Bh يدخل موازياً لمحور الحساسين → gradient أقصى
 *    جنوب: Bh معكوس → gradient سالب أقصى
 *    شرق/غرب: Bh عمودي على المحور → gradient أدنى
 *
 *  المعادلات:
 *    sensor_mismatch   = (grad_N + grad_S) / 2   → فرق بين الحساسين
 *    heading_coeff_NS  = (grad_N - grad_S) / 2   → حساسية المحور N-S
 *    heading_coeff_EW  = (grad_E - grad_W) / 2   → حساسية المحور E-W
 *
 *  التصحيح لكل اتجاه مسح:
 *    correction[N] = -(grad_N - sensor_mismatch)
 *    correction[S] = -(grad_S - sensor_mismatch)
 *    correction[E] = -(grad_E - sensor_mismatch)
 *    correction[W] = -(grad_W - sensor_mismatch)
 *
 *  النتيجة: يمكن المسح بأي اتجاه بدون تأثير مغناطيسية الأرض ✓
 *
 * ═══════════════════════════════════════════════════════════════════
 *  PHASE 3B — Vertical Rotation (Device Body Iron)
 * ═══════════════════════════════════════════════════════════════════
 *
 *  الجهاز عمودي يدور → يكشف soft/hard iron في الجسم
 */

#include "devcal_phase3.h"
#include "devcal_phase3c.h"
#include "devcal_common.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "DevCal_P3";

#define P3A_DIR_MS   16000u   /* was 8000 — doubled: 16s per direction */
#define P3B_DIR_MS   16000u   /* was 8000 — doubled: 16s per direction */

/* Direction names */
static const char *DIR_NAME[4]  = {"NORTH", "EAST",  "SOUTH",  "WEST"};
static const char *DIR_ARROW[4] = {"↑ N",   "→ E",   "↓ S",    "← W"};

/* ═══════════════════════════════════════════════════════════════════
 * PHASE 3A — HORIZONTAL COMPASS
 * ═══════════════════════════════════════════════════════════════════ */

static const char *P3A_INST[4] = {
    "Hold rod HORIZONTAL like a rifle\n"
    "Point it toward NORTH\n"
    "Use compass app — geographic North",

    "Rotate 90° clockwise\n"
    "Now pointing EAST\n"
    "Keep rod perfectly horizontal",

    "Rotate another 90° clockwise\n"
    "Now pointing SOUTH\n"
    "Keep rod perfectly horizontal",

    "Rotate another 90° clockwise\n"
    "Now pointing WEST\n"
    "Keep rod perfectly horizontal"
};

static const char *P3A_TIP[4] = {
    "Aim like a rifle at North horizon.\n"
    "Rod must be level — not tilted.\n"
    "SCAN button = start 8s measurement",

    "Same level, now facing East.\n"
    "This measures cross-axis sensitivity.",

    "Opposite to North — field reversed.\n"
    "grad_S should be opposite to grad_N.",

    "Last direction. Almost done!\n"
    "Keep rod level and still."
};

static void run_phase3a(DeviceProfile_t *p)
{
    ESP_LOGI(TAG, "=== Phase 3A: Horizontal Compass ===");

    float horiz[4] = {0};

    /* ── Intro screen ── */
    devcal_ui_set(
        "Phase 3A / 5  —  Horizontal Compass",
        "Earth Field Compensation",
        "Hold rod HORIZONTAL like a rifle.\n"
        "Point toward each cardinal direction.\n"
        "This removes Earth field from measurements.",
        "Goal: scan in ANY direction without\n"
        "magnetic interference from Earth field.\n"
        "Hold SCAN 2s to skip this step."
    );
    devcal_ui_btn_prompt(LV_SYMBOL_PLAY " Press SCAN to start horizontal calibration");

    bool go = devcal_btn_wait(0);
    if (!go) {
        /* Skip — zero all heading corrections */
        memset(p->horiz_grad, 0, sizeof(p->horiz_grad));
        p->sensor_mismatch  = p->asymmetry;  /* use Phase 2 value */
        p->heading_coeff_ns = 0.0f;
        p->heading_coeff_ew = 0.0f;
        memset(p->heading_correction, 0, sizeof(p->heading_correction));
        ESP_LOGI(TAG, "Phase 3A skipped");
        devcal_ui_result_fail("3A skipped — no heading compensation");
        return;
    }

    /* ── Measure each direction ── */
    for (int d = 0; d < 4; d++) {
        char phase_str[32];
        snprintf(phase_str, sizeof(phase_str),
                 "Phase 3A-%s / 5", DIR_NAME[d]);

        devcal_ui_set(phase_str,
                      "Horizontal — Point rod toward direction",
                      P3A_INST[d], P3A_TIP[d]);

        char prompt[48];
        snprintf(prompt, sizeof(prompt),
                 LV_SYMBOL_PLAY " Pointing %s? Press SCAN", DIR_NAME[d]);
        devcal_ui_btn_prompt(prompt);

        bool ok = devcal_btn_wait(0);
        if (!ok) {
            /* Skip remaining — use zeros */
            for (int dd = d; dd < 4; dd++) horiz[dd] = 0.0f;
            break;
        }

        char meas[32];
        snprintf(meas, sizeof(meas), "Measuring %s...", DIR_NAME[d]);
        devcal_ui_set(phase_str, meas,
                      "HOLD STILL — rod horizontal and level",
                      "Do not tilt or rotate during measurement");
        devcal_ui_btn_prompt("");

        float mean, std;
        devcal_collect_timed(P3A_DIR_MS, &mean, &std);
        horiz[d] = mean;

        char vbuf[32], v2buf[48];
        snprintf(vbuf, sizeof(vbuf), "%s  %.0f LSB", DIR_ARROW[d], mean);
        snprintf(v2buf, sizeof(v2buf), "std=%.1f LSB", std);
        devcal_ui_val(vbuf, lv_color_hex(DCOL_CYAN));
        devcal_ui_val2(v2buf, lv_color_hex(DCOL_GREY));

        ESP_LOGI(TAG, "P3A[%s]: mean=%.1f  std=%.2f", DIR_NAME[d], mean, std);

        char res[48];
        snprintf(res, sizeof(res), "%s: %.0f LSB", DIR_NAME[d], mean);
        devcal_ui_result_ok(res);
    }

    /* ── Compute compensation coefficients ── */
    float grad_N = horiz[0];
    float grad_E = horiz[1];
    float grad_S = horiz[2];
    float grad_W = horiz[3];

    float mismatch     = (grad_N + grad_S) / 2.0f;
    float coeff_ns     = (grad_N - grad_S) / 2.0f;
    float coeff_ew     = (grad_E - grad_W) / 2.0f;

    /* Per-direction correction: subtract Earth field component */
    float correction[4];
    correction[0] = -(grad_N - mismatch);  /* North */
    correction[1] = -(grad_E - mismatch);  /* East  */
    correction[2] = -(grad_S - mismatch);  /* South */
    correction[3] = -(grad_W - mismatch);  /* West  */

    /* Store in profile */
    for (int d = 0; d < 4; d++) p->horiz_grad[d] = horiz[d];
    p->sensor_mismatch  = mismatch;
    p->heading_coeff_ns = coeff_ns;
    p->heading_coeff_ew = coeff_ew;
    for (int d = 0; d < 4; d++) p->heading_correction[d] = correction[d];

    ESP_LOGI(TAG, "P3A: mismatch=%.1f  coeff_NS=%.1f  coeff_EW=%.1f",
             mismatch, coeff_ns, coeff_ew);
    ESP_LOGI(TAG, "  Corrections: N=%.1f E=%.1f S=%.1f W=%.1f",
             correction[0], correction[1], correction[2], correction[3]);

    /* Quality assessment */
    float symmetry_error = fabsf(grad_N + grad_S);  /* should be ~0 if symmetric */
    const char *quality =
        (symmetry_error < p->noise_floor * 2.0f) ? "Excellent symmetry" :
        (symmetry_error < p->noise_floor * 5.0f) ? "Good" :
        "Asymmetric — check sensor alignment";

    char res[80];
    snprintf(res, sizeof(res),
             "Mismatch:%.1f  NS:%.1f  EW:%.1f\n%s",
             mismatch, coeff_ns, coeff_ew, quality);
    devcal_ui_result_ok("Phase 3A complete  " LV_SYMBOL_OK);

    ESP_LOGI(TAG, "Phase 3A done — %s", quality);
}

/* ═══════════════════════════════════════════════════════════════════
 * PHASE 3B — VERTICAL ROTATION (unchanged logic)
 * ═══════════════════════════════════════════════════════════════════ */

static const char *P3B_INST[4] = {
    "Hold rod VERTICAL (scanning position)\n"
    "Face geographic NORTH\n"
    "Use compass app on your phone",

    "Rotate 90° CLOCKWISE\n"
    "Now facing EAST\n"
    "Hold rod perfectly VERTICAL",

    "Rotate another 90° CLOCKWISE\n"
    "Now facing SOUTH\n"
    "Hold rod perfectly VERTICAL",

    "Rotate another 90° CLOCKWISE\n"
    "Now facing WEST\n"
    "Hold rod perfectly VERTICAL"
};

static void run_phase3b(DeviceProfile_t *p)
{
    ESP_LOGI(TAG, "=== Phase 3B: Vertical Rotation ===");

    float max_val = -1e9f, min_val = 1e9f;

    devcal_ui_set(
        "Phase 3B / 5  —  Vertical Rotation",
        "Device Body Iron Calibration",
        "Now hold rod VERTICAL (normal scan position).\n"
        "Rotate to face each direction.\n"
        "Measures magnetic distortion from device body.",
        "Hold SCAN 2s to skip this step.\n"
        "Less critical if Phase 3A was completed."
    );
    devcal_ui_btn_prompt(LV_SYMBOL_PLAY " Press SCAN to start vertical calibration");

    bool go = devcal_btn_wait(0);
    if (!go) {
        memset(p->hard_iron, 0, sizeof(p->hard_iron));
        p->dir_variation  = 0.0f;
        p->dir_correction = 0.0f;
        devcal_ui_result_fail("3B skipped — no body iron correction");
        return;
    }

    for (int d = 0; d < 4; d++) {
        char phase_str[24];
        snprintf(phase_str, sizeof(phase_str), "Phase 3B-%s / 5", DIR_NAME[d]);

        devcal_ui_set(phase_str, "Vertical — Facing direction",
                      P3B_INST[d],
                      "Keep rod VERTICAL\n"
                      "Stand still for 8 seconds");

        char prompt[48];
        snprintf(prompt, sizeof(prompt),
                 LV_SYMBOL_PLAY " Facing %s? Press SCAN", DIR_NAME[d]);
        devcal_ui_btn_prompt(prompt);

        bool ok = devcal_btn_wait(0);
        if (!ok) {
            for (int dd = d; dd < 4; dd++) p->hard_iron[dd] = (float)p->nominal_offset;
            break;
        }

        char meas[32];
        snprintf(meas, sizeof(meas), "Measuring %s...", DIR_NAME[d]);
        devcal_ui_set(phase_str, meas,
                      "HOLD STILL — rod vertical", "");
        devcal_ui_btn_prompt("");

        float mean, std;
        devcal_collect_timed(P3B_DIR_MS, &mean, &std);
        p->hard_iron[d] = mean;

        if (mean > max_val) max_val = mean;
        if (mean < min_val) min_val = mean;

        char vbuf[32];
        snprintf(vbuf, sizeof(vbuf), "%s  %.0f", DIR_ARROW[d], mean);
        devcal_ui_val(vbuf, lv_color_hex(DCOL_CYAN));

        char res[32];
        snprintf(res, sizeof(res), "%s: %.0f LSB", DIR_NAME[d], mean);
        devcal_ui_result_ok(res);

        ESP_LOGI(TAG, "P3B[%s]: %.1f", DIR_NAME[d], mean);
    }

    float variation  = max_val - min_val;
    float center     = (max_val + min_val) / 2.0f;
    float correction = (variation > 1.0f) ?
                       (center - (float)p->nominal_offset) / variation : 0.0f;

    p->dir_variation  = variation;
    p->dir_correction = correction;

    ESP_LOGI(TAG, "P3B done: variation=%.1f  correction=%.4f",
             variation, correction);

    devcal_ui_result_ok("Phase 3B complete  " LV_SYMBOL_OK);
}

/* ═══════════════════════════════════════════════════════════════════
 * PUBLIC ENTRY POINT
 * ═══════════════════════════════════════════════════════════════════ */

void devcal_run_phase3(DeviceProfile_t *p)
{
    run_phase3a(p);       /* Horizontal compass — Earth field       */
    run_phase3b(p);       /* Vertical rotation  — body iron         */
    devcal_run_phase3c(p); /* Static sensor matching — 4000 samples */
    p->phases_completed |= (1u << 2u);
}
