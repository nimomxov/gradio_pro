/**
 * @file devcal_phase4.c
 * @brief Phase 4 — Tilt Test (N↔S axis then E↔W axis)
 *
 * PROCEDURE:
 *   Step A (N↔S): Face North. Tilt rod forward (North) ~15°, hold 4s,
 *                 return vertical, tilt backward (South) ~15°, hold 4s,
 *                 return vertical. Total ~12 seconds.
 *
 *   Step B (E↔W): Face North. Tilt rod right (East) ~15°, hold 4s,
 *                 return vertical, tilt left (West) ~15°, hold 4s,
 *                 return vertical. Total ~12 seconds.
 *
 * PHYSICS:
 *   When rod tilts angle θ from vertical, the horizontal Earth field
 *   component Bh enters the sensors:
 *     ΔB ≈ Bh × sin(θ) × (sensor_spacing / some_factor)
 *
 *   tilt_coefficient = peak_deviation / angle_degrees
 *   tilt_threshold   = noise_floor × 3 / tilt_coefficient
 *
 *   This tells us: "at what tilt angle does the signal become
 *   indistinguishable from a real target?"
 *
 * WHY TWO AXES?
 *   N↔S tilt affects Bx component (North field).
 *   E↔W tilt affects By component (East field).
 *   In non-equatorial regions, these are different → 2 coefficients.
 */

#include "devcal_phase4.h"
#include "devcal_common.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "DevCal_P4";

#define P4_STEP_MS        24000u   /* was 12000 — doubled: 24s per axis sweep */
#define P4_TILT_ANGLE_DEG 15.0f   /* Assumed tilt angle */
#define P4_BUF_MAX        300u

static float run_tilt_axis(const char *phase_str,
                            const char *axis_name,
                            const char *inst_text,
                            const char *tip_text)
{
    devcal_ui_set(phase_str, "Tilt Test", inst_text, tip_text);

    char prompt[64];
    snprintf(prompt, sizeof(prompt),
             LV_SYMBOL_PLAY " Ready? Press SCAN to start %s tilt", axis_name);
    devcal_ui_btn_prompt(prompt);

    bool go = devcal_btn_wait(0);
    if (!go) return 0.0f;

    devcal_ui_set(phase_str,
                  "Tilt — Move SLOWLY",
                  "Tilt to angle, hold, return vertical,\n"
                  "tilt other direction, return vertical",
                  "Slow and smooth — total 12 seconds");
    devcal_ui_btn_prompt("");

    /* Collect samples during sweep */
    uint32_t n = P4_STEP_MS / DCAL_SAMPLE_MS;
    if (n > P4_BUF_MAX) n = P4_BUF_MAX;

    float buf[P4_BUF_MAX];
    float sum = 0;

    for (uint32_t i = 0; i < n; i++) {
        buf[i] = (float)devcal_read_raw();
        sum += buf[i];
        if (i % 5 == 0) {
            char vbuf[16];
            snprintf(vbuf, sizeof(vbuf), "%.0f", buf[i]);
            devcal_ui_val(vbuf, lv_color_hex(DCOL_AMBER));
            devcal_ui_prog((i + 1) * 100 / n, "");
            devcal_ui_timer(P4_STEP_MS - i * DCAL_SAMPLE_MS);
        }
        vTaskDelay(pdMS_TO_TICKS(DCAL_SAMPLE_MS));
    }

    /* Mean and peak deviation */
    float mean = sum / (float)n;
    float max_dev = 0.0f;
    for (uint32_t i = 0; i < n; i++) {
        float dev = fabsf(buf[i] - mean);
        if (dev > max_dev) max_dev = dev;
    }

    float coeff = max_dev / P4_TILT_ANGLE_DEG;
    ESP_LOGI(TAG, "Tilt[%s]: peak_dev=%.1f LSB  coeff=%.2f LSB/deg",
             axis_name, max_dev, coeff);

    return coeff;
}

void devcal_run_phase4(DeviceProfile_t *p)
{
    ESP_LOGI(TAG, "=== Phase 4: Tilt Test N-S + E-W ===");

    /* ── Overview screen ── */
    devcal_ui_set(
        "Phase 4 / 5  —  Tilt Test",
        "Tilt Sensitivity Test",

        "This phase measures how the reading\n"
        "changes when the rod is tilted.\n"
        "Two axes: North-South and East-West",

        "Face NORTH throughout both steps.\n"
        "Tilt slowly and smoothly ~15 degrees.\n"
        "Hold 2s to skip this phase"
    );
    devcal_ui_btn_prompt(LV_SYMBOL_PLAY " Press SCAN to begin tilt test");

    bool go = devcal_btn_wait(0);
    if (!go) {
        p->tilt_ns            = 0.0f;
        p->tilt_ew            = 0.0f;
        p->tilt_threshold_deg = 30.0f;
        p->phases_completed  |= (1 << 3);
        devcal_ui_result_fail("Phase 4 skipped — no tilt correction");
        return;
    }

    /* ── Step A: North-South axis ── */
    float coeff_ns = run_tilt_axis(
        "Phase 4A / 5  —  N-S Tilt",
        "North-South",

        "Face NORTH\n"
        "1. Tilt rod FORWARD (North) ~15°, hold 4s\n"
        "2. Return vertical\n"
        "3. Tilt BACKWARD (South) ~15°, hold 4s\n"
        "4. Return vertical",

        "Keep East-West position fixed.\n"
        "Slow and smooth movement.\n"
        "Total time: ~12 seconds"
    );

    char v1[32];
    snprintf(v1, sizeof(v1), "N-S: %.2f LSB/deg", coeff_ns);
    devcal_ui_val(v1, lv_color_hex(DCOL_CYAN));
    vTaskDelay(pdMS_TO_TICKS(1200));

    /* ── Step B: East-West axis ── */
    float coeff_ew = run_tilt_axis(
        "Phase 4B / 5  —  E-W Tilt",
        "East-West",

        "Still facing NORTH\n"
        "1. Tilt rod RIGHT (East) ~15°, hold 4s\n"
        "2. Return vertical\n"
        "3. Tilt LEFT (West) ~15°, hold 4s\n"
        "4. Return vertical",

        "Keep North-South position fixed.\n"
        "Slow and smooth movement.\n"
        "Total time: ~12 seconds"
    );

    /* ── Compute threshold ── */
    float max_coeff = (coeff_ns > coeff_ew) ? coeff_ns : coeff_ew;
    float threshold = (max_coeff > 0.01f) ?
                      (p->noise_floor * 3.0f / max_coeff) : 30.0f;
    if (threshold > 30.0f) threshold = 30.0f;

    p->tilt_ns            = coeff_ns;
    p->tilt_ew            = coeff_ew;
    p->tilt_threshold_deg = threshold;
    p->phases_completed  |= (1 << 3);

    ESP_LOGI(TAG, "P4 done: NS=%.2f  EW=%.2f  threshold=%.1f deg",
             coeff_ns, coeff_ew, threshold);

    char res[80];
    snprintf(res, sizeof(res),
             "N-S:%.2f  E-W:%.2f LSB/°  Limit:%.0f°",
             coeff_ns, coeff_ew, threshold);

    devcal_ui_val2(res, lv_color_hex(DCOL_CYAN));
    devcal_ui_result_ok("Phase 4 complete  " LV_SYMBOL_OK);
}
