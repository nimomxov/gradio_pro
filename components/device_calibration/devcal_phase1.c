/**
 * @file devcal_phase1.c
 * @brief Phase 1 — Sensor Offset + Noise Floor (30 seconds)
 *
 * INSTRUCTIONS displayed on screen:
 *   - Stand device vertical
 *   - Move 3m away from all metal objects
 *   - No walls with reinforced concrete nearby
 *   - Do not walk near device during measurement
 *
 * PHYSICS:
 *   At zero magnetic field perturbation, the differential reading
 *   should be near zero but offset by the Earth's field component
 *   and individual sensor offsets.
 *   → We measure the TRUE baseline and its noise floor.
 *   → R = noise_floor² (measurement noise covariance for Kalman)
 */

#include "devcal_phase1.h"
#include "devcal_common.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>

static const char *TAG = "DevCal_P1";

#define P1_DURATION_MS   60000u   /* was 30000 — doubled for accuracy */
#define P1_WARMUP_MS     10000u   /* was 5000  — doubled for ADC settling */

void devcal_run_phase1(DeviceProfile_t *p)
{
    ESP_LOGI(TAG, "=== Phase 1: Sensor Offset + Noise Floor ===");

    /* ── INSTRUCTION SCREEN ── */
    devcal_ui_set(
        "Phase 1 / 5  —  Sensor Offset",
        "Sensor Offset & Noise Floor",

        "1. Hold rod VERTICAL\n"
        "2. Stand 3m away from walls & metal\n"
        "3. Keep device COMPLETELY STILL",

        "PRO TIP: Avoid reinforced concrete walls,\n"
        "metal pipes, and electrical panels.\n"
        "Hold 2s to skip (use defaults)"
    );
    devcal_ui_btn_prompt(LV_SYMBOL_PLAY " Press SCAN to start 30s measurement");

    bool go = devcal_btn_wait(0);
    if (!go) {
        /* Skipped — use safe defaults */
        p->nominal_offset = 0;
        p->noise_floor    = 5.0f;
        p->kalman_R       = 25.0f;
        p->phases_completed |= (1 << 0);
        devcal_ui_result_fail("Phase 1 skipped — using defaults");
        return;
    }

    /* ── WARMUP (5s, discard samples) ── */
    devcal_ui_set(
        "Phase 1 / 5  —  Warming up",
        "ADC Settling...",
        "Please WAIT — do not move",
        "Allowing sensor to stabilise"
    );
    devcal_ui_btn_prompt("");

    uint32_t warmup_n = P1_WARMUP_MS / DCAL_SAMPLE_MS;
    for (uint32_t i = 0; i < warmup_n; i++) {
        devcal_read_raw();  /* discard */
        devcal_ui_prog(i * 100 / warmup_n, "Warming up...");
        devcal_ui_timer(P1_WARMUP_MS - i * DCAL_SAMPLE_MS);
        vTaskDelay(pdMS_TO_TICKS(DCAL_SAMPLE_MS));
    }

    /* ── MEASUREMENT (30s) ── */
    devcal_ui_set(
        "Phase 1 / 5  —  Measuring",
        "Measuring Noise Floor",
        "KEEP STILL — Do not move or touch\n"
        "Do not walk within 2m of device",
        "Hold 2s to skip"
    );
    devcal_ui_btn_prompt("");

    float mean, std_dev;
    devcal_collect_timed(P1_DURATION_MS, &mean, &std_dev);

    /* ── RESULTS ── */
    p->nominal_offset    = (int32_t)(mean + 0.5f);
    p->noise_floor       = std_dev;
    p->kalman_R          = std_dev * std_dev;
    p->phases_completed |= (1 << 0);

    ESP_LOGI(TAG, "P1 done: offset=%ld  noise=%.3f LSB  R=%.4f",
             (long)p->nominal_offset, p->noise_floor, p->kalman_R);

    char res[64];
    snprintf(res, sizeof(res),
             "Offset: %ld LSB   Noise: %.1f LSB",
             (long)p->nominal_offset, p->noise_floor);

    devcal_ui_val2(res, lv_color_hex(DCOL_CYAN));
    devcal_ui_result_ok("Phase 1 complete  " LV_SYMBOL_OK);
}
