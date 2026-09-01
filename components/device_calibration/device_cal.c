/**
 * @file device_cal.c
 * @brief Device calibration orchestrator — 5-phase system.
 *
 * Calls phases in order:
 *   Phase 1 → Phase 2 → Phase 3 → Phase 4 → Phase 5
 *   → Save to NVS → Show summary → Wait for button → Return
 *
 * Each phase can be skipped by long-pressing the SCAN button (GPIO16).
 * Skipped phases use safe defaults.
 */

#include "device_cal.h"
#include "devcal_common.h"
#include "devcal_phase1.h"
#include "devcal_phase2.h"
#include "devcal_phase3.h"
#include "devcal_phase4.h"
#include "devcal_phase5.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <string.h>
#include <math.h>

static const char *TAG = "DevCal";

static DeviceProfile_t s_profile = {0};

/* ═══════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ═══════════════════════════════════════════════════════════════════ */

bool devcal_load(void)
{
    DeviceProfile_t tmp = {0};
    if (devcal_nvs_load(&tmp)) {
        s_profile = tmp;
        ESP_LOGI(TAG, "Profile loaded: phases=0x%02X  det=%.1f LSB  Q=%.5f  R=%.4f",
                 s_profile.phases_completed,
                 s_profile.detection_limit,
                 s_profile.kalman_Q_final,
                 s_profile.kalman_R_final);
        return true;
    }
    ESP_LOGI(TAG, "No valid device profile in NVS");
    return false;
}

void devcal_run(void)
{
    ESP_LOGI(TAG, "Starting 5-phase device calibration");
    memset(&s_profile, 0, sizeof(s_profile));

    devcal_btn_init();
    devcal_ui_create();

    /* ── Welcome screen ── */
    devcal_ui_set(
        "Device Calibration",
        "5-Phase Sensor Calibration",

        "This wizard calibrates your gradiometer\n"
        "for maximum accuracy and sensitivity.\n"
        "Each phase takes 1-2 minutes.",

        "You will need:\n"
        "  - A bar magnet\n"
        "  - A compass app on your phone\n"
        "  - Open area away from metal\n"
        "SCAN button = confirm / advance"
    );
    devcal_ui_btn_prompt(LV_SYMBOL_PLAY " Press SCAN to begin  (hold 2s = skip phase)");
    devcal_btn_wait(0);

    /* ── Run all phases ── */
    devcal_run_phase1(&s_profile);
    devcal_run_phase2(&s_profile);
    devcal_run_phase3(&s_profile);
    devcal_run_phase4(&s_profile);
    devcal_run_phase5(&s_profile);

    /* ── Finalise ── */
    s_profile.calibration_epoch = (uint32_t)xTaskGetTickCount();
    s_profile.valid             = (s_profile.phases_completed & 0x03) == 0x03;
    /* Valid if at least Phase 1 and Phase 2 completed */

    devcal_nvs_save(&s_profile);

    ESP_LOGI(TAG, "Calibration complete. phases=0x%02X  valid=%d",
             s_profile.phases_completed, s_profile.valid);

    /* ── Summary screen — loop if user requests recalibration ── */
    do {
        devcal_ui_summary(&s_profile);
        if (devcal_summary_wants_recal()) {
            ESP_LOGI(TAG, "Recalibrate requested — restarting");
            devcal_nvs_clear();
            memset(&s_profile, 0, sizeof(s_profile));
            devcal_run();   /* recursive — user chose to redo */
            return;
        }
    } while (false);
}

bool devcal_is_valid(void)
{
    return s_profile.valid;
}

DevCalResult_t devcal_get_result(void)
{
    DevCalResult_t r = {0};
    if (s_profile.valid) {
        r.kalman_Q        = (s_profile.kalman_Q_final > 0.0f) ?
                             s_profile.kalman_Q_final : s_profile.kalman_Q;
        r.kalman_R        = (s_profile.kalman_R_final > 0.0f) ?
                             s_profile.kalman_R_final : s_profile.kalman_R;
        r.nominal_offset  = s_profile.nominal_offset;
        r.asymmetry       = s_profile.asymmetry;
        r.detection_limit = s_profile.detection_limit;
        r.valid           = true;
    } else {
        /* Safe defaults — device works but unoptimised */
        r.kalman_Q  = 0.001f;
        r.kalman_R  = 1.0f;
        r.valid     = false;
    }
    return r;
}

const DeviceProfile_t *devcal_get_profile(void)
{
    return &s_profile;
}

void devcal_clear(void)
{
    devcal_nvs_clear();
    memset(&s_profile, 0, sizeof(s_profile));
    ESP_LOGI(TAG, "Device profile cleared");
}

int32_t devcal_apply_sensor_match(int32_t raw)
{
    /* ── Phase 3C correction ─────────────────────────────────────────
     * corrected = raw_diff + match_addend
     *
     * match_addend is precomputed once after Phase 3C:
     *   addend = (1 - gain2) × ain1_dc + offset2 × gain2
     *
     * This applies:
     *   a) offset2: removes static DC mismatch between sensors (exact)
     *   b) gain2:   corrects sensitivity ratio (< 2% residual error)
     *
     * Fallback logic (ChatGPT/DeepSeek recommendation):
     *   Phase 3C NOT run (gain2 == 0.0 or 1.0 exactly, p3c_samples==0):
     *     → apply offset2 only (from Phase 1 nominal_offset)
     *     → never apply an uncalibrated gain (could worsen signal)
     * ─────────────────────────────────────────────────────────────── */
    if (!s_profile.valid) return raw;

    bool phase3c_done = (s_profile.p3c_samples >= 100u) &&
                        (s_profile.sensor_gain2 > 0.1f) &&
                        (s_profile.sensor_gain2 < 2.0f);

    if (!phase3c_done) {
        /* Fallback: offset only (Phase 1 nominal_offset), gain=1 */
        int32_t corrected = raw - s_profile.nominal_offset;
        return corrected;
    }

    /* Full Phase 3C correction using precomputed addend (O(1)) */
    float corrected = (float)raw + s_profile.sensor_match_addend;

    /* Clamp to int16 range */
    if      (corrected >  32767.0f) corrected =  32767.0f;
    else if (corrected < -32768.0f) corrected = -32768.0f;

    return (int32_t)(corrected + (corrected >= 0.0f ? 0.5f : -0.5f));
}

int32_t devcal_apply_asymmetry(int32_t raw)
{
    if (!s_profile.valid) return raw;
    return raw - (int32_t)(s_profile.asymmetry + 0.5f);
}

int32_t devcal_apply_direction(int32_t raw, uint8_t heading)
{
    if (!s_profile.valid || heading > 3) return raw;
    if (s_profile.dir_variation < 1.0f)  return raw;
    float correction = (s_profile.hard_iron[heading] -
                        (float)s_profile.nominal_offset)
                       * s_profile.dir_correction;
    return raw - (int32_t)(correction + 0.5f);
}

int32_t devcal_apply_heading_compensation(int32_t raw, uint8_t heading)
{
    /* Phase 3A: Remove Earth field component for this scan direction.
     * Apply when user scans in a known direction.
     * heading: 0=North 1=East 2=South 3=West */
    if (!s_profile.valid || heading > 3) return raw;

    float correction = s_profile.heading_correction[heading];
    int32_t result   = raw + (int32_t)(correction + 0.5f);

    /* Also subtract sensor_mismatch (physical sensor offset diff) */
    result -= (int32_t)(s_profile.sensor_mismatch + 0.5f);

    return result;
}

const float *devcal_get_heading_corrections(void)
{
    return s_profile.heading_correction;
}
