#pragma once
/**
 * @file devcal_common.h
 * @brief Shared types, UI helpers, button control, and statistics
 *        for the 5-phase device calibration system.
 *
 * SCAN BUTTON (GPIO16) — used as the sole control during calibration:
 *   Single press → confirm / proceed to next step
 *   Long press (2s) → skip current phase (use defaults)
 */

#include <stdint.h>
#include <stdbool.h>
#include "lvgl.h"
#include "device_cal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── LCD dimensions ── */
#define DCAL_LCD_W   320
#define DCAL_LCD_H   240

/* ── GPIO scan button ── */
#define DCAL_BTN_GPIO        16
#define DCAL_BTN_LONG_MS     2000u   /* hold 2s = skip phase */
#define DCAL_BTN_DEBOUNCE_MS 50u

/* ── Sampling ── */
#define DCAL_SAMPLE_HZ       20u     /* 20 samples/sec via 860SPS decimation */
#define DCAL_SAMPLE_MS       (1000u / DCAL_SAMPLE_HZ)

/* ── UI Colors ── */
#define DCOL_BG       0x0A0A0F
#define DCOL_WHITE    0xFFFFFF
#define DCOL_CYAN     0x00D4FF
#define DCOL_AMBER    0xFFC107
#define DCOL_GREEN    0x47FF70
#define DCOL_RED      0xFF4444
#define DCOL_GREY     0x666688
#define DCOL_DARK     0x222233
#define DCOL_ORANGE   0xFF8C00

/* ═══════════════════════════════════════════════════════════════════
 * SHARED UI STATE
 * ═══════════════════════════════════════════════════════════════════ */

typedef struct {
    lv_obj_t *screen;
    lv_obj_t *lbl_phase;    /* "Phase 1/5"          */
    lv_obj_t *lbl_title;    /* phase title           */
    lv_obj_t *lbl_inst;     /* instruction text      */
    lv_obj_t *lbl_tip;      /* pro tip               */
    lv_obj_t *lbl_val;      /* live value (big font) */
    lv_obj_t *lbl_val2;     /* second value line     */
    lv_obj_t *lbl_btn;      /* "Press button..."     */
    lv_obj_t *bar_prog;     /* progress bar          */
    lv_obj_t *lbl_prog;     /* progress label        */
    lv_obj_t *lbl_timer;    /* countdown             */
} DcalUI_t;

/* Shared UI instance — initialised by devcal_ui_create() */
extern DcalUI_t g_ui;

/* ═══════════════════════════════════════════════════════════════════
 * UI API
 * ═══════════════════════════════════════════════════════════════════ */

/** Create fresh calibration screen (deletes previous if exists) */
void devcal_ui_create(void);

/** Set phase badge, title, instruction, tip */
void devcal_ui_set(const char *phase, const char *title,
                   const char *inst,  const char *tip);

/** Update live value display */
void devcal_ui_val(const char *val, lv_color_t color);

/** Update second value line */
void devcal_ui_val2(const char *val, lv_color_t color);

/** Update progress bar [0-100] and label */
void devcal_ui_prog(uint32_t pct, const char *msg);

/** Update countdown timer label */
void devcal_ui_timer(uint32_t remaining_ms);

/** Show green OK result then pause 1.5s */
void devcal_ui_result_ok(const char *msg);

/** Show red FAIL result then pause 1.5s */
void devcal_ui_result_fail(const char *msg);

/** Set button prompt text */
void devcal_ui_btn_prompt(const char *msg);

/** Refresh LVGL */
void devcal_ui_refresh(void);

/** Show 5-phase summary and destroy screen */
void devcal_ui_summary(const DeviceProfile_t *p);

/** Returns true if user pressed [Recalibrate] in summary screen */
bool devcal_summary_wants_recal(void);

/* ═══════════════════════════════════════════════════════════════════
 * BUTTON API
 * ═══════════════════════════════════════════════════════════════════ */

/** Initialise GPIO16 input (call once) */
void devcal_btn_init(void);

/**
 * Block until button press or timeout.
 * @param timeout_ms  0 = wait forever
 * @return true = pressed, false = timeout / long-press skip
 */
bool devcal_btn_wait(uint32_t timeout_ms);

/**
 * Check if button is currently held for long-press (skip).
 * Non-blocking — call in sampling loops.
 */
bool devcal_btn_is_long_press(void);

/* ═══════════════════════════════════════════════════════════════════
 * SAMPLING API
 * ═══════════════════════════════════════════════════════════════════ */

/** Read one raw gradient sample from ADS1115 */
int32_t devcal_read_raw(void);

/**
 * Collect N samples, compute mean and std.
 * Updates progress bar if ui_bar != NULL.
 */
void devcal_collect_stats(uint32_t n_samples,
                           float *out_mean, float *out_std,
                           lv_obj_t *ui_bar);

/**
 * Collect samples for duration_ms, compute mean and std.
 * Shows live countdown timer.
 */
void devcal_collect_timed(uint32_t duration_ms,
                           float *out_mean, float *out_std);

/* ═══════════════════════════════════════════════════════════════════
 * NVS API
 * ═══════════════════════════════════════════════════════════════════ */

void devcal_nvs_save(const DeviceProfile_t *p);
bool devcal_nvs_load(DeviceProfile_t *p);
void devcal_nvs_clear(void);

/**
 * @brief Fast validity check without loading full profile.
 * Checks device_valid flag AND version match.
 * Touch calibration is NOT checked — independent.
 */
bool devcal_nvs_is_valid(void);

#ifdef __cplusplus
}
#endif
