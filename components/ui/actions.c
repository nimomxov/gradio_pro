/**
 * @file actions.c
 * @brief تسجيل event callbacks على EEZ Studio objects.
 *
 * يُستدعى مرة واحدة من ui_event_task بعد create_screens().
 * كل callback تقرأ state الـ object ثم تستدعي ui_event_task API.
 *
 * ═══════════════════════════════════════════════════════════════════
 *  CALLBACK MAP
 * ═══════════════════════════════════════════════════════════════════
 *
 *  calibration_button        → ui_request_calibration()
 *  manual_scan_button        → ui_request_manual_scan()
 *  auto_scan_button          → يُظهر auto_scan_option_panel
 *  autoscan_start_button     → يقرأ steps/seconds → ui_request_auto_scan()
 *  auto_scan_cancel_button   → ui_cancel_scan()
 *  manual_scan_cancel_button → ui_cancel_scan()
 *  manual_sensibility_switch → ui_set_manual_sensitivity_mode()
 *  sensibility_settings      → ui_set_sensitivity()
 *  logs_button               → يُظهر/يُخفي logs_text
 */

#include "actions.h"
#include "screens.h"
#include "ui_event_task.h"
#include "core/signal_task.h"
#include "ble_logger/ble_data_logger.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "lvgl.h"

#include <stdlib.h>   /* atoi */
#include <string.h>

extern SemaphoreHandle_t g_lvgl_mutex;

/* ── logs toggle state ── */
static bool s_logs_visible = false;

/* ════════════════════════════════════════════════════════════════════
 * HELPERS
 * ════════════════════════════════════════════════════════════════════ */

/** قراءة رقم من textarea بأمان */
static uint8_t read_textarea_uint8(lv_obj_t *ta, uint8_t default_val,
                                   uint8_t min_val, uint8_t max_val)
{
    if (!ta) return default_val;
    const char *txt = lv_textarea_get_text(ta);
    if (!txt || txt[0] == '\0') return default_val;
    int v = atoi(txt);
    if (v < min_val) return min_val;
    if (v > max_val) return max_val;
    return (uint8_t)v;
}

/* ════════════════════════════════════════════════════════════════════
 * CALIBRATE BUTTON
 * ════════════════════════════════════════════════════════════════════ */

static void on_calibration_btn(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_RELEASED) return;
    (void)lv_event_get_user_data(e);
    ui_request_calibration();
}

/* ════════════════════════════════════════════════════════════════════
 * MANUAL SCAN BUTTON
 * ════════════════════════════════════════════════════════════════════ */

static void on_manual_scan_btn(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_RELEASED) return;
    ui_request_manual_scan();
}

/* ════════════════════════════════════════════════════════════════════
 * AUTO SCAN BUTTON → يُظهر لوحة الإعدادات
 * ════════════════════════════════════════════════════════════════════ */

static void on_auto_scan_btn(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_RELEASED) return;
    lv_obj_clear_flag(objects.auto_scan_option_panel, LV_OBJ_FLAG_HIDDEN);
}

/* ════════════════════════════════════════════════════════════════════
 * AUTOSCAN START BUTTON → قراءة steps/seconds + بدء
 * ════════════════════════════════════════════════════════════════════ */

static void on_autoscan_start(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_RELEASED) return;

    uint8_t steps   = read_textarea_uint8(objects.steps_number,   5, 2, 50);
    uint8_t seconds = read_textarea_uint8(objects.seconds_numbers, 2, 1, 10);

    ui_request_auto_scan(steps, seconds);
}

/* ════════════════════════════════════════════════════════════════════
 * CANCEL BUTTONS (Auto + Manual)
 * ════════════════════════════════════════════════════════════════════ */

static void on_cancel_scan(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_RELEASED) return;
    ui_cancel_scan();
}

/* ════════════════════════════════════════════════════════════════════
 * UP/DOWN STEPS & SECONDS
 * ════════════════════════════════════════════════════════════════════ */

static void on_up_steps(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_RELEASED) return;
    uint8_t v = read_textarea_uint8(objects.steps_number, 5, 2, 49);
    char buf[8];
    snprintf(buf, sizeof(buf), "%u", (unsigned)(v + 1));
    lv_textarea_set_text(objects.steps_number, buf);
}

static void on_down_steps(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_RELEASED) return;
    uint8_t v = read_textarea_uint8(objects.steps_number, 5, 3, 50);
    char buf[8];
    snprintf(buf, sizeof(buf), "%u", (unsigned)(v - 1));
    lv_textarea_set_text(objects.steps_number, buf);
}

static void on_up_seconds(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_RELEASED) return;
    uint8_t v = read_textarea_uint8(objects.seconds_numbers, 2, 1, 9);
    char buf[8];
    snprintf(buf, sizeof(buf), "%u", (unsigned)(v + 1));
    lv_textarea_set_text(objects.seconds_numbers, buf);
}

static void on_down_seconds(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_RELEASED) return;
    uint8_t v = read_textarea_uint8(objects.seconds_numbers, 2, 2, 10);
    char buf[8];
    snprintf(buf, sizeof(buf), "%u", (unsigned)(v - 1));
    lv_textarea_set_text(objects.seconds_numbers, buf);
}

/* ════════════════════════════════════════════════════════════════════
 * MANUAL SENSIBILITY SWITCH
 * ════════════════════════════════════════════════════════════════════ */

static void on_manual_sens_switch(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    bool enabled = lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED);
    ui_set_manual_sensitivity_mode(enabled);
}

/* ════════════════════════════════════════════════════════════════════
 * SENSIBILITY DROPDOWN
 * ════════════════════════════════════════════════════════════════════ */

static void on_sens_dropdown(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    lv_obj_t *dd = lv_event_get_target(e);
    uint16_t  idx = lv_dropdown_get_selected(dd);
    ui_set_sensitivity((uint8_t)idx);
}

/* ════════════════════════════════════════════════════════════════════
 * LOGS BUTTON — toggle logs_text visibility
 * ════════════════════════════════════════════════════════════════════ */

static void on_logs_btn(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_RELEASED) return;

    s_logs_visible = !s_logs_visible;
    if (s_logs_visible) {
        lv_obj_clear_flag(objects.logs_text, LV_OBJ_FLAG_HIDDEN);
        /* scroll للآخر */
        lv_textarea_set_cursor_pos(objects.logs_text, LV_TEXTAREA_CURSOR_LAST);
    } else {
        lv_obj_add_flag(objects.logs_text, LV_OBJ_FLAG_HIDDEN);
    }
}

/* ════════════════════════════════════════════════════════════════════
 * REGISTER ALL
 * ════════════════════════════════════════════════════════════════════ */


/* ════════════════════════════════════════════════════════════════════
 * KALMAN FILTER CHECKBOX (Settings tab)
 * ════════════════════════════════════════════════════════════════════ */

static void on_kalman_filter(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    bool enabled = lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED);
    ui_set_kalman(enabled);

    /* If Kalman disabled → uncheck Kalman+Spatial automatically */
    if (!enabled && objects.kalman_spatial) {
        lv_obj_clear_state(objects.kalman_spatial, LV_STATE_CHECKED);
    }
}

/* ════════════════════════════════════════════════════════════════════
 * KALMAN+SPATIAL CHECKBOX (Settings tab — Experimental)
 * Rules:
 *   Checking   → auto-checks kalman_filter too
 *   Unchecking → only disables spatial (Kalman stays)
 * ════════════════════════════════════════════════════════════════════ */

static void on_kalman_spatial(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    bool enabled = lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED);
    ui_set_kalman_spatial(enabled);

    /* Auto-check Kalman when Spatial is enabled */
    if (enabled && objects.kalman_filter) {
        lv_obj_add_state(objects.kalman_filter, LV_STATE_CHECKED);
    }
}

/* ════════════════════════════════════════════════════════════════════
 * SCAN MODE DROPDOWN (3D Scan tab)
 * 0 = RAW        — for expert users & Visualizer analysis
 * 1 = NORMALIZED — noise-normalized, no scale distortion
 * 2 = ENHANCED   — full processing, best visual clarity
 * ════════════════════════════════════════════════════════════════════ */

static void on_scan_mode_dropdown(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    lv_obj_t *dd  = lv_event_get_target(e);
    uint16_t  idx = lv_dropdown_get_selected(dd);
    ui_set_bt_mode((uint8_t)idx);
}


/* ════════════════════════════════════════════════════════════════════
 * SCAN DIRECTIONS DROPDOWN (3D Scan tab)
 *
 * Heading compensation for Earth magnetic field.
 * Makes sensors behave as paired twins (gradient=0 at no target)
 * regardless of scan direction.
 *
 * Mapping:
 *   0=N->S  → apply North correction  (heading=0)
 *   1=S->N  → apply South correction  (heading=2)
 *   2=E->W  → apply East correction   (heading=1)
 *   3=W->E  → apply West correction   (heading=3)
 *   4=Disabled → no compensation      (heading=4)
 *
 * Default: N->S (0) — best performance for most field conditions.
 * ════════════════════════════════════════════════════════════════════ */

static const uint8_t SCAN_DIR_TO_HEADING[5] = {
    0,  /* N->S → North */
    2,  /* S->N → South */
    1,  /* E->W → East  */
    3,  /* W->E → West  */
    4,  /* Disabled      */
};

static void on_scan_directions(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    lv_obj_t *dd  = lv_event_get_target(e);
    uint16_t  idx = lv_dropdown_get_selected(dd);
    if (idx > 4) idx = 4;
    uint8_t heading = SCAN_DIR_TO_HEADING[idx];
    ui_set_scan_heading(heading);
    ui_set_heading_comp(heading < 4);
}


/* ════════════════════════════════════════════════════════════════════
 * BOOST MODE CHECKBOX (Settings tab)
 *
 * OFF: Live=8   Scan=32   samples — normal speed
 * ON:  Live=64  Scan=256  samples — max depth, slower
 *
 * Beep changes: 800Hz/90ms → 1000Hz/150ms
 * Focus: depth and signal clarity over scan speed
 * ════════════════════════════════════════════════════════════════════ */

static void on_boost_mode(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    bool enabled = lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED);
    ui_set_boost_mode(enabled);
}

/* ════════════════════════════════════════════════════════════════════
 * STABILITY GATE / ADAPTIVE SAMPLING (Advanced Settings)
 *
 * يعمل في Auto Scan فقط.
 * default = ON (مُفعَّل من EEZ Studio: lv_obj_add_state CHECKED).
 * ════════════════════════════════════════════════════════════════════ */

static void on_stability_gate(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    bool enabled = lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED);
    ui_set_stability_gate(enabled);
}

/* ════════════════════════════════════════════════════════════════════
 * DATA VIA BLE SWITCH (3D Scan tab)
 * يُفعِّل/يُعطِّل BLE CSV logging للهاتف.
 * HC-05 → OKM لا يتأثر.
 * ════════════════════════════════════════════════════════════════════ */

static void on_data_via_ble(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    bool enabled = lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED);
    ble_logger_set_enabled(enabled);
}

/* ════════════════════════════════════════════════════════════════════
 * RESET DEVICE CALIBRATION BUTTON
 * Shows confirmation dialog → erase NVS → restart
 * Touch calibration is NOT affected.
 * ════════════════════════════════════════════════════════════════════ */

static void on_reset_device_calib(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_RELEASED) return;
    ui_request_reset_device_calib();
}

void actions_register_all(void)
{
    /* calibration_button */
    if (objects.calibration_button)
        lv_obj_add_event_cb(objects.calibration_button,
                            on_calibration_btn, LV_EVENT_ALL, NULL);

    /* manual_scan_button */
    if (objects.manual_scan_button)
        lv_obj_add_event_cb(objects.manual_scan_button,
                            on_manual_scan_btn, LV_EVENT_ALL, NULL);

    /* auto_scan_button */
    if (objects.auto_scan_button)
        lv_obj_add_event_cb(objects.auto_scan_button,
                            on_auto_scan_btn, LV_EVENT_ALL, NULL);

    /* autoscan_start_button */
    if (objects.autoscan_start_button)
        lv_obj_add_event_cb(objects.autoscan_start_button,
                            on_autoscan_start, LV_EVENT_ALL, NULL);

    /* auto_scan_cancel_button */
    if (objects.auto_scan_cancel_button)
        lv_obj_add_event_cb(objects.auto_scan_cancel_button,
                            on_cancel_scan, LV_EVENT_ALL, NULL);

    /* manual_scan_cancel_button */
    if (objects.manual_scan_cancel_button)
        lv_obj_add_event_cb(objects.manual_scan_cancel_button,
                            on_cancel_scan, LV_EVENT_ALL, NULL);

    /* up/down steps */
    if (objects.up_steps)
        lv_obj_add_event_cb(objects.up_steps,
                            on_up_steps, LV_EVENT_ALL, NULL);
    if (objects.down_steps)
        lv_obj_add_event_cb(objects.down_steps,
                            on_down_steps, LV_EVENT_ALL, NULL);

    /* up/down seconds */
    if (objects.up_seconds)
        lv_obj_add_event_cb(objects.up_seconds,
                            on_up_seconds, LV_EVENT_ALL, NULL);
    if (objects.down_seconds)
        lv_obj_add_event_cb(objects.down_seconds,
                            on_down_seconds, LV_EVENT_ALL, NULL);

    /* manual_sensibility_switch */
    if (objects.manual_sensibility_switch)
        lv_obj_add_event_cb(objects.manual_sensibility_switch,
                            on_manual_sens_switch, LV_EVENT_ALL, NULL);

    /* sensibility_settings dropdown */
    if (objects.sensibility_settings)
        lv_obj_add_event_cb(objects.sensibility_settings,
                            on_sens_dropdown, LV_EVENT_ALL, NULL);

    /* logs_button */
    if (objects.logs_button)
        lv_obj_add_event_cb(objects.logs_button,
                            on_logs_btn, LV_EVENT_ALL, NULL);

    /* boost_mode checkbox */
    if (objects.boost_mode)
        lv_obj_add_event_cb(objects.boost_mode,
                            on_boost_mode, LV_EVENT_ALL, NULL);

    /* kalman_filter checkbox */
    if (objects.kalman_filter)
        lv_obj_add_event_cb(objects.kalman_filter,
                            on_kalman_filter, LV_EVENT_ALL, NULL);

    /* kalman_spatial checkbox (Experimental) */
    if (objects.kalman_spatial)
        lv_obj_add_event_cb(objects.kalman_spatial,
                            on_kalman_spatial, LV_EVENT_ALL, NULL);

    /* scan_mode dropdown (3D Scan tab) */
    if (objects.scan_mode)
        lv_obj_add_event_cb(objects.scan_mode,
                            on_scan_mode_dropdown, LV_EVENT_ALL, NULL);

    /* scan_directions dropdown (3D Scan tab) */
    if (objects.scan_directions)
        lv_obj_add_event_cb(objects.scan_directions,
                            on_scan_directions, LV_EVENT_ALL, NULL);

    /* reset_device_calib button (Settings tab) */
    if (objects.reset_device_calib)
        lv_obj_add_event_cb(objects.reset_device_calib,
                            on_reset_device_calib, LV_EVENT_ALL, NULL);

    /* stability_gate checkbox (Advanced Settings — Auto Scan only) */
    if (objects.stability_gate)
        lv_obj_add_event_cb(objects.stability_gate,
                            on_stability_gate, LV_EVENT_ALL, NULL);

    /* data_via_ble switch (3D Scan tab — BLE CSV → mobile) */
    if (objects.data_via_ble)
        lv_obj_add_event_cb(objects.data_via_ble,
                            on_data_via_ble, LV_EVENT_ALL, NULL);

    /* Default: N->S heading enabled */
    ui_set_scan_heading(SCAN_DIR_TO_HEADING[0]);
    ui_set_heading_comp(true);
}