/**
 * @file devcal_common.c
 * @brief Shared UI, button, sampling, and NVS for device calibration.
 */

#include "devcal_common.h"
#include "device_cal.h"
#include "adc_task.h"
#include "ads1115_driver.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

static const char *TAG __attribute__((unused)) = "DevCal";

/* FIX: g_lvgl_mutex declared in app_main.c — we must take it before any
 * lv_timer_handler() call from devcal context (ui_event_task, Core 1).
 * lvgl_task on Core 1 also calls lv_timer_handler() with this same mutex.
 * Without it, concurrent access corrupts LVGL internal state. */
extern SemaphoreHandle_t g_lvgl_mutex;

/* Safe LVGL refresh — takes mutex, calls timer handler, releases.
 * All devcal UI functions MUST use this instead of bare lv_timer_handler(). */
static void lvgl_refresh_safe(void)
{
    if (xSemaphoreTake(g_lvgl_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        lvgl_refresh_safe();
        xSemaphoreGive(g_lvgl_mutex);
    }
}

#define NVS_NAMESPACE  "dev_cal"
#define NVS_KEY        "profile_v2"

DcalUI_t g_ui = {0};

/* ═══════════════════════════════════════════════════════════════════
 * UI
 * ═══════════════════════════════════════════════════════════════════ */

void devcal_ui_create(void)
{
    if (g_ui.screen) { lv_obj_del(g_ui.screen); memset(&g_ui, 0, sizeof(g_ui)); }

    g_ui.screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(g_ui.screen, lv_color_hex(DCOL_BG), 0);
    lv_obj_set_style_bg_opa(g_ui.screen, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(g_ui.screen, 0, 0);
    lv_scr_load(g_ui.screen);

    /* Phase badge — top left */
    g_ui.lbl_phase = lv_label_create(g_ui.screen);
    lv_obj_set_style_text_font(g_ui.lbl_phase, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(g_ui.lbl_phase, lv_color_hex(DCOL_GREY), 0);
    lv_obj_align(g_ui.lbl_phase, LV_ALIGN_TOP_LEFT, 6, 4);
    lv_label_set_text(g_ui.lbl_phase, "");

    /* Title */
    g_ui.lbl_title = lv_label_create(g_ui.screen);
    lv_obj_set_style_text_font(g_ui.lbl_title, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(g_ui.lbl_title, lv_color_hex(DCOL_CYAN), 0);
    lv_label_set_long_mode(g_ui.lbl_title, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(g_ui.lbl_title, 308);
    lv_obj_align(g_ui.lbl_title, LV_ALIGN_TOP_MID, 0, 22);
    lv_label_set_text(g_ui.lbl_title, "");

    /* Live value — large center */
    g_ui.lbl_val = lv_label_create(g_ui.screen);
    lv_obj_set_style_text_font(g_ui.lbl_val, &lv_font_montserrat_30, 0);
    lv_obj_set_style_text_color(g_ui.lbl_val, lv_color_hex(DCOL_WHITE), 0);
    lv_obj_align(g_ui.lbl_val, LV_ALIGN_CENTER, 0, -18);
    lv_label_set_text(g_ui.lbl_val, "---");

    /* Second value line */
    g_ui.lbl_val2 = lv_label_create(g_ui.screen);
    lv_obj_set_style_text_font(g_ui.lbl_val2, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(g_ui.lbl_val2, lv_color_hex(DCOL_GREY), 0);
    lv_obj_align(g_ui.lbl_val2, LV_ALIGN_CENTER, 0, 12);
    lv_label_set_text(g_ui.lbl_val2, "");

    /* Instruction */
    g_ui.lbl_inst = lv_label_create(g_ui.screen);
    lv_obj_set_style_text_font(g_ui.lbl_inst, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(g_ui.lbl_inst, lv_color_hex(DCOL_AMBER), 0);
    lv_label_set_long_mode(g_ui.lbl_inst, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(g_ui.lbl_inst, 308);
    lv_obj_align(g_ui.lbl_inst, LV_ALIGN_CENTER, 0, 36);
    lv_label_set_text(g_ui.lbl_inst, "");

    /* Tip */
    g_ui.lbl_tip = lv_label_create(g_ui.screen);
    lv_obj_set_style_text_font(g_ui.lbl_tip, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(g_ui.lbl_tip, lv_color_hex(DCOL_GREY), 0);
    lv_label_set_long_mode(g_ui.lbl_tip, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(g_ui.lbl_tip, 308);
    lv_obj_align(g_ui.lbl_tip, LV_ALIGN_BOTTOM_MID, 0, -38);
    lv_label_set_text(g_ui.lbl_tip, "");

    /* Button prompt */
    g_ui.lbl_btn = lv_label_create(g_ui.screen);
    lv_obj_set_style_text_font(g_ui.lbl_btn, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(g_ui.lbl_btn, lv_color_hex(DCOL_GREEN), 0);
    lv_obj_align(g_ui.lbl_btn, LV_ALIGN_BOTTOM_MID, 0, -22);
    lv_label_set_text(g_ui.lbl_btn, LV_SYMBOL_PLAY " Press SCAN button to start");

    /* Progress bar */
    g_ui.bar_prog = lv_bar_create(g_ui.screen);
    lv_obj_set_size(g_ui.bar_prog, 260, 5);
    lv_obj_align(g_ui.bar_prog, LV_ALIGN_BOTTOM_MID, 0, -12);
    lv_bar_set_range(g_ui.bar_prog, 0, 100);
    lv_bar_set_value(g_ui.bar_prog, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(g_ui.bar_prog, lv_color_hex(DCOL_DARK), 0);
    lv_obj_set_style_bg_color(g_ui.bar_prog, lv_color_hex(DCOL_CYAN),
                              LV_PART_INDICATOR);

    /* Timer */
    g_ui.lbl_timer = lv_label_create(g_ui.screen);
    lv_obj_set_style_text_font(g_ui.lbl_timer, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(g_ui.lbl_timer, lv_color_hex(DCOL_GREY), 0);
    lv_obj_align(g_ui.lbl_timer, LV_ALIGN_BOTTOM_MID, 0, -4);
    lv_label_set_text(g_ui.lbl_timer, "");

    lvgl_refresh_safe();
}

void devcal_ui_set(const char *phase, const char *title,
                   const char *inst,  const char *tip)
{
    lv_label_set_text(g_ui.lbl_phase, phase  ? phase  : "");
    lv_label_set_text(g_ui.lbl_title, title  ? title  : "");
    lv_label_set_text(g_ui.lbl_inst,  inst   ? inst   : "");
    lv_label_set_text(g_ui.lbl_tip,   tip    ? tip    : "");
    lv_label_set_text(g_ui.lbl_val,   "---");
    lv_label_set_text(g_ui.lbl_val2,  "");
    lv_label_set_text(g_ui.lbl_timer, "");
    lv_bar_set_value(g_ui.bar_prog, 0, LV_ANIM_OFF);
    lvgl_refresh_safe();
}

void devcal_ui_val(const char *val, lv_color_t color)
{
    lv_label_set_text(g_ui.lbl_val, val ? val : "");
    lv_obj_set_style_text_color(g_ui.lbl_val, color, 0);
    lvgl_refresh_safe();
}

void devcal_ui_val2(const char *val, lv_color_t color)
{
    lv_label_set_text(g_ui.lbl_val2, val ? val : "");
    lv_obj_set_style_text_color(g_ui.lbl_val2, color, 0);
    lvgl_refresh_safe();
}

void devcal_ui_prog(uint32_t pct, const char *msg)
{
    if (pct > 100) pct = 100;
    lv_bar_set_value(g_ui.bar_prog, (int32_t)pct, LV_ANIM_OFF);
    if (msg) lv_label_set_text(g_ui.lbl_timer, msg);
    lvgl_refresh_safe();
}

void devcal_ui_timer(uint32_t remaining_ms)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%lu s", (unsigned long)(remaining_ms / 1000));
    lv_label_set_text(g_ui.lbl_timer, buf);
    lvgl_refresh_safe();
}

void devcal_ui_btn_prompt(const char *msg)
{
    lv_label_set_text(g_ui.lbl_btn, msg ? msg : "");
    lvgl_refresh_safe();
}

void devcal_ui_refresh(void)
{
    lvgl_refresh_safe();
}

void devcal_ui_result_ok(const char *msg)
{
    lv_label_set_text(g_ui.lbl_val, LV_SYMBOL_OK);
    lv_obj_set_style_text_color(g_ui.lbl_val, lv_color_hex(DCOL_GREEN), 0);
    lv_label_set_text(g_ui.lbl_inst, msg ? msg : "Done");
    lv_obj_set_style_text_color(g_ui.lbl_inst, lv_color_hex(DCOL_GREEN), 0);
    lv_bar_set_value(g_ui.bar_prog, 100, LV_ANIM_ON);
    lv_label_set_text(g_ui.lbl_btn, "");
    lvgl_refresh_safe();
    vTaskDelay(pdMS_TO_TICKS(1800));
}

void devcal_ui_result_fail(const char *msg)
{
    lv_label_set_text(g_ui.lbl_val, LV_SYMBOL_WARNING);
    lv_obj_set_style_text_color(g_ui.lbl_val, lv_color_hex(DCOL_RED), 0);
    lv_label_set_text(g_ui.lbl_inst, msg ? msg : "Using defaults");
    lv_obj_set_style_text_color(g_ui.lbl_inst, lv_color_hex(DCOL_AMBER), 0);
    lvgl_refresh_safe();
    vTaskDelay(pdMS_TO_TICKS(2000));
}

/* ═══════════════════════════════════════════════════════════════════
 * RESULTS SCREEN — paginated, touch navigation, Save/Recalibrate
 * ═══════════════════════════════════════════════════════════════════
 *
 * 5 pages — one per phase:
 *   Page 0: Phase 1 — Offset / Noise / Kalman R
 *   Page 1: Phase 2 — Matching / Range / Asymmetry / Kalman Q
 *   Page 2: Phase 3A — Heading coefficients + corrections
 *   Page 3: Phase 4  — Tilt NS / EW
 *   Page 4: Phase 5  — Detection limit / Kalman Q_final / R_final
 *
 * Navigation:
 *   Right circle btn ">"  → next page
 *   Left  circle btn "<"  → prev page
 *   Dot indicators        → current position
 *
 * Actions (bottom):
 *   [Save]        → write to NVS
 *   [Recalibrate] → erase + restart (GPIO16 long-press equivalent)
 * ═══════════════════════════════════════════════════════════════════ */

#define RS_PAGES      5
#define RS_BTN_R      22        /* circle button radius */
#define RS_DOT_R      5
#define RS_DOT_GAP    14
#define RS_CONTENT_Y  34        /* content area top (below header) */
#define RS_CONTENT_H  158       /* content area height */
#define RS_ACTION_Y   202       /* action buttons Y */

typedef struct {
    lv_obj_t           *scr;
    lv_obj_t           *lbl_page_title;
    lv_obj_t           *lbl_content;
    lv_obj_t           *btn_next;
    lv_obj_t           *btn_prev;
    lv_obj_t           *dots[RS_PAGES];
    lv_obj_t           *btn_save;
    lv_obj_t           *btn_recal;
    lv_obj_t           *lbl_status;
    int                 cur_page;
    const DeviceProfile_t *profile;
    bool                action_done;
    bool                do_recal;
} ResultsScreen_t;

static ResultsScreen_t s_rs;

/* ── Global state for recalibration flag ──
 * FIX: Separated from s_rs because s_rs is zeroed out via memset
 * after the UI is deleted, which previously destroyed the flag
 * before the caller (devcal_run) could read it.
 */
static bool s_wants_recal = false;

/* ── helpers ── */

static void rs_build_page(int page)
{
    const DeviceProfile_t *p = s_rs.profile;
    char buf[380];

    switch (page) {

        case 0:
            lv_label_set_text(s_rs.lbl_page_title,
                              LV_SYMBOL_SETTINGS "  Phase 1 — Sensor Offset");
            snprintf(buf, sizeof(buf),
                "Nominal Offset : %ld LSB\n"
                "Noise Floor    : %.2f LSB\n"
                "Kalman  R      : %.5f\n"
                "Status         : %s",
                (long)p->nominal_offset,
                p->noise_floor,
                p->kalman_R,
                (p->phases_completed & 0x01) ? "OK" : "Skipped");
            break;

        case 1:
            lv_label_set_text(s_rs.lbl_page_title,
                              LV_SYMBOL_SETTINGS "  Phase 2 — Sensor Matching");
            snprintf(buf, sizeof(buf),
                "Gradient  N    : %.1f LSB\n"
                "Gradient  S    : %.1f LSB\n"
                "Dynamic Range  : %.1f LSB\n"
                "Asymmetry      : %.2f LSB\n"
                "Sensitivity    : %.3f\n"
                "Kalman  Q      : %.6f\n"
                "Status         : %s",
                p->gradient_N,
                p->gradient_S,
                p->dynamic_range,
                p->asymmetry,
                p->sensitivity,
                p->kalman_Q,
                (p->phases_completed & 0x02) ? "OK" : "Skipped");
            break;

        case 2:
            lv_label_set_text(s_rs.lbl_page_title,
                              LV_SYMBOL_SETTINGS "  Phase 3 — Compass");
            snprintf(buf, sizeof(buf),
                "Mismatch       : %.2f LSB\n"
                "Coeff N-S      : %.2f\n"
                "Coeff E-W      : %.2f\n"
                "Corr N/E/S/W   :\n"
                "  %.1f / %.1f / %.1f / %.1f\n"
                "Dir Variation  : %.1f LSB\n"
                "Dir Correction : %.4f\n"
                "Status         : %s",
                p->sensor_mismatch,
                p->heading_coeff_ns,
                p->heading_coeff_ew,
                p->heading_correction[0], p->heading_correction[1],
                p->heading_correction[2], p->heading_correction[3],
                p->dir_variation,
                p->dir_correction,
                (p->phases_completed & 0x04) ? "OK" : "Skipped");
            break;

        case 3:
            lv_label_set_text(s_rs.lbl_page_title,
                              LV_SYMBOL_SETTINGS "  Phase 4 — Tilt");
            snprintf(buf, sizeof(buf),
                "Tilt N-S       : %.3f LSB/deg\n"
                "Tilt E-W       : %.3f LSB/deg\n"
                "Threshold      : %.1f deg\n"
                "Status         : %s",
                p->tilt_ns,
                p->tilt_ew,
                p->tilt_threshold_deg,
                (p->phases_completed & 0x08) ? "OK" : "Skipped");
            break;

        case 4:
            lv_label_set_text(s_rs.lbl_page_title,
                              LV_SYMBOL_SETTINGS "  Phase 5 — Final");
            snprintf(buf, sizeof(buf),
                "Detection Lim  : %.1f LSB\n"
                "Kalman Q final : %.6f\n"
                "Kalman R final : %.5f\n"
                "Phases done    : 0x%02X / 0x1F\n"
                "Status         : %s",
                p->detection_limit,
                p->kalman_Q_final,
                p->kalman_R_final,
                (unsigned)p->phases_completed,
                (p->phases_completed & 0x10) ? "OK" : "Skipped");
            break;

        default:
            lv_label_set_text(s_rs.lbl_page_title, "");
            buf[0] = '\0';
            break;
    }

    lv_label_set_text(s_rs.lbl_content, buf);

    /* update dots */
    for (int i = 0; i < RS_PAGES; i++) {
        lv_color_t c = (i == page)
            ? lv_color_hex(DCOL_CYAN)
            : lv_color_hex(DCOL_GREY);
        lv_obj_set_style_bg_color(s_rs.dots[i], c, 0);
    }

    /* hide/show nav buttons */
    if (page == 0)
        lv_obj_add_flag(s_rs.btn_prev, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_clear_flag(s_rs.btn_prev, LV_OBJ_FLAG_HIDDEN);

    if (page == RS_PAGES - 1)
        lv_obj_add_flag(s_rs.btn_next, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_clear_flag(s_rs.btn_next, LV_OBJ_FLAG_HIDDEN);
}

static void rs_cb_next(lv_event_t *e)
{
    (void)e;
    if (s_rs.cur_page < RS_PAGES - 1) {
        s_rs.cur_page++;
        rs_build_page(s_rs.cur_page);
    }
}

static void rs_cb_prev(lv_event_t *e)
{
    (void)e;
    if (s_rs.cur_page > 0) {
        s_rs.cur_page--;
        rs_build_page(s_rs.cur_page);
    }
}

static void rs_cb_save(lv_event_t *e)
{
    (void)e;
    devcal_nvs_save(s_rs.profile);
    lv_label_set_text(s_rs.lbl_status,
                      LV_SYMBOL_OK "  Saved to NVS");
    lv_obj_set_style_text_color(s_rs.lbl_status,
                                lv_color_hex(DCOL_GREEN), 0);
    lv_obj_add_flag(s_rs.btn_save, LV_OBJ_FLAG_HIDDEN);
    s_rs.action_done = true;
}

static void rs_cb_recal(lv_event_t *e)
{
    (void)e;
    s_rs.do_recal    = true;
    s_rs.action_done = true;
}

void devcal_ui_summary(const DeviceProfile_t *p)
{
    /* ── clean up previous calibration screen ── */
    if (g_ui.screen) {
        lv_obj_del(g_ui.screen);
        memset(&g_ui, 0, sizeof(g_ui));
    }

    memset(&s_rs, 0, sizeof(s_rs));
    s_rs.profile  = p;
    s_rs.cur_page = 0;

    /* ── root screen ── */
    lv_obj_t *scr = lv_obj_create(NULL);
    s_rs.scr = scr;
    lv_obj_set_style_bg_color(scr, lv_color_hex(DCOL_BG), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_scr_load(scr);

    /* ── header bar ── */
    lv_obj_t *hdr = lv_obj_create(scr);
    lv_obj_set_size(hdr, 320, 30);
    lv_obj_set_pos(hdr, 0, 0);
    lv_obj_set_style_bg_color(hdr, lv_color_hex(DCOL_DARK), 0);
    lv_obj_set_style_border_width(hdr, 0, 0);
    lv_obj_set_style_radius(hdr, 0, 0);
    lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

    /* global title */
    lv_obj_t *lbl_main = lv_label_create(hdr);
    lv_label_set_text(lbl_main, LV_SYMBOL_OK "  Device Calibration Results");
    lv_obj_set_style_text_color(lbl_main, lv_color_hex(DCOL_GREEN), 0);
    lv_obj_set_style_text_font(lbl_main, &lv_font_montserrat_12, 0);
    lv_obj_align(lbl_main, LV_ALIGN_LEFT_MID, 6, 0);

    /* phases badge */
    char badge[12];
    snprintf(badge, sizeof(badge), "%d/5 OK",
             __builtin_popcount(p->phases_completed & 0x1F));
    lv_obj_t *lbl_badge = lv_label_create(hdr);
    lv_label_set_text(lbl_badge, badge);
    lv_obj_set_style_text_color(lbl_badge, lv_color_hex(DCOL_AMBER), 0);
    lv_obj_set_style_text_font(lbl_badge, &lv_font_montserrat_12, 0);
    lv_obj_align(lbl_badge, LV_ALIGN_RIGHT_MID, -6, 0);

    /* ── page title ── */
    s_rs.lbl_page_title = lv_label_create(scr);
    lv_obj_set_style_text_color(s_rs.lbl_page_title,
                                lv_color_hex(DCOL_CYAN), 0);
    lv_obj_set_style_text_font(s_rs.lbl_page_title,
                               &lv_font_montserrat_12, 0);
    lv_obj_set_pos(s_rs.lbl_page_title, 38, 33);

    /* ── content label ── */
    s_rs.lbl_content = lv_label_create(scr);
    lv_obj_set_style_text_color(s_rs.lbl_content,
                                lv_color_hex(DCOL_WHITE), 0);
    lv_obj_set_style_text_font(s_rs.lbl_content,
                               &lv_font_montserrat_12, 0);
    lv_obj_set_pos(s_rs.lbl_content, 38, 52);
    lv_obj_set_width(s_rs.lbl_content, 244);
    lv_label_set_long_mode(s_rs.lbl_content, LV_LABEL_LONG_WRAP);

    /* ── dot indicators (bottom of content area) ── */
    int dots_total_w = RS_PAGES * RS_DOT_R * 2 + (RS_PAGES - 1) * (RS_DOT_GAP - RS_DOT_R * 2);
    int dot_x_start  = (320 - dots_total_w) / 2;
    for (int i = 0; i < RS_PAGES; i++) {
        lv_obj_t *dot = lv_obj_create(scr);
        lv_obj_set_size(dot, RS_DOT_R * 2, RS_DOT_R * 2);
        lv_obj_set_style_radius(dot, RS_DOT_R, 0);
        lv_obj_set_style_border_width(dot, 0, 0);
        lv_obj_set_style_bg_color(dot, lv_color_hex(DCOL_GREY), 0);
        lv_obj_set_pos(dot, dot_x_start + i * RS_DOT_GAP, 193);
        s_rs.dots[i] = dot;
    }

    /* ── nav button PREV (left, middle height) ── */
    s_rs.btn_prev = lv_btn_create(scr);
    lv_obj_set_size(s_rs.btn_prev, RS_BTN_R * 2, RS_BTN_R * 2);
    lv_obj_set_style_radius(s_rs.btn_prev, RS_BTN_R, 0);
    lv_obj_set_style_bg_color(s_rs.btn_prev, lv_color_hex(DCOL_DARK), 0);
    lv_obj_set_style_border_color(s_rs.btn_prev, lv_color_hex(DCOL_CYAN), 0);
    lv_obj_set_style_border_width(s_rs.btn_prev, 2, 0);
    lv_obj_set_pos(s_rs.btn_prev, 2, 109);   /* left mid */
    lv_obj_t *lp = lv_label_create(s_rs.btn_prev);
    lv_label_set_text(lp, "<");
    lv_obj_set_style_text_color(lp, lv_color_hex(DCOL_CYAN), 0);
    lv_obj_set_style_text_font(lp, &lv_font_montserrat_14, 0);
    lv_obj_align(lp, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(s_rs.btn_prev, rs_cb_prev, LV_EVENT_RELEASED, NULL);

    /* ── nav button NEXT (right, middle height) ── */
    s_rs.btn_next = lv_btn_create(scr);
    lv_obj_set_size(s_rs.btn_next, RS_BTN_R * 2, RS_BTN_R * 2);
    lv_obj_set_style_radius(s_rs.btn_next, RS_BTN_R, 0);
    lv_obj_set_style_bg_color(s_rs.btn_next, lv_color_hex(DCOL_DARK), 0);
    lv_obj_set_style_border_color(s_rs.btn_next, lv_color_hex(DCOL_CYAN), 0);
    lv_obj_set_style_border_width(s_rs.btn_next, 2, 0);
    lv_obj_set_pos(s_rs.btn_next, 296, 109);  /* right mid */
    lv_obj_t *ln = lv_label_create(s_rs.btn_next);
    lv_label_set_text(ln, ">");
    lv_obj_set_style_text_color(ln, lv_color_hex(DCOL_CYAN), 0);
    lv_obj_set_style_text_font(ln, &lv_font_montserrat_14, 0);
    lv_obj_align(ln, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(s_rs.btn_next, rs_cb_next, LV_EVENT_RELEASED, NULL);

    /* ── divider line ── */
    lv_obj_t *div = lv_obj_create(scr);
    lv_obj_set_size(div, 280, 1);
    lv_obj_set_pos(div, 20, 200);
    lv_obj_set_style_bg_color(div, lv_color_hex(DCOL_GREY), 0);
    lv_obj_set_style_border_width(div, 0, 0);

    /* ── action buttons ── */
    /* [Save] */
    s_rs.btn_save = lv_btn_create(scr);
    lv_obj_set_size(s_rs.btn_save, 110, 30);
    lv_obj_set_pos(s_rs.btn_save, 20, RS_ACTION_Y);
    lv_obj_set_style_bg_color(s_rs.btn_save, lv_color_hex(0x1a5c2e), 0);
    lv_obj_set_style_border_color(s_rs.btn_save, lv_color_hex(DCOL_GREEN), 0);
    lv_obj_set_style_border_width(s_rs.btn_save, 1, 0);
    lv_obj_set_style_radius(s_rs.btn_save, 6, 0);
    lv_obj_t *ls = lv_label_create(s_rs.btn_save);
    lv_label_set_text(ls, LV_SYMBOL_SAVE "  Save");
    lv_obj_set_style_text_color(ls, lv_color_hex(DCOL_GREEN), 0);
    lv_obj_set_style_text_font(ls, &lv_font_montserrat_12, 0);
    lv_obj_align(ls, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(s_rs.btn_save, rs_cb_save, LV_EVENT_RELEASED, NULL);

    /* [Recalibrate] */
    s_rs.btn_recal = lv_btn_create(scr);
    lv_obj_set_size(s_rs.btn_recal, 140, 30);
    lv_obj_set_pos(s_rs.btn_recal, 160, RS_ACTION_Y);
    lv_obj_set_style_bg_color(s_rs.btn_recal, lv_color_hex(0x5c1a1a), 0);
    lv_obj_set_style_border_color(s_rs.btn_recal, lv_color_hex(DCOL_RED), 0);
    lv_obj_set_style_border_width(s_rs.btn_recal, 1, 0);
    lv_obj_set_style_radius(s_rs.btn_recal, 6, 0);
    lv_obj_t *lr = lv_label_create(s_rs.btn_recal);
    lv_label_set_text(lr, LV_SYMBOL_REFRESH "  Recalibrate");
    lv_obj_set_style_text_color(lr, lv_color_hex(DCOL_RED), 0);
    lv_obj_set_style_text_font(lr, &lv_font_montserrat_12, 0);
    lv_obj_align(lr, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(s_rs.btn_recal, rs_cb_recal, LV_EVENT_RELEASED, NULL);

    /* ── status label (replaces Save button after save) ── */
    s_rs.lbl_status = lv_label_create(scr);
    lv_label_set_text(s_rs.lbl_status, "");
    lv_obj_set_style_text_font(s_rs.lbl_status, &lv_font_montserrat_12, 0);
    lv_obj_set_pos(s_rs.lbl_status, 20, RS_ACTION_Y + 8);

    /* ── render first page ── */
    rs_build_page(0);
    lvgl_refresh_safe();

    /* ── event loop — wait for user action ── */
    while (!s_rs.action_done) {
        lvgl_refresh_safe();
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    /* small pause so user sees feedback */
    for (int i = 0; i < 50; i++) {
        lvgl_refresh_safe();
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    /* FIX: Save the recalibration request to a dedicated static variable 
     * BEFORE zeroing out the UI struct, otherwise the flag is destroyed
     * and devcal_summary_wants_recal() always returns false. */
    s_wants_recal = s_rs.do_recal;

    lv_obj_del(scr);
    memset(&s_rs, 0, sizeof(s_rs));

    /* if Recalibrate requested — caller (device_cal.c) must handle */
    /* We signal via a global flag checked by devcal_run() */
}

/* Public: returns true if last summary requested recalibration */
bool devcal_summary_wants_recal(void)
{
    return s_wants_recal;
}

/* ═══════════════════════════════════════════════════════════════════
 * BUTTON
 * ═══════════════════════════════════════════════════════════════════ */

void devcal_btn_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << DCAL_BTN_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
}

bool devcal_btn_wait(uint32_t timeout_ms)
{
    uint32_t elapsed = 0;
    uint32_t step    = DCAL_BTN_DEBOUNCE_MS;

    /* Wait for release first (in case already pressed) */
    while (gpio_get_level(DCAL_BTN_GPIO) == 0) {
        lvgl_refresh_safe();
        vTaskDelay(pdMS_TO_TICKS(step));
    }

    /* Wait for press */
    while (gpio_get_level(DCAL_BTN_GPIO) != 0) {
        lvgl_refresh_safe();
        vTaskDelay(pdMS_TO_TICKS(step));
        elapsed += step;
        if (timeout_ms > 0 && elapsed >= timeout_ms) return false;
    }

    /* Measure hold duration */
    uint32_t held = 0;
    while (gpio_get_level(DCAL_BTN_GPIO) == 0) {
        lvgl_refresh_safe();
        vTaskDelay(pdMS_TO_TICKS(step));
        held += step;
        if (held >= DCAL_BTN_LONG_MS) {
            /* Long press = skip */
            ESP_LOGI("DevCal", "Long press — skipping phase");
            return false;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(DCAL_BTN_DEBOUNCE_MS));
    return true;
}

bool devcal_btn_is_long_press(void)
{
    if (gpio_get_level(DCAL_BTN_GPIO) != 0) return false;
    uint32_t held = 0;
    while (gpio_get_level(DCAL_BTN_GPIO) == 0 && held < DCAL_BTN_LONG_MS) {
        vTaskDelay(pdMS_TO_TICKS(20));
        held += 20;
    }
    return (held >= DCAL_BTN_LONG_MS);
}

/* ═══════════════════════════════════════════════════════════════════
 * SAMPLING
 * ═══════════════════════════════════════════════════════════════════ */

int32_t devcal_read_raw(void)
{
    ADS1115Driver_t *drv = adc_task_get_driver();
    if (!drv) return 0;
    int16_t diff = 0;
    ads1115_read_differential(drv, &diff, NULL, NULL);
    return (int32_t)diff;
}

void devcal_collect_stats(uint32_t n_samples,
                           float *out_mean, float *out_std,
                           lv_obj_t *ui_bar)
{
    float sum = 0, sum2 = 0;
    for (uint32_t i = 0; i < n_samples; i++) {
        float v = (float)devcal_read_raw();
        sum  += v;
        sum2 += v * v;
        if (ui_bar && (i % 5 == 0)) {
            lv_bar_set_value(ui_bar,
                             (int32_t)((i + 1) * 100 / n_samples),
                             LV_ANIM_OFF);
            lvgl_refresh_safe();
        }
        vTaskDelay(pdMS_TO_TICKS(DCAL_SAMPLE_MS));
    }
    *out_mean = sum / (float)n_samples;
    float var = sum2 / (float)n_samples - (*out_mean) * (*out_mean);
    *out_std  = sqrtf(var > 0.0f ? var : 0.0f);
}

void devcal_collect_timed(uint32_t duration_ms,
                           float *out_mean, float *out_std)
{
    uint32_t n       = duration_ms / DCAL_SAMPLE_MS;
    float    sum     = 0, sum2 = 0;
    uint32_t t_start = xTaskGetTickCount() * portTICK_PERIOD_MS;

    for (uint32_t i = 0; i < n; i++) {
        /* Check long-press skip */
        if (devcal_btn_is_long_press()) {
            ESP_LOGI("DevCal", "Timed collect skipped by long press");
            break;
        }

        float v = (float)devcal_read_raw();
        sum  += v;
        sum2 += v * v;

        /* Update UI every 5 samples */
        if (i % 5 == 0) {
            uint32_t elapsed = xTaskGetTickCount() * portTICK_PERIOD_MS - t_start;
            uint32_t remain  = (elapsed < duration_ms) ?
                               (duration_ms - elapsed) : 0;

            char vbuf[24];
            float cur_mean = (i > 0) ? sum / (float)(i + 1) : 0;
            snprintf(vbuf, sizeof(vbuf), "%.0f LSB", cur_mean);
            devcal_ui_val(vbuf, lv_color_hex(DCOL_CYAN));

            devcal_ui_prog((i + 1) * 100 / n, "");
            devcal_ui_timer(remain);
        }
        vTaskDelay(pdMS_TO_TICKS(DCAL_SAMPLE_MS));
    }

    uint32_t actual = (n > 0) ? n : 1;
    *out_mean = sum / (float)actual;
    float var = sum2 / (float)actual - (*out_mean) * (*out_mean);
    *out_std  = sqrtf(var > 0.0f ? var : 0.0f);
}

/* ═══════════════════════════════════════════════════════════════════
 * NVS — Device Calibration Storage
 *
 * Namespace : "dev_cal"
 * Keys:
 *   "profile_v2"   → DeviceProfile_t blob
 *   "device_valid" → uint8  (1 = valid)
 *   "cal_version"  → uint8  (DEVCAL_NVS_VERSION)
 *   "dev_crc"      → uint32 (CRC32 of profile blob)
 *
 * Save order (atomic — power-loss safe):
 *   1. write blob   → commit
 *   2. write crc    → commit
 *   3. write version
 *   4. write valid=1 → commit  ← set LAST, boot checks this first
 *
 * Load order:
 *   valid==1 → version match → blob+size → CRC verify → profile.valid
 *
 * Touch calibration lives in "touch_cal" — NEVER touched here.
 * ═══════════════════════════════════════════════════════════════════ */

#define DEVCAL_NVS_VERSION    2u
#define NVS_KEY_VALID         "device_valid"
#define NVS_KEY_VERSION       "cal_version"
#define NVS_KEY_CRC           "dev_crc"

/* ── CRC32 (bitwise, no lookup table — zero extra RAM) ── */
static uint32_t devcal_crc32(const void *data, size_t len)
{
    const uint8_t *buf = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++)
            crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1u));
    }
    return crc ^ 0xFFFFFFFFu;
}

/* ── open namespace, recover on corruption ── */
static esp_err_t devcal_nvs_open(nvs_open_mode_t mode, nvs_handle_t *out_h)
{
    esp_err_t err = nvs_open(NVS_NAMESPACE, mode, out_h);
    if (err == ESP_ERR_NVS_NOT_INITIALIZED) {
        nvs_flash_erase();
        nvs_flash_init();
        err = nvs_open(NVS_NAMESPACE, mode, out_h);
    }
    return err;
}

void devcal_nvs_save(const DeviceProfile_t *p)
{
    nvs_handle_t h;
    if (devcal_nvs_open(NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE("DevCal", "NVS open failed — profile NOT saved");
        return;
    }

    /* ── ATOMIC SAVE — power-loss safe ──────────────────────────
     * valid flag is written LAST.
     * Partial write → valid stays 0 → boot detects → re-calibrate.
     * ────────────────────────────────────────────────────────── */

    /* Step 1: blob + commit */
    esp_err_t err = nvs_set_blob(h, NVS_KEY, p, sizeof(DeviceProfile_t));
    if (err != ESP_OK) {
        ESP_LOGE("DevCal", "nvs_set_blob failed: %s", esp_err_to_name(err));
        nvs_close(h); return;
    }
    if (nvs_commit(h) != ESP_OK) {
        ESP_LOGE("DevCal", "commit(blob) failed");
        nvs_close(h); return;
    }

    /* Step 2: CRC + commit */
    uint32_t crc = devcal_crc32(p, sizeof(DeviceProfile_t));
    nvs_set_u32(h, NVS_KEY_CRC, crc);
    if (nvs_commit(h) != ESP_OK) {
        ESP_LOGE("DevCal", "commit(crc) failed");
        nvs_close(h); return;
    }

    /* Step 3: version (no commit needed — batched with step 4) */
    nvs_set_u8(h, NVS_KEY_VERSION, DEVCAL_NVS_VERSION);

    /* Step 4: valid=1 + final commit — gate that unlocks the profile */
    nvs_set_u8(h, NVS_KEY_VALID, 1u);
    err = nvs_commit(h);
    nvs_close(h);

    if (err == ESP_OK) {
        ESP_LOGI("DevCal", "Profile saved OK — ver=%u  phases=0x%02X  crc=0x%08lX",
                 DEVCAL_NVS_VERSION, p->phases_completed, (unsigned long)crc);
    } else {
        ESP_LOGE("DevCal", "final commit failed: %s", esp_err_to_name(err));
    }
}

bool devcal_nvs_load(DeviceProfile_t *p)
{
    nvs_handle_t h;
    if (devcal_nvs_open(NVS_READONLY, &h) != ESP_OK) return false;

    /* 1. valid flag */
    uint8_t valid = 0;
    if (nvs_get_u8(h, NVS_KEY_VALID, &valid) != ESP_OK || valid != 1u) {
        ESP_LOGI("DevCal", "device_valid=%u — calibration needed", valid);
        nvs_close(h); return false;
    }

    /* 2. version */
    uint8_t ver = 0;
    if (nvs_get_u8(h, NVS_KEY_VERSION, &ver) != ESP_OK || ver != DEVCAL_NVS_VERSION) {
        ESP_LOGW("DevCal", "version mismatch (stored=%u expected=%u) — recalibrate",
                 ver, DEVCAL_NVS_VERSION);
        nvs_close(h); return false;
    }

    /* 3. blob + size check */
    size_t sz = sizeof(DeviceProfile_t);
    esp_err_t err = nvs_get_blob(h, NVS_KEY, p, &sz);
    if (err != ESP_OK) {
        ESP_LOGE("DevCal", "nvs_get_blob failed: %s", esp_err_to_name(err));
        nvs_close(h); return false;
    }
    if (sz != sizeof(DeviceProfile_t)) {
        ESP_LOGE("DevCal", "size mismatch (%u != %u) — corrupt",
                 (unsigned)sz, (unsigned)sizeof(DeviceProfile_t));
        nvs_close(h); return false;
    }

    /* 4. CRC integrity check */
    uint32_t stored_crc = 0;
    if (nvs_get_u32(h, NVS_KEY_CRC, &stored_crc) != ESP_OK) {
        ESP_LOGW("DevCal", "CRC key missing — treating as corrupt");
        nvs_close(h); return false;
    }
    nvs_close(h);

    uint32_t computed_crc = devcal_crc32(p, sizeof(DeviceProfile_t));
    if (computed_crc != stored_crc) {
        ESP_LOGE("DevCal", "CRC MISMATCH stored=0x%08lX computed=0x%08lX — data corrupt!",
                 (unsigned long)stored_crc, (unsigned long)computed_crc);
        return false;
    }

    /* 5. internal struct valid flag */
    if (!p->valid) {
        ESP_LOGW("DevCal", "profile.valid=false — incomplete calibration");
        nvs_close(h); return false;
    }

    ESP_LOGI("DevCal", "Profile loaded OK — phases=0x%02X  det_lim=%.1f  crc=0x%08lX",
             p->phases_completed, p->detection_limit, (unsigned long)computed_crc);
    return true;
}

void devcal_nvs_clear(void)
{
    nvs_handle_t h;
    if (devcal_nvs_open(NVS_READWRITE, &h) != ESP_OK) return;
    nvs_erase_key(h, NVS_KEY);
    nvs_erase_key(h, NVS_KEY_VALID);
    nvs_erase_key(h, NVS_KEY_VERSION);
    nvs_erase_key(h, NVS_KEY_CRC);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI("DevCal", "Device calibration erased — touch calibration preserved");
}

bool devcal_nvs_is_valid(void)
{
    nvs_handle_t h;
    if (devcal_nvs_open(NVS_READONLY, &h) != ESP_OK) return false;
    uint8_t valid = 0, ver = 0;
    bool ok = (nvs_get_u8(h, NVS_KEY_VALID,  &valid) == ESP_OK && valid == 1u) &&
              (nvs_get_u8(h, NVS_KEY_VERSION, &ver)   == ESP_OK && ver   == DEVCAL_NVS_VERSION);
    nvs_close(h);
    ESP_LOGI("DevCal", "NVS check — valid=%u  ver=%u  ok=%d", valid, ver, (int)ok);
    return ok;
}