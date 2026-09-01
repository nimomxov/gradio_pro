/**
 * @file touch_calibration.c
 * @brief Professional 9-point touch calibration — Affine Transform + Least Squares
 *
 * ═══════════════════════════════════════════════════════════════════
 *  ALGORITHM
 * ═══════════════════════════════════════════════════════════════════
 *
 *  9 reference points arranged in 3×3 grid:
 *
 *    [0]──[1]──[2]
 *     │    │    │
 *    [3]──[4]──[5]
 *     │    │    │
 *    [6]──[7]──[8]
 *
 *  For each point, user touches the crosshair.
 *  We collect SAMPLES_PER_POINT readings and median-filter them.
 *
 *  Affine Transform (6 unknowns, 9×2=18 equations → overdetermined):
 *    screen_x = A*raw_x + B*raw_y + C
 *    screen_y = D*raw_x + E*raw_y + F
 *
 *  Solved via Least Squares:
 *    [A B C] = (MtM)^-1 * Mt * screen_x_vec
 *    [D E F] = (MtM)^-1 * Mt * screen_y_vec
 *
 *  where M is the 9×3 matrix [raw_x | raw_y | 1]
 *
 * ═══════════════════════════════════════════════════════════════════
 *  UI DESIGN
 * ═══════════════════════════════════════════════════════════════════
 *
 *  - Dark background (professional feel)
 *  - Crosshair: 20px arms + 4px center dot
 *  - Progress bar at bottom
 *  - Instruction text: "Touch the crosshair" / "Hold steady..."
 *  - Visual feedback: crosshair turns green on successful capture
 *  - Countdown indicator (3 samples shown as dots)
 */

#include "touch_calibration.h"
#include "xpt2046_driver.h"
#include "lvgl.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

static const char *TAG = "TouchCal";

/* ── Configuration ── */
#define LCD_W               320
#define LCD_H               240
#define MARGIN              30          /* Distance from edge for corner points */
#define SAMPLES_PER_POINT   12          /* Raw samples collected per point      */
#define DISCARD_OUTLIERS    3           /* Discard top+bottom N outliers        */
#define TOUCH_HOLD_MS       600         /* Hold time before capture (ms)        */
#define DEBOUNCE_MS         80          /* Min time between samples             */
#define RELEASE_TIMEOUT_MS  2000        /* Max wait for release after capture   */
#define NVS_NAMESPACE       "touch_cal"
#define NVS_KEY             "coeffs_v2"

/* ── Crosshair appearance ── */
#define CH_ARM_LEN          18          /* Length of each arm                   */
#define CH_DOT_R            4           /* Center dot radius                    */
#define CH_LINE_W           2           /* Line width                           */
#define CH_COLOR_IDLE       0x00D4FF    /* Cyan — waiting                       */
#define CH_COLOR_HOLD       0xFFC107    /* Amber — holding                      */
#define CH_COLOR_OK         0x47FF70    /* Green — captured                     */

/* ── Calibration state ── */
static TouchCalCoeffs_t s_coeffs = {0};

/* ═══════════════════════════════════════════════════════════════════
 * 9 reference points — screen coordinates
 * ═══════════════════════════════════════════════════════════════════ */
static const lv_coord_t REF_X[9] = {
    MARGIN,        LCD_W/2,       LCD_W-MARGIN,
    MARGIN,        LCD_W/2,       LCD_W-MARGIN,
    MARGIN,        LCD_W/2,       LCD_W-MARGIN,
};
static const lv_coord_t REF_Y[9] = {
    MARGIN,        MARGIN,        MARGIN,
    LCD_H/2,       LCD_H/2,       LCD_H/2,
    LCD_H-MARGIN,  LCD_H-MARGIN,  LCD_H-MARGIN,
};

/* ═══════════════════════════════════════════════════════════════════
 * MATH — Least Squares Affine Transform
 * ═══════════════════════════════════════════════════════════════════ */

/*
 * Solve 3×3 linear system Ax = b using Gaussian elimination.
 * A is modified in place. Returns false if singular.
 */
static bool solve_3x3(float A[3][3], float b[3], float x[3])
{
    /* Forward elimination */
    for (int col = 0; col < 3; col++) {
        /* Find pivot */
        int pivot = col;
        float max_val = fabsf(A[col][col]);
        for (int row = col + 1; row < 3; row++) {
            if (fabsf(A[row][col]) > max_val) {
                max_val = fabsf(A[row][col]);
                pivot = row;
            }
        }
        if (max_val < 1e-6f) return false;  /* Singular */

        /* Swap rows */
        if (pivot != col) {
            for (int k = 0; k < 3; k++) {
                float tmp = A[col][k]; A[col][k] = A[pivot][k]; A[pivot][k] = tmp;
            }
            float tmp = b[col]; b[col] = b[pivot]; b[pivot] = tmp;
        }

        /* Eliminate below */
        for (int row = col + 1; row < 3; row++) {
            float factor = A[row][col] / A[col][col];
            for (int k = col; k < 3; k++) A[row][k] -= factor * A[col][k];
            b[row] -= factor * b[col];
        }
    }

    /* Back substitution */
    for (int row = 2; row >= 0; row--) {
        x[row] = b[row];
        for (int k = row + 1; k < 3; k++) x[row] -= A[row][k] * x[k];
        x[row] /= A[row][row];
    }
    return true;
}

/*
 * Compute Affine Transform coefficients from 9 point pairs.
 * raw[9] → ref[9] via: out = A*raw_x + B*raw_y + C
 *
 * Normal equations: (M^T M) * [A B C]^T = M^T * ref
 * where M[i] = [raw_x[i], raw_y[i], 1]
 */
static bool compute_affine(
    const float raw_x[9], const float raw_y[9],
    const float ref_x[9], const float ref_y[9],
    float abc[3], float def[3])
{
    /* Build M^T * M (3×3) and M^T * ref_x, M^T * ref_y */
    float MtM[3][3] = {{0}};
    float MtRx[3]   = {0};
    float MtRy[3]   = {0};

    for (int i = 0; i < 9; i++) {
        float row[3] = {raw_x[i], raw_y[i], 1.0f};
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) MtM[r][c] += row[r] * row[c];
            MtRx[r] += row[r] * ref_x[i];
            MtRy[r] += row[r] * ref_y[i];
        }
    }

    /* Solve for X coefficients */
    float MtM_x[3][3];
    memcpy(MtM_x, MtM, sizeof(MtM));
    if (!solve_3x3(MtM_x, MtRx, abc)) return false;

    /* Solve for Y coefficients */
    float MtM_y[3][3];
    memcpy(MtM_y, MtM, sizeof(MtM));
    if (!solve_3x3(MtM_y, MtRy, def)) return false;

    return true;
}

/* ═══════════════════════════════════════════════════════════════════
 * NVS STORAGE
 * ═══════════════════════════════════════════════════════════════════ */

static esp_err_t nvs_save(const TouchCalCoeffs_t *c)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(h, NVS_KEY, c, sizeof(TouchCalCoeffs_t));
    if (err == ESP_OK) nvs_commit(h);
    nvs_close(h);
    return err;
}

static esp_err_t nvs_load(TouchCalCoeffs_t *c)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) return err;
    size_t sz = sizeof(TouchCalCoeffs_t);
    err = nvs_get_blob(h, NVS_KEY, c, &sz);
    nvs_close(h);
    return err;
}

/* ═══════════════════════════════════════════════════════════════════
 * RAW TOUCH SAMPLING
 * ═══════════════════════════════════════════════════════════════════ */

/* Read multiple raw samples and return median-filtered X,Y */
static void sample_raw_median(int32_t *out_x, int32_t *out_y)
{
    int32_t xs[SAMPLES_PER_POINT];
    int32_t ys[SAMPLES_PER_POINT];

    for (int i = 0; i < SAMPLES_PER_POINT; i++) {
        /* Use the raw read function from xpt2046 */
        int32_t rx, ry;
        xpt2046_read_raw_xy(&rx, &ry);
        xs[i] = rx;
        ys[i] = ry;
        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
    }

    /* Simple insertion sort then take median */
    for (int i = 1; i < SAMPLES_PER_POINT; i++) {
        int32_t kx = xs[i], ky = ys[i];
        int j = i - 1;
        while (j >= 0 && xs[j] > kx) { xs[j+1] = xs[j]; ys[j+1] = ys[j]; j--; }
        xs[j+1] = kx; ys[j+1] = ky;
    }

    /* Average of middle values after discarding outliers */
    int64_t sum_x = 0, sum_y = 0;
    int count = SAMPLES_PER_POINT - 2 * DISCARD_OUTLIERS;
    for (int i = DISCARD_OUTLIERS; i < SAMPLES_PER_POINT - DISCARD_OUTLIERS; i++) {
        sum_x += xs[i];
        sum_y += ys[i];
    }
    *out_x = (int32_t)(sum_x / count);
    *out_y = (int32_t)(sum_y / count);
}

/* ═══════════════════════════════════════════════════════════════════
 * UI DRAWING
 * ═══════════════════════════════════════════════════════════════════ */

static lv_obj_t *s_screen    = NULL;

static lv_obj_t *s_lbl_inst  = NULL;
static lv_obj_t *s_lbl_prog  = NULL;
static lv_obj_t *s_bar_prog  = NULL;

/* Draw crosshair at (cx, cy) with given color */
static void draw_crosshair(lv_coord_t cx, lv_coord_t cy, lv_color_t color)
{
    /* Horizontal arm */
    lv_obj_t *h = lv_obj_create(s_screen);
    lv_obj_set_size(h, CH_ARM_LEN * 2, CH_LINE_W);
    lv_obj_set_pos(h, cx - CH_ARM_LEN, cy - CH_LINE_W/2);
    lv_obj_set_style_bg_color(h, color, 0);
    lv_obj_set_style_border_width(h, 0, 0);
    lv_obj_set_style_radius(h, 0, 0);
    lv_obj_clear_flag(h, LV_OBJ_FLAG_CLICKABLE);

    /* Vertical arm */
    lv_obj_t *v = lv_obj_create(s_screen);
    lv_obj_set_size(v, CH_LINE_W, CH_ARM_LEN * 2);
    lv_obj_set_pos(v, cx - CH_LINE_W/2, cy - CH_ARM_LEN);
    lv_obj_set_style_bg_color(v, color, 0);
    lv_obj_set_style_border_width(v, 0, 0);
    lv_obj_set_style_radius(v, 0, 0);
    lv_obj_clear_flag(v, LV_OBJ_FLAG_CLICKABLE);

    /* Center dot */
    lv_obj_t *dot = lv_obj_create(s_screen);
    lv_obj_set_size(dot, CH_DOT_R * 2, CH_DOT_R * 2);
    lv_obj_set_pos(dot, cx - CH_DOT_R, cy - CH_DOT_R);
    lv_obj_set_style_bg_color(dot, color, 0);
    lv_obj_set_style_border_width(dot, 0, 0);
    lv_obj_set_style_radius(dot, CH_DOT_R, 0);
    lv_obj_clear_flag(dot, LV_OBJ_FLAG_CLICKABLE);
}

static void ui_create(void)
{
    s_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_screen, lv_color_hex(0x0A0A0F), 0);
    lv_obj_set_style_bg_opa(s_screen, LV_OPA_COVER, 0);

    /* Title */
    lv_obj_t *title = lv_label_create(s_screen);
    lv_label_set_text(title, "TOUCH CALIBRATION");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 6);

    /* Instruction label */
    s_lbl_inst = lv_label_create(s_screen);
    lv_label_set_text(s_lbl_inst, "Touch the crosshair precisely");
    lv_obj_set_style_text_color(s_lbl_inst, lv_color_hex(0xAAAAAA), 0);
    lv_obj_set_style_text_font(s_lbl_inst, &lv_font_montserrat_12, 0);
    lv_obj_align(s_lbl_inst, LV_ALIGN_BOTTOM_MID, 0, -28);

    /* Progress bar */
    s_bar_prog = lv_bar_create(s_screen);
    lv_obj_set_size(s_bar_prog, 200, 6);
    lv_obj_align(s_bar_prog, LV_ALIGN_BOTTOM_MID, 0, -14);
    lv_bar_set_range(s_bar_prog, 0, 9);
    lv_bar_set_value(s_bar_prog, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_bar_prog, lv_color_hex(0x222233), 0);
    lv_obj_set_style_bg_color(s_bar_prog, lv_color_hex(0x00D4FF),
                              LV_PART_INDICATOR);

    /* Progress text */
    s_lbl_prog = lv_label_create(s_screen);
    lv_label_set_text(s_lbl_prog, "0 / 9");
    lv_obj_set_style_text_color(s_lbl_prog, lv_color_hex(0x666688), 0);
    lv_obj_set_style_text_font(s_lbl_prog, &lv_font_montserrat_12, 0);
    lv_obj_align(s_lbl_prog, LV_ALIGN_BOTTOM_MID, 0, -4);

    lv_scr_load(s_screen);
}

static void ui_show_point(int idx, bool captured)
{
    /* Clear previous crosshair objects (keep title/bar/labels) */
    /* Simple approach: delete children that are clickable-flagged as crosshair */
    /* We redraw fresh each time — screen is small so this is fast */

    /* Remove all crosshair objects by tag */
    uint32_t child_cnt = lv_obj_get_child_cnt(s_screen);
    for (int i = (int)child_cnt - 1; i >= 0; i--) {
        lv_obj_t *child = lv_obj_get_child(s_screen, i);
        if (child == s_lbl_inst || child == s_lbl_prog || child == s_bar_prog)
            continue;
        /* Check if it's a title label */
        if (lv_obj_check_type(child, &lv_label_class)) continue;
        lv_obj_del(child);
    }

    lv_color_t col = captured ? lv_color_hex(CH_COLOR_OK)
                               : lv_color_hex(CH_COLOR_IDLE);
    draw_crosshair(REF_X[idx], REF_Y[idx], col);

    /* Update progress */
    char buf[12];
    snprintf(buf, sizeof(buf), "%d / 9", idx + (captured ? 1 : 0));
    lv_label_set_text(s_lbl_prog, buf);
    lv_bar_set_value(s_bar_prog, idx + (captured ? 1 : 0), LV_ANIM_ON);

    lv_timer_handler();
}

static void ui_set_instruction(const char *text, lv_color_t color)
{
    lv_label_set_text(s_lbl_inst, text);
    lv_obj_set_style_text_color(s_lbl_inst, color, 0);
    lv_timer_handler();
}

static void ui_show_result(bool success)
{
    /* Clear everything */
    lv_obj_clean(s_screen);

    lv_obj_t *icon = lv_label_create(s_screen);
    lv_obj_t *msg  = lv_label_create(s_screen);

    if (success) {
        lv_label_set_text(icon, LV_SYMBOL_OK);
        lv_obj_set_style_text_color(icon, lv_color_hex(0x47FF70), 0);
        lv_label_set_text(msg, "Calibration saved!");
        lv_obj_set_style_text_color(msg, lv_color_hex(0xFFFFFF), 0);
    } else {
        lv_label_set_text(icon, LV_SYMBOL_WARNING);
        lv_obj_set_style_text_color(icon, lv_color_hex(0xFF4444), 0);
        lv_label_set_text(msg, "Calibration failed\nUsing defaults");
        lv_obj_set_style_text_color(msg, lv_color_hex(0xAAAAAA), 0);
    }

    lv_obj_set_style_text_font(icon, &lv_font_montserrat_30, 0);
    lv_obj_align(icon, LV_ALIGN_CENTER, 0, -20);
    lv_obj_set_style_text_font(msg, &lv_font_montserrat_14, 0);
    lv_obj_align(msg, LV_ALIGN_CENTER, 0, 20);

    lv_timer_handler();
    vTaskDelay(pdMS_TO_TICKS(1500));
}

/* ═══════════════════════════════════════════════════════════════════
 * MAIN CALIBRATION ROUTINE
 * ═══════════════════════════════════════════════════════════════════ */

void touch_cal_run(void)
{
    ESP_LOGI(TAG, "Starting 9-point calibration");

    float raw_x[9], raw_y[9];
    float ref_x[9], ref_y[9];

    ui_create();

    for (int i = 0; i < 9; i++) {

        ref_x[i] = (float)REF_X[i];
        ref_y[i] = (float)REF_Y[i];

        /* Show current point */
        ui_show_point(i, false);
        ui_set_instruction("Touch the crosshair precisely",
                           lv_color_hex(0xAAAAAA));

        /* ── Wait for touch ── */
        ESP_LOGI(TAG, "Point %d/%d: waiting touch at (%d,%d)",
                 i+1, 9, REF_X[i], REF_Y[i]);

        /* Wait for finger down */
        int32_t rx = 0, ry = 0;
        bool touched = false;

        while (!touched) {
            xpt2046_read_raw_xy(&rx, &ry);
            touched = xpt2046_is_touched();
            lv_timer_handler();
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        /* Show hold feedback */
        ui_set_instruction("Hold steady...",
                           lv_color_hex(CH_COLOR_HOLD));

        /* Redraw crosshair in amber */
        uint32_t child_cnt = lv_obj_get_child_cnt(s_screen);
        for (int c = (int)child_cnt - 1; c >= 0; c--) {
            lv_obj_t *child = lv_obj_get_child(s_screen, c);
            if (child == s_lbl_inst || child == s_lbl_prog ||
                child == s_bar_prog) continue;
            if (lv_obj_check_type(child, &lv_label_class)) continue;
            lv_obj_del(child);
        }
        draw_crosshair(REF_X[i], REF_Y[i], lv_color_hex(CH_COLOR_HOLD));
        lv_timer_handler();

        /* Wait HOLD time then sample */
        vTaskDelay(pdMS_TO_TICKS(TOUCH_HOLD_MS));

        /* Collect median-filtered sample */
        int32_t sx, sy;
        sample_raw_median(&sx, &sy);
        raw_x[i] = (float)sx;
        raw_y[i] = (float)sy;

        ESP_LOGI(TAG, "  Point %d: raw=(%d,%d) ref=(%d,%d)",
                 i+1, sx, sy, REF_X[i], REF_Y[i]);

        /* Show captured */
        ui_show_point(i, true);
        ui_set_instruction("Good! Next point...",
                           lv_color_hex(CH_COLOR_OK));

        /* Wait for release */
        uint32_t t0 = xTaskGetTickCount();
        while (xpt2046_is_touched() &&
               (xTaskGetTickCount() - t0) < pdMS_TO_TICKS(RELEASE_TIMEOUT_MS)) {
            lv_timer_handler();
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        vTaskDelay(pdMS_TO_TICKS(300));  /* Pause between points */
    }

    /* ── Compute Affine Transform ── */
    float abc[3], def[3];
    bool ok = compute_affine(raw_x, raw_y, ref_x, ref_y, abc, def);

    if (ok) {
        s_coeffs.A = abc[0]; s_coeffs.B = abc[1]; s_coeffs.C = abc[2];
        s_coeffs.D = def[0]; s_coeffs.E = def[1]; s_coeffs.F = def[2];
        s_coeffs.valid = true;

        ESP_LOGI(TAG, "Affine: A=%.4f B=%.4f C=%.4f", abc[0], abc[1], abc[2]);
        ESP_LOGI(TAG, "        D=%.4f E=%.4f F=%.4f", def[0], def[1], def[2]);

        /* Validate: test all 9 points */
        float max_err = 0;
        for (int i = 0; i < 9; i++) {
            float ex = abc[0]*raw_x[i] + abc[1]*raw_y[i] + abc[2] - ref_x[i];
            float ey = def[0]*raw_x[i] + def[1]*raw_y[i] + def[2] - ref_y[i];
            float err = sqrtf(ex*ex + ey*ey);
            if (err > max_err) max_err = err;
        }
        ESP_LOGI(TAG, "Max pixel error: %.2f px", max_err);

        if (max_err > 15.0f) {
            ESP_LOGW(TAG, "High error (%.1f px) — calibration may be inaccurate", max_err);
        }

        /* Save to NVS */
        esp_err_t err = nvs_save(&s_coeffs);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Calibration saved to NVS");
            ui_show_result(true);
        } else {
            ESP_LOGE(TAG, "NVS save failed: %s", esp_err_to_name(err));
            ui_show_result(false);
        }
    } else {
        ESP_LOGE(TAG, "Affine solve failed — singular matrix");
        s_coeffs.valid = false;
        ui_show_result(false);
    }

    lv_obj_del(s_screen);
    s_screen = NULL;
}

/* ═══════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ═══════════════════════════════════════════════════════════════════ */

bool touch_cal_load(void)
{
    TouchCalCoeffs_t tmp = {0};
    esp_err_t err = nvs_load(&tmp);
    if (err == ESP_OK && tmp.valid) {
        s_coeffs = tmp;
        ESP_LOGI(TAG, "Calibration loaded from NVS");
        ESP_LOGI(TAG, "  A=%.4f B=%.4f C=%.4f", tmp.A, tmp.B, tmp.C);
        ESP_LOGI(TAG, "  D=%.4f E=%.4f F=%.4f", tmp.D, tmp.E, tmp.F);
        return true;
    }
    ESP_LOGI(TAG, "No valid calibration in NVS — first run");
    return false;
}

void touch_cal_apply(int32_t raw_x, int32_t raw_y,
                     int32_t *out_x, int32_t *out_y)
{
    if (!s_coeffs.valid) {
        /* Fallback: linear mapping with default constants */
        *out_x = (int32_t)(((float)(raw_x - 200) / 3600.0f) * LCD_W);
        *out_y = (int32_t)(((float)(raw_y - 200) / 3600.0f) * LCD_H);
    } else {
        *out_x = (int32_t)(s_coeffs.A * raw_x + s_coeffs.B * raw_y + s_coeffs.C);
        *out_y = (int32_t)(s_coeffs.D * raw_x + s_coeffs.E * raw_y + s_coeffs.F);
    }

    /* Clamp to screen */
    if (*out_x < 0)       *out_x = 0;
    if (*out_x >= LCD_W)  *out_x = LCD_W - 1;
    if (*out_y < 0)       *out_y = 0;
    if (*out_y >= LCD_H)  *out_y = LCD_H - 1;
}

bool touch_cal_is_valid(void) { return s_coeffs.valid; }

void touch_cal_clear(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, NVS_KEY);
        nvs_commit(h);
        nvs_close(h);
    }
    s_coeffs.valid = false;
    ESP_LOGI(TAG, "Calibration cleared");
}

const TouchCalCoeffs_t *touch_cal_get_coeffs(void) { return &s_coeffs; }
