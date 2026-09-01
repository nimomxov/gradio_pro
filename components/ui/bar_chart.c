/**
 * @file bar_chart.c
 * @brief Live scan vertical bar chart — Gradiometer Pro.
 *
 * RENDERING STRATEGY (LV_EVENT_DRAW_MAIN):
 *   All bars are drawn in a single callback invocation triggered by
 *   lv_obj_invalidate(). LVGL calls this once per dirty-region refresh
 *   cycle — NOT once per bar. This is the most CPU-efficient approach
 *   for a fully custom widget on ESP32 + ILI9341 SPI.
 *
 *   Per refresh: 1 lv_draw_line (baseline) + N lv_draw_rect (visible bars).
 *   Bars at baseline (diff < DEAD_ZONE_PX) are skipped entirely.
 *
 * MEMORY:
 *   bar_chart_data_t is heap-allocated once, freed via LV_EVENT_DELETE
 *   with a proper LVGL-compatible callback (NOT by casting free() directly
 *   which is undefined behaviour — lv_event_cb_t != free's signature).
 *
 * COLOR ENCODING (deviation-based, not output_scaled-based):
 *   Deviation is the raw DSP output in LSB units after calibration.
 *   Using it directly gives meaningful thresholds tied to soil noise floor.
 *
 *   METAL (deviation > 0, bar goes UP):
 *     < 10 LSB  → dark green   #1E6B3E  (near-noise, tread carefully)
 *     < 30 LSB  → green        #2ECC71  (weak metal / deep target)
 *     < 70 LSB  → orange       #E67E22  (clear target)
 *     ≥ 70 LSB  → red          #E74C3C  (strong / close metal)
 *
 *   VOID (deviation < 0, bar goes DOWN):
 *     < 10 LSB  → pale blue    #A9CCE3  (micro-void, soil texture)
 *     < 30 LSB  → light blue   #5DADE2  (cavity / void)
 *     < 70 LSB  → blue         #2E86C1  (chamber / tomb)
 *     ≥ 70 LSB  → deep blue    #1B2A4A  (deep large void)
 */

#include "bar_chart.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "BarChart";

/* ── Chart geometry constants ─────────────────────────────────────────
 * NUM_BARS: 76 bars × (BAR_WIDTH=3 + GAP=1) = 304px < 320px display.
 * BAR_WIDTH=3: thinnest visible bar on 320px wide ILI9341 display.
 * DEAD_ZONE_PX: bars shorter than this are skipped (noise suppression).
 * ───────────────────────────────────────────────────────────────────── */
#define NUM_BARS       76
#define BAR_WIDTH      3
#define BAR_GAP        1
#define SLOT_W         (BAR_WIDTH + BAR_GAP)
#define CENTER_VALUE   512u
#define DEAD_ZONE_PX   3      /* skip bars shorter than 3px */

/* ── Color thresholds (LSB deviation from calibrated baseline) ────── */
#define THR_WEAK       10.0f
#define THR_MED        30.0f
#define THR_STRONG     70.0f

/* ── Per-bar slot: scaled value + raw deviation for color ─────────── */
typedef struct {
    uint16_t scaled;     /* output_scaled [0..1024], 512=baseline */
    float    deviation;  /* DSP deviation in LSB — for color logic */
} BarSlot_t;

/* ── Widget private data (heap-allocated, freed on LV_EVENT_DELETE) ── */
typedef struct {
    BarSlot_t slots[NUM_BARS];
} BarChartData_t;

/* ─────────────────────────────────────────────────────────────────────
 * Color lookup — deviation-based
 * ───────────────────────────────────────────────────────────────────── */
static lv_color_t bar_color(float dev)
{
    float a = dev >= 0.0f ? dev : -dev;

    if (dev > 0.0f) {
        /* METAL — bars going UP */
        if      (a < THR_WEAK)   return lv_color_hex(0x1E6B3Eu);  /* dark green  */
        else if (a < THR_MED)    return lv_color_hex(0x2ECC71u);  /* green       */
        else if (a < THR_STRONG) return lv_color_hex(0xE67E22u);  /* orange      */
        else                     return lv_color_hex(0xE74C3Cu);  /* red         */
    } else {
        /* VOID — bars going DOWN */
        if      (a < THR_WEAK)   return lv_color_hex(0xA9CCE3u);  /* pale blue   */
        else if (a < THR_MED)    return lv_color_hex(0x5DADE2u);  /* light blue  */
        else if (a < THR_STRONG) return lv_color_hex(0x2E86C1u);  /* blue        */
        else                     return lv_color_hex(0x1B2A4Au);  /* deep blue   */
    }
}

/* ─────────────────────────────────────────────────────────────────────
 * LV_EVENT_DELETE — proper LVGL-compatible cleanup callback
 *
 * FIX vs GLM code: GLM casts free() directly to lv_event_cb_t.
 *   free() signature: void free(void *ptr)
 *   lv_event_cb_t:   void cb(lv_event_t *e)
 * These are NOT the same — calling free(lv_event_t*) frees the EVENT
 * STRUCT, not our data. Immediate heap corruption and crash.
 * Correct pattern: extract user_data from event, then free it.
 * ───────────────────────────────────────────────────────────────────── */
static void bar_chart_delete_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    BarChartData_t *data = (BarChartData_t *)lv_obj_get_user_data(obj);
    if (data) {
        free(data);
        lv_obj_set_user_data(obj, NULL);
        ESP_LOGD(TAG, "BarChart data freed");
    }
}

/* ─────────────────────────────────────────────────────────────────────
 * LV_EVENT_DRAW_MAIN — single callback draws all bars
 *
 * Called once per LVGL refresh cycle when the widget is dirty.
 * Draws: dashed baseline + N visible bars (skips zero-height bars).
 * ───────────────────────────────────────────────────────────────────── */
static void bar_chart_draw_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    BarChartData_t *data = (BarChartData_t *)lv_obj_get_user_data(obj);
    if (!data) return;

    lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);

    /* Widget bounds */
    lv_area_t coords;
    lv_obj_get_coords(obj, &coords);
    lv_coord_t w = coords.x2 - coords.x1;
    lv_coord_t h = coords.y2 - coords.y1;
    if (w < 10 || h < 10) return;

    /* Scale: maps [0..1024] → [coords.y2..coords.y1] (inverted Y) */
    lv_coord_t y_base = coords.y1 + h / 2;          /* 512 maps here */
    float scale       = (float)h / 1024.0f;         /* px per unit   */

    /* ── 1. Dashed baseline ──────────────────────────────────────────
     * Subtle horizontal reference line at y_base (value=512).
     * Dashed to not interfere visually with short bars near zero.
     * ──────────────────────────────────────────────────────────────── */
    {
        lv_draw_line_dsc_t ld;
        lv_draw_line_dsc_init(&ld);
        ld.color      = lv_color_hex(0x3A4A5Au);
        ld.width      = 1;
        ld.dash_width = 5;
        ld.dash_gap   = 5;
        ld.opa        = LV_OPA_80;

        lv_point_t p1 = { .x = coords.x1,     .y = y_base };
        lv_point_t p2 = { .x = coords.x1 + w, .y = y_base };
        lv_draw_line(draw_ctx, &ld, &p1, &p2);
    }

    /* ── 2. Bars ─────────────────────────────────────────────────────
     * One lv_draw_rect per visible bar. Rounded tip only on open end.
     * ──────────────────────────────────────────────────────────────── */
    lv_draw_rect_dsc_t rd;
    lv_draw_rect_dsc_init(&rd);
    rd.bg_opa = LV_OPA_COVER;
    rd.radius = 2;

    for (int i = 0; i < NUM_BARS; i++) {
        BarSlot_t *sl = &data->slots[i];

        /* Bar height in pixels */
        lv_coord_t y_val = (lv_coord_t)(coords.y1 + (1024 - sl->scaled) * scale);
        lv_coord_t bar_h = y_base - y_val;   /* positive = metal (up) */

        if (bar_h > -DEAD_ZONE_PX && bar_h < DEAD_ZONE_PX) continue;

        lv_coord_t x1 = coords.x1 + i * SLOT_W;
        lv_coord_t x2 = x1 + BAR_WIDTH - 1;

        lv_area_t ba = {
            .x1 = x1,
            .x2 = x2,
            .y1 = (bar_h > 0) ? y_val    : y_base,
            .y2 = (bar_h > 0) ? y_base   : y_val,
        };

        /* Clamp to widget bounds (prevents overdraw outside container) */
        if (ba.y1 < coords.y1) ba.y1 = coords.y1;
        if (ba.y2 > coords.y2) ba.y2 = coords.y2;
        if (ba.y2 <= ba.y1) continue;   /* fully clipped */

        rd.bg_color = bar_color(sl->deviation);
        lv_draw_rect(draw_ctx, &rd, &ba);
    }
}

/* ─────────────────────────────────────────────────────────────────────
 * PUBLIC API
 * ───────────────────────────────────────────────────────────────────── */

lv_obj_t *bar_chart_create(lv_obj_t *parent)
{
    if (!parent) return NULL;

    /* Allocate private data first — fail early if OOM */
    BarChartData_t *data = (BarChartData_t *)calloc(1, sizeof(BarChartData_t));
    if (!data) {
        ESP_LOGE(TAG, "OOM — bar chart data");
        return NULL;
    }

    /* Fill slots with baseline (no target) */
    for (int i = 0; i < NUM_BARS; i++) {
        data->slots[i].scaled    = CENTER_VALUE;
        data->slots[i].deviation = 0.0f;
    }

    /* Create plain lv_obj — no lv_chart overhead */
    lv_obj_t *obj = lv_obj_create(parent);
    if (!obj) { free(data); return NULL; }

    lv_obj_remove_style_all(obj);

    /* Size: fill parent exactly */
    lv_obj_set_size(obj, lv_pct(100), lv_pct(100));
    lv_obj_set_pos(obj, 0, 0);

    /* Dark background matching the device theme */
    lv_obj_set_style_bg_color (obj, lv_color_hex(0x060A10u), LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa   (obj, LV_OPA_COVER,            LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(obj, LV_OPA_TRANSP,          LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all   (obj, 0,                      LV_STATE_DEFAULT);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_user_data(obj, data);

    /* Register draw and delete callbacks */
    lv_obj_add_event_cb(obj, bar_chart_draw_cb,   LV_EVENT_DRAW_MAIN, NULL);
    lv_obj_add_event_cb(obj, bar_chart_delete_cb, LV_EVENT_DELETE,    NULL);

    ESP_LOGI(TAG, "BarChart created — %d bars × %dpx, parent %dx%d",
             NUM_BARS, BAR_WIDTH,
             (int)lv_obj_get_width(parent),
             (int)lv_obj_get_height(parent));
    return obj;
}

void bar_chart_add_value(lv_obj_t *obj, uint16_t scaled, float deviation)
{
    if (!obj) return;
    BarChartData_t *data = (BarChartData_t *)lv_obj_get_user_data(obj);
    if (!data) return;

    /* Clamp scaled to valid range */
    if (scaled > 1024u) scaled = 1024u;

    /* Shift all slots left by one (oldest drops off, newest enters right) */
    memmove(&data->slots[0], &data->slots[1],
            (NUM_BARS - 1) * sizeof(BarSlot_t));

    data->slots[NUM_BARS - 1].scaled    = scaled;
    data->slots[NUM_BARS - 1].deviation = deviation;

    /* Mark dirty — triggers LV_EVENT_DRAW_MAIN on next LVGL refresh */
    lv_obj_invalidate(obj);
}

void bar_chart_clear(lv_obj_t *obj)
{
    if (!obj) return;
    BarChartData_t *data = (BarChartData_t *)lv_obj_get_user_data(obj);
    if (!data) return;

    for (int i = 0; i < NUM_BARS; i++) {
        data->slots[i].scaled    = CENTER_VALUE;
        data->slots[i].deviation = 0.0f;
    }
    lv_obj_invalidate(obj);
}
