/**
 * @file bar_chart.h
 * @brief Live scan vertical bar chart — Gradiometer Pro.
 *
 * Architecture: LV_EVENT_DRAW_MAIN on a plain lv_obj.
 *   - One single full redraw per lv_obj_invalidate() call.
 *   - No lv_chart series overhead, no per-point callbacks.
 *   - ~1 lv_draw_rect per visible bar (skips baseline bars).
 *
 * Color encoding (metal ↑ / void ↓):
 *   METAL: green → yellow → orange → red  (weak → strong)
 *   VOID:  light blue → mid blue → dark blue  (shallow → deep)
 *
 * Integration: call bar_chart_create() inside live_chart container,
 *   then bar_chart_add_value() on every new ProcessedSample_t.
 */

#pragma once

#include "lvgl.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create the bar chart widget inside a parent container.
 *
 * Sizes itself to 100% of parent. The parent should be objects.live_chart.
 * Internally stores deviation floats for accurate color coding.
 *
 * @param parent  Parent lv_obj_t (the live_chart container from EEZ).
 * @return        Pointer to the new widget, or NULL on OOM.
 */
lv_obj_t *bar_chart_create(lv_obj_t *parent);

/**
 * @brief Push a new sample — shifts all bars left by one slot.
 *
 * @param obj       Widget returned by bar_chart_create().
 * @param scaled    output_scaled [0..1024]. 512 = baseline (no target).
 * @param deviation Actual DSP deviation in LSB (float, signed).
 *                  Used for precise color threshold calculation.
 */
void bar_chart_add_value(lv_obj_t *obj, uint16_t scaled, float deviation);

/**
 * @brief Reset all bars to baseline (512). Call on calibration start.
 *
 * @param obj  Widget returned by bar_chart_create().
 */
void bar_chart_clear(lv_obj_t *obj);

#ifdef __cplusplus
}
#endif
