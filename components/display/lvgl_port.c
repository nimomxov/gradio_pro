/**
 * @file lvgl_port.c
 * @brief LVGL v8 port — يُسجِّل ILI9341 و XPT2046 مع LVGL
 *
 * يُستبدل به lvgl_helpers بالكامل.
 * يُستدعى من app_main بعد lv_init().
 */

#include "lvgl_port.h"
#include "ili9341_driver.h"
#include "xpt2046_driver.h"
#include "lvgl.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

static const char *TAG = "LVGLPort";

#define LCD_W        320
#define LCD_H        240
/* BLE stack يحتاج ~70KB DRAM — نخفض لـ single buffer 10 lines.
 * توفير: ~24KB DRAM (من 30KB إلى 6.4KB).
 * الأثر: خط أفتح أحياناً أثناء scroll سريع — مقبول تماماً. */
#define BUF_LINES    10
#define BUF_SIZE     (LCD_W * BUF_LINES)

static lv_disp_draw_buf_t s_draw_buf;
static lv_color_t         s_buf1[BUF_SIZE];
/* Single buffer — s_buf2 محذوف لتوفير الـ DRAM */

static lv_disp_drv_t  s_disp_drv;
static lv_indev_drv_t s_indev_drv;

void lvgl_port_init(void)
{
    /* ── 1. Display init ── */
    ili9341_init();

    /* ── 2. Draw buffer ── */
    lv_disp_draw_buf_init(&s_draw_buf, s_buf1, NULL, BUF_SIZE);

    /* ── 3. Display driver ── */
    lv_disp_drv_init(&s_disp_drv);
    s_disp_drv.hor_res    = LCD_W;
    s_disp_drv.ver_res    = LCD_H;
    s_disp_drv.flush_cb   = ili9341_flush;
    s_disp_drv.draw_buf   = &s_draw_buf;
    s_disp_drv.full_refresh = 0;
    lv_disp_drv_register(&s_disp_drv);

    /* ── 4. Touch init ── */
    xpt2046_init();

    /* ── 5. Input device driver ── */
    lv_indev_drv_init(&s_indev_drv);
    s_indev_drv.type    = LV_INDEV_TYPE_POINTER;
    s_indev_drv.read_cb = xpt2046_read;
    lv_indev_drv_register(&s_indev_drv);

    ESP_LOGI(TAG, "LVGL port ready — %dx%d, buf=%d lines",
             LCD_W, LCD_H, BUF_LINES);
}
