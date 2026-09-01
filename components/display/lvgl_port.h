#pragma once

/**
 * يُستدعى من app_main بعد lv_init() مباشرة.
 * يُهيِّئ ILI9341 + XPT2046 ويُسجِّلهما مع LVGL.
 */
void lvgl_port_init(void);
