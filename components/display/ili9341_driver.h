#pragma once
#include "lvgl.h"
#include "driver/spi_master.h"

void ili9341_init(void);
void ili9341_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);
spi_device_handle_t ili9341_get_spi(void);
