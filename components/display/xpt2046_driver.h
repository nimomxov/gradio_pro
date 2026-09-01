#pragma once
/**
 * @file xpt2046_driver.h
 * @brief XPT2046 touch controller driver
 */

#include <stdbool.h>
#include <stdint.h>
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

void xpt2046_init(void);
void xpt2046_read(lv_indev_drv_t *drv, lv_indev_data_t *data);

/* For calibration system */
void xpt2046_read_raw_xy(int32_t *out_x, int32_t *out_y);
bool xpt2046_is_touched(void);

#ifdef __cplusplus
}
#endif
