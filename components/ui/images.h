#ifndef EEZ_LVGL_UI_IMAGES_H
#define EEZ_LVGL_UI_IMAGES_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

extern const lv_img_dsc_t img_mainbackground;
extern const lv_img_dsc_t img_live_scan_button;
extern const lv_img_dsc_t img_scan3d_button;
extern const lv_img_dsc_t img_settings_button;
extern const lv_img_dsc_t img_background1;
extern const lv_img_dsc_t img_scan3d_photo;
extern const lv_img_dsc_t img_battry;
extern const lv_img_dsc_t img_bluetooth_idle;
extern const lv_img_dsc_t img_up_arrow;
extern const lv_img_dsc_t img_down_steps;
extern const lv_img_dsc_t img_bluetooth_failed;
extern const lv_img_dsc_t img_bluetooth_success;
extern const lv_img_dsc_t img_low_battery;
extern const lv_img_dsc_t img_boot_screen;

#ifndef EXT_IMG_DESC_T
#define EXT_IMG_DESC_T
typedef struct _ext_img_desc_t {
    const char *name;
    const lv_img_dsc_t *img_dsc;
} ext_img_desc_t;
#endif

extern const ext_img_desc_t images[14];

#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_IMAGES_H*/