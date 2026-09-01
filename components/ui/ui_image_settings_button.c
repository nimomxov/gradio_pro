#ifdef __has_include
    #if __has_include("lvgl.h")
        #ifndef LV_LVGL_H_INCLUDE_SIMPLE
            #define LV_LVGL_H_INCLUDE_SIMPLE
        #endif
    #endif
#endif
#ifdef __has_include
    #if __has_include("lvgl.h")
        #ifndef LV_LVGL_H_INCLUDE_SIMPLE
            #define LV_LVGL_H_INCLUDE_SIMPLE
        #endif
    #endif
#endif

#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
    #include "lvgl.h"
#else
    #include "lvgl/lvgl.h"
#endif

#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif

#ifndef LV_ATTRIBUTE_IMG_IMG_SETTINGS_BUTTON
#define LV_ATTRIBUTE_IMG_IMG_SETTINGS_BUTTON
#endif

const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST LV_ATTRIBUTE_IMG_IMG_SETTINGS_BUTTON uint8_t img_settings_button_map[] = {
  0x00, 0x00, 0x00, 0x00, 	/*Color of index 0*/
 
};

const lv_img_dsc_t img_settings_button = {
  .header.cf = LV_IMG_CF_INDEXED_8BIT,
  .header.always_zero = 0,
  .header.reserved = 0,
  .header.w = 67,
  .header.h = 66,
  .data_size = 5446,
  .data = img_settings_button_map,
};