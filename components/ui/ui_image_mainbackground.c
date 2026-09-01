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

#ifndef LV_ATTRIBUTE_IMG_IMG_MAINBACKGROUND
#define LV_ATTRIBUTE_IMG_IMG_MAINBACKGROUND
#endif

const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST LV_ATTRIBUTE_IMG_IMG_MAINBACKGROUND uint8_t img_mainbackground_map[] = {
  0x5a, 0x5a, 0x5a, 0xff, 	/*Color of index 0*/
};

const lv_img_dsc_t img_mainbackground = {
  .header.cf = LV_IMG_CF_INDEXED_8BIT,
  .header.always_zero = 0,
  .header.reserved = 0,
  .header.w = 320,
  .header.h = 240,
  .data_size = 77824,
  .data = img_mainbackground_map,
};