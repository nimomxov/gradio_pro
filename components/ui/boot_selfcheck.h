/**
 * @file boot_selfcheck.h
 * @brief Self-check system — يُشغَّل عند الإقلاع قبل الانتقال للـ Home.
 *
 * يفحص خمسة مكونات بالترتيب ويعرض النتيجة في boot_check_events label:
 *   1. ESP32 internal (heap + chip info)
 *   2. ADS1115 (I2C scan @ 0x48 + قراءة تجريبية)
 *   3. Upper FLC100 (قيمة ADC channel A0 في المدى المنطقي)
 *   4. Lower FLC100 (قيمة ADC channel A1 في المدى المنطقي)
 *   5. HC-05 (UART init + AT ping أو rx activity check)
 *
 * عند نجاح كل مكوِّن  → النص بالأخضر #477f53
 * عند فشله            → النص بالأحمر  #cd4755 + الجهاز يدخل lockdown
 * عند اكتمال الفحص   → الانتقال التلقائي لـ SCREEN_ID_MAIN
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief يُشغِّل سلسلة self-check كاملة على boot_screen.
 *
 * يُستدعى من ui_event_task بعد تهيئة LVGL.
 * الدالة blocking — تنتهي إما بالانتقال للـ main screen أو بـ lockdown.
 *
 * @param ads_driver  مؤشر ADS1115Driver_t المُهيَّأ مسبقاً من adc_task.
 */
void boot_selfcheck_run(void *ads_driver);

/**
 * @brief نتيجة الفحص الكامل — true إذا اجتاز كل المكونات.
 * تُستعمَل من app_main لتحديد هل يُشغَّل النظام أم لا.
 */
bool boot_selfcheck_passed(void);

#ifdef __cplusplus
}
#endif
