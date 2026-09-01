/**
 * @file actions.h
 * @brief Action callbacks — تُسجَّل على objects بعد create_screens()
 *
 * EEZ Studio يستخدم flowPropagateValueLVGLEvent داخلياً.
 * لربط Backend بالـ objects نضيف event callbacks إضافية بعد create_screens().
 * كل callback تستدعي الـ ui_event_task API المناسب.
 */

#ifndef EEZ_LVGL_UI_EVENTS_H
#define EEZ_LVGL_UI_EVENTS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief يُسجِّل كل event callbacks على الـ objects بعد create_screens().
 *        استدعِه مرة واحدة من ui_event_task بعد انتظار init الشاشات.
 */
void actions_register_all(void);

#ifdef __cplusplus
}
#endif

#endif /* EEZ_LVGL_UI_EVENTS_H */
