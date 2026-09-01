#include <string.h>

#include "screens.h"
#include "images.h"
#include "fonts.h"
#include "actions.h"
#include "vars.h"
#include "styles.h"
#include "ui.h"

#include <string.h>

objects_t objects;

static const char *screen_names[] = { "Boot_screen", "Main", "TabGroup" };
static const char *object_names[] = { "boot_screen", "main", "tab_group", "boot_background", "boot_spinner", "boot_check_events", "live_scan_button", "scan3_dbutton", "settings_button", "low_battery_panel", "tabview", "live_scan_tab", "live_chart", "obj0", "current_value_panel", "baseline_panel", "baseline_text", "values_text", "live_scan_chart", "snr_panel", "snr_progress_bar", "snr_val", "confidence_value", "info_for_user", "smart_detection", "scan3_dtab", "manual_scan_button", "auto_scan_button", "scan_mode", "scan_directions", "auto_scan_option_panel", "steps_number", "seconds_numbers", "autoscan_start_button", "auto_scan_start_button", "up_seconds", "down_seconds", "up_steps", "down_steps", "auto_scan_process", "auto_scan_cancel_button", "manual_scan_process", "manual_scan_cancel_button", "state_infos", "data_via_ble", "settings_tab", "manual_sens_label", "manual_sensibility_switch", "sensibility_settings", "logs_button", "view_logs", "reset_device_calib", "reset", "advanced_settings_panel", "kalman_filter", "boost_mode", "low_batt", "kalman_spatial", "stability_gate", "calibration_progress_bar", "calibration_button", "calibration_label_statut", "led_calibration_status", "obj1", "bluetooth_status_idle", "bluetooth_status_failed", "bluetooth_status_success", "logs_text" };

//
// Event handlers
//

lv_obj_t *tick_value_change_obj;

static void event_handler_cb_boot_screen_boot_background(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    void *flowState = lv_event_get_user_data(e);
    (void)flowState;
    
    if (event == LV_EVENT_PRESSED) {
        e->user_data = (void *)0;
        flowPropagateValueLVGLEvent(flowState, 0, 0, e);
    }
}

static void event_handler_cb_main_live_scan_button(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    void *flowState = lv_event_get_user_data(e);
    (void)flowState;
    
    if (event == LV_EVENT_RELEASED) {
        e->user_data = (void *)0;
        flowPropagateValueLVGLEvent(flowState, 0, 0, e);
    }
}

static void event_handler_cb_main_scan3_dbutton(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    void *flowState = lv_event_get_user_data(e);
    (void)flowState;
    
    if (event == LV_EVENT_RELEASED) {
        e->user_data = (void *)0;
        flowPropagateValueLVGLEvent(flowState, 2, 0, e);
    }
}

static void event_handler_cb_main_settings_button(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    void *flowState = lv_event_get_user_data(e);
    (void)flowState;
    
    if (event == LV_EVENT_RELEASED) {
        e->user_data = (void *)0;
        flowPropagateValueLVGLEvent(flowState, 3, 0, e);
    }
}

static void event_handler_cb_tab_group_manual_scan_button(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    void *flowState = lv_event_get_user_data(e);
    (void)flowState;
    
    if (event == LV_EVENT_RELEASED) {
        e->user_data = (void *)0;
        flowPropagateValueLVGLEvent(flowState, 26, 0, e);
    }
}

static void event_handler_cb_tab_group_auto_scan_button(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    void *flowState = lv_event_get_user_data(e);
    (void)flowState;
    
    if (event == LV_EVENT_PRESSED) {
        e->user_data = (void *)0;
        flowPropagateValueLVGLEvent(flowState, 28, 0, e);
    }
}

static void event_handler_cb_tab_group_steps_number(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    void *flowState = lv_event_get_user_data(e);
    (void)flowState;
    
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target(e);
        if (tick_value_change_obj != ta) {
            const char *value = lv_textarea_get_text(ta);
            assignStringProperty(flowState, 36, 3, value, "Failed to assign Text in Textarea widget");
        }
    }
}

static void event_handler_cb_tab_group_seconds_numbers(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    void *flowState = lv_event_get_user_data(e);
    (void)flowState;
    
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *ta = lv_event_get_target(e);
        if (tick_value_change_obj != ta) {
            const char *value = lv_textarea_get_text(ta);
            assignStringProperty(flowState, 38, 3, value, "Failed to assign Text in Textarea widget");
        }
    }
}

static void event_handler_cb_tab_group_autoscan_start_button(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    void *flowState = lv_event_get_user_data(e);
    (void)flowState;
    
    if (event == LV_EVENT_RELEASED) {
        e->user_data = (void *)0;
        flowPropagateValueLVGLEvent(flowState, 39, 0, e);
    }
}

static void event_handler_cb_tab_group_up_seconds(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    void *flowState = lv_event_get_user_data(e);
    (void)flowState;
    
    if (event == LV_EVENT_RELEASED) {
        e->user_data = (void *)0;
        flowPropagateValueLVGLEvent(flowState, 41, 0, e);
    }
}

static void event_handler_cb_tab_group_down_seconds(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    void *flowState = lv_event_get_user_data(e);
    (void)flowState;
    
    if (event == LV_EVENT_RELEASED) {
        e->user_data = (void *)0;
        flowPropagateValueLVGLEvent(flowState, 42, 0, e);
    }
}

static void event_handler_cb_tab_group_up_steps(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    void *flowState = lv_event_get_user_data(e);
    (void)flowState;
    
    if (event == LV_EVENT_RELEASED) {
        e->user_data = (void *)0;
        flowPropagateValueLVGLEvent(flowState, 43, 0, e);
    }
}

static void event_handler_cb_tab_group_down_steps(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    void *flowState = lv_event_get_user_data(e);
    (void)flowState;
    
    if (event == LV_EVENT_RELEASED) {
        e->user_data = (void *)0;
        flowPropagateValueLVGLEvent(flowState, 44, 0, e);
    }
}

static void event_handler_cb_tab_group_auto_scan_cancel_button(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    void *flowState = lv_event_get_user_data(e);
    (void)flowState;
    
    if (event == LV_EVENT_RELEASED) {
        e->user_data = (void *)0;
        flowPropagateValueLVGLEvent(flowState, 46, 0, e);
    }
}

static void event_handler_cb_tab_group_manual_scan_cancel_button(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    void *flowState = lv_event_get_user_data(e);
    (void)flowState;
    
    if (event == LV_EVENT_RELEASED) {
        e->user_data = (void *)0;
        flowPropagateValueLVGLEvent(flowState, 49, 0, e);
    }
}

static void event_handler_cb_tab_group_settings_tab(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    void *flowState = lv_event_get_user_data(e);
    (void)flowState;
    
    if (event == LV_EVENT_CLICKED) {
        e->user_data = (void *)0;
        flowPropagateValueLVGLEvent(flowState, 56, 0, e);
    }
}

static void event_handler_cb_tab_group_manual_sensibility_switch(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    void *flowState = lv_event_get_user_data(e);
    (void)flowState;
    
    if (event == LV_EVENT_VALUE_CHANGED) {
        if (lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED)) {
            e->user_data = (void *)0;
            flowPropagateValueLVGLEvent(flowState, 60, 0, e);
        }
    }
    if (event == LV_EVENT_VALUE_CHANGED) {
        if (!lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED)) {
            e->user_data = (void *)0;
            flowPropagateValueLVGLEvent(flowState, 60, 1, e);
        }
    }
}

static void event_handler_cb_tab_group_logs_button(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    void *flowState = lv_event_get_user_data(e);
    (void)flowState;
    
    if (event == LV_EVENT_RELEASED) {
        e->user_data = (void *)0;
        flowPropagateValueLVGLEvent(flowState, 62, 0, e);
    }
}

static void event_handler_cb_tab_group_low_batt(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    void *flowState = lv_event_get_user_data(e);
    (void)flowState;
    
    if (event == LV_EVENT_VALUE_CHANGED) {
        if (lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED)) {
            e->user_data = (void *)0;
            flowPropagateValueLVGLEvent(flowState, 70, 0, e);
        }
    }
}

static void event_handler_cb_tab_group_calibration_label_statut(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    void *flowState = lv_event_get_user_data(e);
    (void)flowState;
    
    if (event == LV_EVENT_PRESSED) {
        e->user_data = (void *)0;
        flowPropagateValueLVGLEvent(flowState, 76, 0, e);
    }
}

static void event_handler_cb_tab_group_obj1(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    void *flowState = lv_event_get_user_data(e);
    (void)flowState;
    
    if (event == LV_EVENT_CLICKED) {
        e->user_data = (void *)0;
        flowPropagateValueLVGLEvent(flowState, 78, 0, e);
    }
}

//
// Screens
//

void create_screen_boot_screen() {
    void *flowState = getFlowState(0, 0);
    (void)flowState;
    lv_obj_t *obj = lv_obj_create(0);
    objects.boot_screen = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 320, 240);
    {
        lv_obj_t *parent_obj = obj;
        {
            // Boot_background
            lv_obj_t *obj = lv_img_create(parent_obj);
            objects.boot_background = obj;
            lv_obj_set_pos(obj, 0, 0);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_img_set_src(obj, &img_boot_screen);
            lv_obj_add_event_cb(obj, event_handler_cb_boot_screen_boot_background, LV_EVENT_ALL, flowState);
            lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_clear_flag(obj, LV_OBJ_FLAG_ADV_HITTEST|LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
        }
        {
            // Boot_spinner
            lv_obj_t *obj = lv_spinner_create(parent_obj, 1000, 60);
            objects.boot_spinner = obj;
            lv_obj_set_pos(obj, 142, 165);
            lv_obj_set_size(obj, 36, 43);
            lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICK_FOCUSABLE|LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
        }
        {
            // Boot_check_events
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.boot_check_events = obj;
            lv_obj_set_pos(obj, 108, 211);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_obj_set_style_text_opa(obj, 210, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text(obj, "System starting...");
        }
    }
    
    tick_screen_boot_screen();
}

void tick_screen_boot_screen() {
    void *flowState = getFlowState(0, 0);
    (void)flowState;
}

void create_screen_main() {
    void *flowState = getFlowState(0, 1);
    (void)flowState;
    lv_obj_t *obj = lv_obj_create(0);
    objects.main = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 320, 240);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_CLICK_FOCUSABLE|LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_SNAPPABLE|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER);
    lv_obj_set_style_bg_img_src(obj, &img_mainbackground, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    {
        lv_obj_t *parent_obj = obj;
        {
            // LiveScanButton
            lv_obj_t *obj = lv_btn_create(parent_obj);
            objects.live_scan_button = obj;
            lv_obj_set_pos(obj, 2, 73);
            lv_obj_set_size(obj, 137, 77);
            lv_obj_add_event_cb(obj, event_handler_cb_main_live_scan_button, LV_EVENT_ALL, flowState);
            lv_obj_clear_flag(obj, LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ON_FOCUS|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
            lv_obj_set_style_bg_opa(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_img_src(obj, &img_live_scan_button, LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        {
            // SCAN3DButton
            lv_obj_t *obj = lv_btn_create(parent_obj);
            objects.scan3_dbutton = obj;
            lv_obj_set_pos(obj, 181, 73);
            lv_obj_set_size(obj, 137, 77);
            lv_obj_add_event_cb(obj, event_handler_cb_main_scan3_dbutton, LV_EVENT_ALL, flowState);
            lv_obj_set_style_bg_opa(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_img_src(obj, &img_scan3d_button, LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        {
            // SettingsButton
            lv_obj_t *obj = lv_btn_create(parent_obj);
            objects.settings_button = obj;
            lv_obj_set_pos(obj, 124, 159);
            lv_obj_set_size(obj, 72, 69);
            lv_obj_add_event_cb(obj, event_handler_cb_main_settings_button, LV_EVENT_ALL, flowState);
            lv_obj_set_style_bg_opa(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_img_src(obj, &img_settings_button, LV_PART_MAIN | LV_STATE_DEFAULT);
        }
    }
    
    tick_screen_main();
}

void tick_screen_main() {
    void *flowState = getFlowState(0, 1);
    (void)flowState;
}

void create_screen_tab_group() {
    void *flowState = getFlowState(0, 2);
    (void)flowState;
    lv_obj_t *obj = lv_obj_create(0);
    objects.tab_group = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 320, 240);
    {
        lv_obj_t *parent_obj = obj;
        {
            // Low_Battery_Panel
            lv_obj_t *obj = lv_obj_create(parent_obj);
            objects.low_battery_panel = obj;
            lv_obj_set_pos(obj, 59, 40);
            lv_obj_set_size(obj, 202, 161);
            lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_CLICK_FOCUSABLE|LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
            {
                lv_obj_t *parent_obj = obj;
                {
                    lv_obj_t *obj = lv_img_create(parent_obj);
                    lv_obj_set_pos(obj, 47, 18);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_img_set_src(obj, &img_low_battery);
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 37, 121);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_label_set_text(obj, "LOW Battery!");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, 53, -7);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_label_set_text(obj, "Warning:");
                }
            }
        }
        {
            // tabview
            lv_obj_t *obj = lv_tabview_create(parent_obj, LV_DIR_TOP, 30);
            objects.tabview = obj;
            lv_obj_set_pos(obj, 0, 0);
            lv_obj_set_size(obj, 320, 240);
            lv_obj_add_flag(obj, LV_OBJ_FLAG_CHECKABLE);
            lv_obj_clear_flag(obj, LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
            lv_obj_set_style_bg_img_src(obj, &img_background1, LV_PART_MAIN | LV_STATE_DEFAULT);
            {
                lv_obj_t *parent_obj = obj;
                {
                    // LiveScanTab
                    lv_obj_t *obj = lv_tabview_add_tab(parent_obj, "Live Scan");
                    objects.live_scan_tab = obj;
                    lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_CLICK_FOCUSABLE|LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                    {
                        lv_obj_t *parent_obj = obj;
                        {
                            // LiveChart
                            lv_obj_t *obj = lv_obj_create(parent_obj);
                            objects.live_chart = obj;
                            lv_obj_set_pos(obj, -3, 3);
                            lv_obj_set_size(obj, 295, 151);
                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_CLICK_FOCUSABLE|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SNAPPABLE);
                            lv_obj_set_style_radius(obj, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_top(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_bottom(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_left(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_right(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_row(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_column(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_bg_opa(obj, 200, LV_PART_MAIN | LV_STATE_DEFAULT);
                        }
                        {
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.obj0 = obj;
                            lv_obj_set_pos(obj, -1, -4);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_decor(obj, LV_TEXT_DECOR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_bg_color(obj, lv_color_hex(0xff1f2329), LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_bg_opa(obj, 220, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_radius(obj, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text(obj, "  Live Scope ");
                        }
                        {
                            // CurrentValuePanel
                            lv_obj_t *obj = lv_obj_create(parent_obj);
                            objects.current_value_panel = obj;
                            lv_obj_set_pos(obj, 96, 156);
                            lv_obj_set_size(obj, 97, 37);
                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_MOMENTUM);
                            lv_obj_set_style_radius(obj, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_top(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_bottom(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_left(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_right(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_row(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_column(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_bg_opa(obj, 200, LV_PART_MAIN | LV_STATE_DEFAULT);
                        }
                        {
                            // BaselinePanel
                            lv_obj_t *obj = lv_obj_create(parent_obj);
                            objects.baseline_panel = obj;
                            lv_obj_set_pos(obj, -3, 156);
                            lv_obj_set_size(obj, 97, 37);
                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_MOMENTUM);
                            lv_obj_set_style_radius(obj, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_top(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_bottom(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_left(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_right(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_row(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_column(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_bg_opa(obj, 200, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_color(obj, lv_color_hex(0xff0b5590), LV_PART_MAIN | LV_STATE_DEFAULT);
                        }
                        {
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            lv_obj_set_pos(obj, -3, 167);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_label_set_text(obj, " Ref:");
                        }
                        {
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            lv_obj_set_pos(obj, 96, 167);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_label_set_text(obj, " Val:");
                        }
                        {
                            // Baseline_text
                            lv_obj_t *obj = lv_textarea_create(parent_obj);
                            objects.baseline_text = obj;
                            lv_obj_set_pos(obj, 23, 159);
                            lv_obj_set_size(obj, 71, 32);
                            lv_textarea_set_max_length(obj, 10);
                            lv_textarea_set_text(obj, "0");
                            lv_textarea_set_one_line(obj, true);
                            lv_textarea_set_password_mode(obj, false);
                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_MOMENTUM);
                            lv_obj_set_style_bg_opa(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_color(obj, lv_color_hex(0xff1772b9), LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                        }
                        {
                            // Values_text
                            lv_obj_t *obj = lv_textarea_create(parent_obj);
                            objects.values_text = obj;
                            lv_obj_set_pos(obj, 118, 159);
                            lv_obj_set_size(obj, 75, 32);
                            lv_textarea_set_max_length(obj, 7);
                            lv_textarea_set_text(obj, "0");
                            lv_textarea_set_one_line(obj, true);
                            lv_textarea_set_password_mode(obj, false);
                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM);
                            lv_obj_set_style_bg_opa(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_color(obj, lv_color_hex(0xff0d7547), LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                        }
                        {
                            // LiveScan_chart
                            lv_obj_t *obj = lv_chart_create(parent_obj);
                            objects.live_scan_chart = obj;
                            lv_obj_set_pos(obj, 3, 11);
                            lv_obj_set_size(obj, 284, 134);
                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_CLICK_FOCUSABLE|LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                            lv_obj_set_style_bg_opa(obj, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
                            {
                                lv_obj_t *parent_obj = obj;
                                {
                                    // SNR Panel
                                    lv_obj_t *obj = lv_obj_create(parent_obj);
                                    objects.snr_panel = obj;
                                    lv_obj_set_pos(obj, 165, -10);
                                    lv_obj_set_size(obj, 108, 48);
                                    lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_CLICK_FOCUSABLE|LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                                    lv_obj_set_style_text_opa(obj, 200, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    {
                                        lv_obj_t *parent_obj = obj;
                                        {
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            lv_obj_set_pos(obj, -10, -10);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_obj_set_style_text_opa(obj, 250, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_label_set_text(obj, "SNR");
                                        }
                                        {
                                            // SNR Progress bar
                                            lv_obj_t *obj = lv_bar_create(parent_obj);
                                            objects.snr_progress_bar = obj;
                                            lv_obj_set_pos(obj, 14, -9);
                                            lv_obj_set_size(obj, 73, 10);
                                            lv_bar_set_range(obj, 0, 6);
                                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_CLICK_FOCUSABLE|LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                                            {
                                                lv_obj_t *parent_obj = obj;
                                                {
                                                    // SNR val
                                                    lv_obj_t *obj = lv_label_create(parent_obj);
                                                    objects.snr_val = obj;
                                                    lv_obj_set_pos(obj, 31, 0);
                                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                                    lv_label_set_recolor(obj, true);
                                                    lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICK_FOCUSABLE|LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                                                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);
                                                    lv_obj_set_style_text_opa(obj, 250, LV_PART_MAIN | LV_STATE_DEFAULT);
                                                    lv_label_set_text(obj, "0");
                                                }
                                            }
                                        }
                                        {
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            lv_obj_set_pos(obj, -11, 1);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_obj_set_style_text_opa(obj, 250, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_label_set_text(obj, "Confidence");
                                        }
                                        {
                                            // Confidence value
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            objects.confidence_value = obj;
                                            lv_obj_set_pos(obj, 58, 2);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICK_FOCUSABLE|LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                                            lv_obj_set_style_text_opa(obj, 250, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_label_set_text(obj, "0%");
                                        }
                                        {
                                            // Info for user
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            objects.info_for_user = obj;
                                            lv_obj_set_pos(obj, 29, 16);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_label_set_recolor(obj, true);
                                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_obj_set_style_text_opa(obj, 250, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_label_set_text(obj, "infos");
                                        }
                                    }
                                }
                            }
                        }
                        {
                            // smart_detection
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            objects.smart_detection = obj;
                            lv_obj_set_pos(obj, 194, 130);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICK_FOCUSABLE|LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text(obj, "");
                        }
                    }
                }
                {
                    // Scan3DTab
                    lv_obj_t *obj = lv_tabview_add_tab(parent_obj, "3D Scan");
                    objects.scan3_dtab = obj;
                    lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICK_FOCUSABLE|LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                    {
                        lv_obj_t *parent_obj = obj;
                        {
                            lv_obj_t *obj = lv_img_create(parent_obj);
                            lv_obj_set_pos(obj, 74, 11);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_img_set_src(obj, &img_scan3d_photo);
                            lv_img_set_zoom(obj, 320);
                            lv_img_set_size_mode(obj, LV_IMG_SIZE_MODE_REAL);
                        }
                        {
                            // Manual_Scan_Button
                            lv_obj_t *obj = lv_btn_create(parent_obj);
                            objects.manual_scan_button = obj;
                            lv_obj_set_pos(obj, 29, 98);
                            lv_obj_set_size(obj, 102, 30);
                            lv_obj_add_event_cb(obj, event_handler_cb_tab_group_manual_scan_button, LV_EVENT_ALL, flowState);
                            lv_obj_set_style_bg_color(obj, lv_color_hex(0xff2196f3), LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_radius(obj, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_bg_opa(obj, 200, LV_PART_MAIN | LV_STATE_DEFAULT);
                            {
                                lv_obj_t *parent_obj = obj;
                                {
                                    lv_obj_t *obj = lv_label_create(parent_obj);
                                    lv_obj_set_pos(obj, 0, -1);
                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_label_set_text(obj, "Manual Scan");
                                }
                            }
                        }
                        {
                            // Auto_Scan_Button
                            lv_obj_t *obj = lv_btn_create(parent_obj);
                            objects.auto_scan_button = obj;
                            lv_obj_set_pos(obj, 153, 98);
                            lv_obj_set_size(obj, 102, 30);
                            lv_obj_add_event_cb(obj, event_handler_cb_tab_group_auto_scan_button, LV_EVENT_ALL, flowState);
                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_ON_FOCUS|LV_OBJ_FLAG_SCROLL_WITH_ARROW);
                            lv_obj_set_style_bg_color(obj, lv_color_hex(0xff2196f3), LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_radius(obj, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_bg_opa(obj, 200, LV_PART_MAIN | LV_STATE_DEFAULT);
                            {
                                lv_obj_t *parent_obj = obj;
                                {
                                    lv_obj_t *obj = lv_label_create(parent_obj);
                                    lv_obj_set_pos(obj, 7, 0);
                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_label_set_text(obj, "Auto Scan");
                                }
                            }
                        }
                        {
                            // Scan_Mode
                            lv_obj_t *obj = lv_dropdown_create(parent_obj);
                            objects.scan_mode = obj;
                            lv_obj_set_pos(obj, -7, 150);
                            lv_obj_set_size(obj, 94, 23);
                            lv_dropdown_set_options(obj, "Raw\nNormalize\nEnhanced");
                            lv_dropdown_set_dir(obj, LV_DIR_RIGHT);
                            lv_dropdown_set_symbol(obj, LV_SYMBOL_RIGHT);
                            lv_dropdown_set_selected(obj, 0);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_top(obj, 3, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_bottom(obj, 3, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_row(obj, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
                        }
                        {
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            lv_obj_set_pos(obj, -5, 135);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text(obj, "Mode:");
                        }
                        {
                            // Scan_Directions
                            lv_obj_t *obj = lv_dropdown_create(parent_obj);
                            objects.scan_directions = obj;
                            lv_obj_set_pos(obj, 101, 150);
                            lv_obj_set_size(obj, 84, 23);
                            lv_dropdown_set_options(obj, "N->S\nS->N\nE->W\nW->E\nDisabled");
                            lv_dropdown_set_dir(obj, LV_DIR_RIGHT);
                            lv_dropdown_set_symbol(obj, LV_SYMBOL_RIGHT);
                            lv_dropdown_set_selected(obj, 0);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_top(obj, 3, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_bottom(obj, 3, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_row(obj, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
                        }
                        {
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            lv_obj_set_pos(obj, 103, 135);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text(obj, "Directions:");
                        }
                        {
                            // Auto_scan_optionPanel
                            lv_obj_t *obj = lv_obj_create(parent_obj);
                            objects.auto_scan_option_panel = obj;
                            lv_obj_set_pos(obj, 54, 11);
                            lv_obj_set_size(obj, 182, 143);
                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_CLICK_FOCUSABLE|LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                            lv_obj_set_style_pad_top(obj, 0, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_bottom(obj, 0, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_left(obj, 0, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_right(obj, 0, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_row(obj, 0, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_column(obj, 0, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
                            lv_obj_set_style_bg_opa(obj, 235, LV_PART_MAIN | LV_STATE_DEFAULT);
                            {
                                lv_obj_t *parent_obj = obj;
                                {
                                    lv_obj_t *obj = lv_label_create(parent_obj);
                                    lv_obj_set_pos(obj, 2, -4);
                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                    lv_label_set_text(obj, "Steps");
                                }
                                {
                                    // steps_number
                                    lv_obj_t *obj = lv_textarea_create(parent_obj);
                                    objects.steps_number = obj;
                                    lv_obj_set_pos(obj, 6, 42);
                                    lv_obj_set_size(obj, 32, 36);
                                    lv_textarea_set_max_length(obj, 2);
                                    lv_textarea_set_placeholder_text(obj, "10");
                                    lv_textarea_set_one_line(obj, true);
                                    lv_textarea_set_password_mode(obj, false);
                                    lv_obj_add_event_cb(obj, event_handler_cb_tab_group_steps_number, LV_EVENT_ALL, flowState);
                                    lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_CLICK_FOCUSABLE|LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_ON_FOCUS|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                                    lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_AUTO, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
                                }
                                {
                                    lv_obj_t *obj = lv_label_create(parent_obj);
                                    lv_obj_set_pos(obj, 116, -4);
                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                    lv_label_set_text(obj, "Sec.");
                                }
                                {
                                    // Seconds_numbers
                                    lv_obj_t *obj = lv_textarea_create(parent_obj);
                                    objects.seconds_numbers = obj;
                                    lv_obj_set_pos(obj, 115, 42);
                                    lv_obj_set_size(obj, 32, 36);
                                    lv_textarea_set_max_length(obj, 1);
                                    lv_textarea_set_placeholder_text(obj, "2");
                                    lv_textarea_set_one_line(obj, true);
                                    lv_textarea_set_password_mode(obj, false);
                                    lv_obj_add_event_cb(obj, event_handler_cb_tab_group_seconds_numbers, LV_EVENT_ALL, flowState);
                                    lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_CLICK_FOCUSABLE|LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_ON_FOCUS|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                                    lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_CENTER, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
                                }
                                {
                                    // Autoscan_Start_button
                                    lv_obj_t *obj = lv_btn_create(parent_obj);
                                    objects.autoscan_start_button = obj;
                                    lv_obj_set_pos(obj, 50, 90);
                                    lv_obj_set_size(obj, 53, 30);
                                    lv_obj_add_event_cb(obj, event_handler_cb_tab_group_autoscan_start_button, LV_EVENT_ALL, flowState);
                                    lv_obj_clear_flag(obj, LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_ON_FOCUS|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                                    lv_obj_set_style_bg_color(obj, lv_color_hex(0xff2196f3), LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_radius(obj, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_bg_opa(obj, 200, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    {
                                        lv_obj_t *parent_obj = obj;
                                        {
                                            // Auto_Scan_Start_Button
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            objects.auto_scan_start_button = obj;
                                            lv_obj_set_pos(obj, -4, -1);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                                            lv_label_set_text(obj, "Start");
                                        }
                                    }
                                }
                                {
                                    // Up_seconds
                                    lv_obj_t *obj = lv_btn_create(parent_obj);
                                    objects.up_seconds = obj;
                                    lv_obj_set_pos(obj, 120, 19);
                                    lv_obj_set_size(obj, 21, 20);
                                    lv_obj_add_event_cb(obj, event_handler_cb_tab_group_up_seconds, LV_EVENT_ALL, flowState);
                                    lv_obj_set_style_bg_img_src(obj, &img_up_arrow, LV_PART_MAIN | LV_STATE_DEFAULT);
                                }
                                {
                                    // Down_seconds
                                    lv_obj_t *obj = lv_btn_create(parent_obj);
                                    objects.down_seconds = obj;
                                    lv_obj_set_pos(obj, 120, 80);
                                    lv_obj_set_size(obj, 21, 20);
                                    lv_obj_add_event_cb(obj, event_handler_cb_tab_group_down_seconds, LV_EVENT_ALL, flowState);
                                    lv_obj_set_style_bg_img_src(obj, &img_down_steps, LV_PART_MAIN | LV_STATE_DEFAULT);
                                }
                                {
                                    // Up_steps
                                    lv_obj_t *obj = lv_btn_create(parent_obj);
                                    objects.up_steps = obj;
                                    lv_obj_set_pos(obj, 12, 19);
                                    lv_obj_set_size(obj, 21, 20);
                                    lv_obj_add_event_cb(obj, event_handler_cb_tab_group_up_steps, LV_EVENT_ALL, flowState);
                                    lv_obj_clear_flag(obj, LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_ON_FOCUS|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                                    lv_obj_set_style_bg_img_src(obj, &img_up_arrow, LV_PART_MAIN | LV_STATE_DEFAULT);
                                }
                                {
                                    // Down_steps
                                    lv_obj_t *obj = lv_btn_create(parent_obj);
                                    objects.down_steps = obj;
                                    lv_obj_set_pos(obj, 12, 80);
                                    lv_obj_set_size(obj, 21, 20);
                                    lv_obj_add_event_cb(obj, event_handler_cb_tab_group_down_steps, LV_EVENT_ALL, flowState);
                                    lv_obj_set_style_bg_img_src(obj, &img_down_steps, LV_PART_MAIN | LV_STATE_DEFAULT);
                                }
                            }
                        }
                        {
                            // Auto_scan_Process
                            lv_obj_t *obj = lv_obj_create(parent_obj);
                            objects.auto_scan_process = obj;
                            lv_obj_set_pos(obj, 65, 28);
                            lv_obj_set_size(obj, 165, 128);
                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_CLICK_FOCUSABLE|LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                            {
                                lv_obj_t *parent_obj = obj;
                                {
                                    // AutoScan_Cancel_Button
                                    lv_obj_t *obj = lv_btn_create(parent_obj);
                                    objects.auto_scan_cancel_button = obj;
                                    lv_obj_set_pos(obj, 32, 84);
                                    lv_obj_set_size(obj, 71, 23);
                                    lv_obj_add_event_cb(obj, event_handler_cb_tab_group_auto_scan_cancel_button, LV_EVENT_ALL, flowState);
                                    lv_obj_set_style_bg_color(obj, lv_color_hex(0xff2196f3), LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_radius(obj, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_bg_opa(obj, 200, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    {
                                        lv_obj_t *parent_obj = obj;
                                        {
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            lv_obj_set_pos(obj, 1, -4);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_label_set_text(obj, "Cancel");
                                        }
                                    }
                                }
                                {
                                    lv_obj_t *obj = lv_label_create(parent_obj);
                                    lv_obj_set_pos(obj, 17, -11);
                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_label_set_text(obj, " Auto Scanning...");
                                }
                            }
                        }
                        {
                            // Manual_scan_Process
                            lv_obj_t *obj = lv_obj_create(parent_obj);
                            objects.manual_scan_process = obj;
                            lv_obj_set_pos(obj, 65, 44);
                            lv_obj_set_size(obj, 165, 95);
                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_CLICK_FOCUSABLE|LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                            {
                                lv_obj_t *parent_obj = obj;
                                {
                                    // ManualScan_Cancel_Button
                                    lv_obj_t *obj = lv_btn_create(parent_obj);
                                    objects.manual_scan_cancel_button = obj;
                                    lv_obj_set_pos(obj, 32, 41);
                                    lv_obj_set_size(obj, 71, 23);
                                    lv_obj_add_event_cb(obj, event_handler_cb_tab_group_manual_scan_cancel_button, LV_EVENT_ALL, flowState);
                                    lv_obj_set_style_bg_color(obj, lv_color_hex(0xff2196f3), LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_radius(obj, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_bg_opa(obj, 200, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    {
                                        lv_obj_t *parent_obj = obj;
                                        {
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            lv_obj_set_pos(obj, 1, -4);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_label_set_text(obj, "Cancel");
                                        }
                                    }
                                }
                                {
                                    lv_obj_t *obj = lv_label_create(parent_obj);
                                    lv_obj_set_pos(obj, 14, -1);
                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_label_set_text(obj, " Manual Scanning...");
                                }
                            }
                        }
                        {
                            // state_infos
                            lv_obj_t *obj = lv_textarea_create(parent_obj);
                            objects.state_infos = obj;
                            lv_obj_set_pos(obj, -12, 179);
                            lv_obj_set_size(obj, 197, 15);
                            lv_textarea_set_max_length(obj, 256);
                            lv_textarea_set_placeholder_text(obj, ">Device logger...");
                            lv_textarea_set_one_line(obj, false);
                            lv_textarea_set_password_mode(obj, false);
                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_CLICK_FOCUSABLE|LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_ON_FOCUS|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                            lv_obj_set_style_bg_opa(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_bg_grad_dir(obj, LV_GRAD_DIR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_border_opa(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_text_align(obj, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_border_side(obj, LV_BORDER_SIDE_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_line_width(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_top(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_bottom(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_left(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_right(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_obj_set_style_pad_row(obj, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
                        }
                        {
                            // Data via BLE
                            lv_obj_t *obj = lv_switch_create(parent_obj);
                            objects.data_via_ble = obj;
                            lv_obj_set_pos(obj, 260, 11);
                            lv_obj_set_size(obj, 37, 17);
                        }
                        {
                            lv_obj_t *obj = lv_label_create(parent_obj);
                            lv_obj_set_pos(obj, 146, 11);
                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                            lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                            lv_label_set_text(obj, "Send Data via BLE");
                        }
                    }
                }
                {
                    // SettingsTab
                    lv_obj_t *obj = lv_tabview_add_tab(parent_obj, "Settings");
                    objects.settings_tab = obj;
                    lv_obj_add_event_cb(obj, event_handler_cb_tab_group_settings_tab, LV_EVENT_ALL, flowState);
                    lv_obj_clear_flag(obj, LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                    {
                        lv_obj_t *parent_obj = obj;
                        {
                            lv_obj_t *obj = lv_obj_create(parent_obj);
                            lv_obj_set_pos(obj, -4, 11);
                            lv_obj_set_size(obj, 142, 179);
                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_CLICK_FOCUSABLE|LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                            lv_obj_set_style_bg_opa(obj, 200, LV_PART_MAIN | LV_STATE_DEFAULT);
                            {
                                lv_obj_t *parent_obj = obj;
                                {
                                    lv_obj_t *obj = lv_label_create(parent_obj);
                                    lv_obj_set_pos(obj, -10, -5);
                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                    lv_label_set_text(obj, "Sensibility:");
                                }
                                {
                                    // Manual_Sens_Label
                                    lv_obj_t *obj = lv_label_create(parent_obj);
                                    objects.manual_sens_label = obj;
                                    lv_obj_set_pos(obj, 54, 29);
                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_label_set_text(obj, "Manual");
                                }
                                {
                                    // Manual_sensibility_switch
                                    lv_obj_t *obj = lv_switch_create(parent_obj);
                                    objects.manual_sensibility_switch = obj;
                                    lv_obj_set_pos(obj, -6, 24);
                                    lv_obj_set_size(obj, 50, 25);
                                    lv_obj_add_event_cb(obj, event_handler_cb_tab_group_manual_sensibility_switch, LV_EVENT_ALL, flowState);
                                }
                                {
                                    // Sensibility_Settings
                                    lv_obj_t *obj = lv_dropdown_create(parent_obj);
                                    objects.sensibility_settings = obj;
                                    lv_obj_set_pos(obj, -6, 55);
                                    lv_obj_set_size(obj, 125, LV_SIZE_CONTENT);
                                    lv_dropdown_set_options(obj, "Very Low\nLow\nMedium\nHigh\nVery High");
                                    lv_dropdown_set_dir(obj, LV_DIR_RIGHT);
                                    lv_dropdown_set_symbol(obj, LV_SYMBOL_RIGHT);
                                    lv_dropdown_set_selected(obj, 2);
                                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_border_side(obj, LV_BORDER_SIDE_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
                                }
                                {
                                    // Logs_Button
                                    lv_obj_t *obj = lv_btn_create(parent_obj);
                                    objects.logs_button = obj;
                                    lv_obj_set_pos(obj, -6, 127);
                                    lv_obj_set_size(obj, 71, 26);
                                    lv_obj_add_event_cb(obj, event_handler_cb_tab_group_logs_button, LV_EVENT_ALL, flowState);
                                    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_ON_FOCUS|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                                    lv_obj_set_style_bg_opa(obj, 200, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    {
                                        lv_obj_t *parent_obj = obj;
                                        {
                                            // View Logs
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            objects.view_logs = obj;
                                            lv_obj_set_pos(obj, 0, -1);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_label_set_text(obj, "Logs");
                                        }
                                    }
                                }
                                {
                                    // Reset_Device_calib
                                    lv_obj_t *obj = lv_btn_create(parent_obj);
                                    objects.reset_device_calib = obj;
                                    lv_obj_set_pos(obj, -6, 93);
                                    lv_obj_set_size(obj, 125, 26);
                                    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_ON_FOCUS|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                                    lv_obj_set_style_bg_opa(obj, 200, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    {
                                        lv_obj_t *parent_obj = obj;
                                        {
                                            // reset
                                            lv_obj_t *obj = lv_label_create(parent_obj);
                                            objects.reset = obj;
                                            lv_obj_set_pos(obj, 0, 0);
                                            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                                            lv_obj_set_style_align(obj, LV_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
                                            lv_label_set_text(obj, "Reset Dev Calib");
                                        }
                                    }
                                }
                            }
                        }
                        {
                            // Advanced_Settings_Panel
                            lv_obj_t *obj = lv_obj_create(parent_obj);
                            objects.advanced_settings_panel = obj;
                            lv_obj_set_pos(obj, 151, 11);
                            lv_obj_set_size(obj, 145, 174);
                            lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_CLICK_FOCUSABLE|LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                            lv_obj_set_style_bg_opa(obj, 200, LV_PART_MAIN | LV_STATE_DEFAULT);
                            {
                                lv_obj_t *parent_obj = obj;
                                {
                                    lv_obj_t *obj = lv_label_create(parent_obj);
                                    lv_obj_set_pos(obj, -10, -5);
                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                    lv_label_set_text(obj, "Advanced:");
                                }
                                {
                                    // Kalman_Filter
                                    lv_obj_t *obj = lv_checkbox_create(parent_obj);
                                    objects.kalman_filter = obj;
                                    lv_obj_set_pos(obj, -9, 72);
                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                    lv_checkbox_set_text(obj, "Kalman");
                                    lv_obj_clear_flag(obj, LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_ON_FOCUS|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                                    lv_obj_add_state(obj, LV_STATE_CHECKED);
                                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_text_letter_space(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                                }
                                {
                                    // Boost_Mode
                                    lv_obj_t *obj = lv_checkbox_create(parent_obj);
                                    objects.boost_mode = obj;
                                    lv_obj_set_pos(obj, -9, 16);
                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                    lv_checkbox_set_text(obj, "Boost Mode [256]");
                                    lv_obj_clear_flag(obj, LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_ON_FOCUS|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                                    lv_obj_add_state(obj, LV_STATE_CHECKED);
                                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                                }
                                {
                                    // Low_Batt
                                    lv_obj_t *obj = lv_checkbox_create(parent_obj);
                                    objects.low_batt = obj;
                                    lv_obj_set_pos(obj, -9, 127);
                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                    lv_checkbox_set_text(obj, "Low Batt.");
                                    lv_obj_add_event_cb(obj, event_handler_cb_tab_group_low_batt, LV_EVENT_ALL, flowState);
                                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                                }
                                {
                                    // Kalman_Spatial
                                    lv_obj_t *obj = lv_checkbox_create(parent_obj);
                                    objects.kalman_spatial = obj;
                                    lv_obj_set_pos(obj, -9, 100);
                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                    lv_checkbox_set_text(obj, "Kalman+Spatial");
                                    lv_obj_clear_flag(obj, LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_ON_FOCUS|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                                }
                                {
                                    // Stability Gate
                                    lv_obj_t *obj = lv_checkbox_create(parent_obj);
                                    objects.stability_gate = obj;
                                    lv_obj_set_pos(obj, -9, 44);
                                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                                    lv_checkbox_set_text(obj, "Adap. Sampling");
                                    lv_obj_clear_flag(obj, LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_ON_FOCUS|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
                                    lv_obj_add_state(obj, LV_STATE_CHECKED);
                                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                                }
                            }
                        }
                    }
                }
            }
        }
        {
            // Calibration_progressBar
            lv_obj_t *obj = lv_bar_create(parent_obj);
            objects.calibration_progress_bar = obj;
            lv_obj_set_pos(obj, 210, 202);
            lv_obj_set_size(obj, 93, 24);
            lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_CLICK_FOCUSABLE|LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xff0b5997), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_opa(obj, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_grad_dir(obj, LV_GRAD_DIR_HOR, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_grad_color(obj, lv_color_hex(0xff0c3555), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_grad_stop(obj, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_radius(obj, 3, LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        {
            // CALIBRATIONButton
            lv_obj_t *obj = lv_btn_create(parent_obj);
            objects.calibration_button = obj;
            lv_obj_set_pos(obj, 211, 202);
            lv_obj_set_size(obj, 91, 24);
            lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_ON_FOCUS|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xff2196f3), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_radius(obj, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_opa(obj, 100, LV_PART_MAIN | LV_STATE_DEFAULT);
            {
                lv_obj_t *parent_obj = obj;
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    lv_obj_set_pos(obj, -2, -3);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "CALIBRATE");
                }
            }
        }
        {
            // Calibration_Label_Statut
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.calibration_label_statut = obj;
            lv_obj_set_pos(obj, 249, 227);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_obj_add_event_cb(obj, event_handler_cb_tab_group_calibration_label_statut, LV_EVENT_ALL, flowState);
            lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICK_FOCUSABLE|LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
            lv_obj_set_style_text_font(obj, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text(obj, ">Status");
        }
        {
            // Led_Calibration_status
            lv_obj_t *obj = lv_led_create(parent_obj);
            objects.led_calibration_status = obj;
            lv_obj_set_pos(obj, 292, 228);
            lv_obj_set_size(obj, 8, 9);
            lv_led_set_color(obj, lv_color_hex(0xff0b3759));
            lv_led_set_brightness(obj, 255);
        }
        {
            lv_obj_t *obj = lv_img_create(parent_obj);
            objects.obj1 = obj;
            lv_obj_set_pos(obj, 289, 28);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_img_set_src(obj, &img_battry);
            lv_img_set_zoom(obj, 150);
            lv_obj_add_event_cb(obj, event_handler_cb_tab_group_obj1, LV_EVENT_ALL, flowState);
            lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
        }
        {
            // Bluetooth_status_idle
            lv_obj_t *obj = lv_img_create(parent_obj);
            objects.bluetooth_status_idle = obj;
            lv_obj_set_pos(obj, 280, 30);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_img_set_src(obj, &img_bluetooth_idle);
            lv_img_set_zoom(obj, 180);
            lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICK_FOCUSABLE|LV_OBJ_FLAG_GESTURE_BUBBLE|LV_OBJ_FLAG_PRESS_LOCK|LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_SCROLL_CHAIN_HOR|LV_OBJ_FLAG_SCROLL_CHAIN_VER|LV_OBJ_FLAG_SCROLL_ELASTIC|LV_OBJ_FLAG_SCROLL_MOMENTUM|LV_OBJ_FLAG_SCROLL_WITH_ARROW|LV_OBJ_FLAG_SNAPPABLE);
        }
        {
            // Bluetooth_status_failed
            lv_obj_t *obj = lv_img_create(parent_obj);
            objects.bluetooth_status_failed = obj;
            lv_obj_set_pos(obj, 280, 30);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_img_set_src(obj, &img_bluetooth_failed);
            lv_img_set_zoom(obj, 180);
            lv_img_set_size_mode(obj, LV_IMG_SIZE_MODE_REAL);
        }
        {
            // Bluetooth_status_Success
            lv_obj_t *obj = lv_img_create(parent_obj);
            objects.bluetooth_status_success = obj;
            lv_obj_set_pos(obj, 280, 30);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_img_set_src(obj, &img_bluetooth_success);
            lv_img_set_zoom(obj, 180);
            lv_img_set_size_mode(obj, LV_IMG_SIZE_MODE_REAL);
        }
        {
            // Logs_text
            lv_obj_t *obj = lv_textarea_create(parent_obj);
            objects.logs_text = obj;
            lv_obj_set_pos(obj, 9, 38);
            lv_obj_set_size(obj, 297, 193);
            lv_textarea_set_max_length(obj, 512);
            lv_textarea_set_one_line(obj, false);
            lv_textarea_set_password_mode(obj, false);
            lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE);
        }
    }
    
    tick_screen_tab_group();
}

void tick_screen_tab_group() {
    void *flowState = getFlowState(0, 2);
    (void)flowState;
    {
        bool new_val = evalBooleanProperty(flowState, 0, 3, "Failed to evaluate Hidden flag");
        bool cur_val = lv_obj_has_flag(objects.low_battery_panel, LV_OBJ_FLAG_HIDDEN);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.low_battery_panel;
            if (new_val) {
                lv_obj_add_flag(objects.low_battery_panel, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_clear_flag(objects.low_battery_panel, LV_OBJ_FLAG_HIDDEN);
            }
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = evalBooleanProperty(flowState, 5, 3, "Failed to evaluate Hidden flag");
        bool cur_val = lv_obj_has_flag(objects.tabview, LV_OBJ_FLAG_HIDDEN);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.tabview;
            if (new_val) {
                lv_obj_add_flag(objects.tabview, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_clear_flag(objects.tabview, LV_OBJ_FLAG_HIDDEN);
            }
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = evalBooleanProperty(flowState, 34, 3, "Failed to evaluate Hidden flag");
        bool cur_val = lv_obj_has_flag(objects.auto_scan_option_panel, LV_OBJ_FLAG_HIDDEN);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.auto_scan_option_panel;
            if (new_val) {
                lv_obj_add_flag(objects.auto_scan_option_panel, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_clear_flag(objects.auto_scan_option_panel, LV_OBJ_FLAG_HIDDEN);
            }
            tick_value_change_obj = NULL;
        }
    }
    {
        const char *new_val = evalTextProperty(flowState, 36, 3, "Failed to evaluate Text in Textarea widget");
        const char *cur_val = lv_textarea_get_text(objects.steps_number);
        uint32_t max_length = lv_textarea_get_max_length(objects.steps_number);
        if (strncmp(new_val, cur_val, max_length) != 0) {
            tick_value_change_obj = objects.steps_number;
            lv_textarea_set_text(objects.steps_number, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        const char *new_val = evalTextProperty(flowState, 38, 3, "Failed to evaluate Text in Textarea widget");
        const char *cur_val = lv_textarea_get_text(objects.seconds_numbers);
        uint32_t max_length = lv_textarea_get_max_length(objects.seconds_numbers);
        if (strncmp(new_val, cur_val, max_length) != 0) {
            tick_value_change_obj = objects.seconds_numbers;
            lv_textarea_set_text(objects.seconds_numbers, new_val);
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = evalBooleanProperty(flowState, 45, 3, "Failed to evaluate Hidden flag");
        bool cur_val = lv_obj_has_flag(objects.auto_scan_process, LV_OBJ_FLAG_HIDDEN);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.auto_scan_process;
            if (new_val) {
                lv_obj_add_flag(objects.auto_scan_process, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_clear_flag(objects.auto_scan_process, LV_OBJ_FLAG_HIDDEN);
            }
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = evalBooleanProperty(flowState, 50, 3, "Failed to evaluate Hidden flag");
        bool cur_val = lv_obj_has_flag(objects.manual_scan_process, LV_OBJ_FLAG_HIDDEN);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.manual_scan_process;
            if (new_val) {
                lv_obj_add_flag(objects.manual_scan_process, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_clear_flag(objects.manual_scan_process, LV_OBJ_FLAG_HIDDEN);
            }
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = evalBooleanProperty(flowState, 61, 3, "Failed to evaluate Disabled state");
        bool cur_val = lv_obj_has_state(objects.sensibility_settings, LV_STATE_DISABLED);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.sensibility_settings;
            if (new_val) {
                lv_obj_add_state(objects.sensibility_settings, LV_STATE_DISABLED);
            } else {
                lv_obj_clear_state(objects.sensibility_settings, LV_STATE_DISABLED);
            }
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = evalBooleanProperty(flowState, 73, 3, "Failed to evaluate Hidden flag");
        bool cur_val = lv_obj_has_flag(objects.calibration_progress_bar, LV_OBJ_FLAG_HIDDEN);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.calibration_progress_bar;
            if (new_val) {
                lv_obj_add_flag(objects.calibration_progress_bar, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_clear_flag(objects.calibration_progress_bar, LV_OBJ_FLAG_HIDDEN);
            }
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = evalBooleanProperty(flowState, 74, 3, "Failed to evaluate Hidden flag");
        bool cur_val = lv_obj_has_flag(objects.calibration_button, LV_OBJ_FLAG_HIDDEN);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.calibration_button;
            if (new_val) {
                lv_obj_add_flag(objects.calibration_button, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_clear_flag(objects.calibration_button, LV_OBJ_FLAG_HIDDEN);
            }
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = evalBooleanProperty(flowState, 78, 3, "Failed to evaluate Hidden flag");
        bool cur_val = lv_obj_has_flag(objects.obj1, LV_OBJ_FLAG_HIDDEN);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.obj1;
            if (new_val) {
                lv_obj_add_flag(objects.obj1, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_clear_flag(objects.obj1, LV_OBJ_FLAG_HIDDEN);
            }
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = evalBooleanProperty(flowState, 79, 3, "Failed to evaluate Hidden flag");
        bool cur_val = lv_obj_has_flag(objects.bluetooth_status_idle, LV_OBJ_FLAG_HIDDEN);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.bluetooth_status_idle;
            if (new_val) {
                lv_obj_add_flag(objects.bluetooth_status_idle, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_clear_flag(objects.bluetooth_status_idle, LV_OBJ_FLAG_HIDDEN);
            }
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = evalBooleanProperty(flowState, 80, 3, "Failed to evaluate Hidden flag");
        bool cur_val = lv_obj_has_flag(objects.bluetooth_status_failed, LV_OBJ_FLAG_HIDDEN);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.bluetooth_status_failed;
            if (new_val) {
                lv_obj_add_flag(objects.bluetooth_status_failed, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_clear_flag(objects.bluetooth_status_failed, LV_OBJ_FLAG_HIDDEN);
            }
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = evalBooleanProperty(flowState, 81, 3, "Failed to evaluate Hidden flag");
        bool cur_val = lv_obj_has_flag(objects.bluetooth_status_success, LV_OBJ_FLAG_HIDDEN);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.bluetooth_status_success;
            if (new_val) {
                lv_obj_add_flag(objects.bluetooth_status_success, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_clear_flag(objects.bluetooth_status_success, LV_OBJ_FLAG_HIDDEN);
            }
            tick_value_change_obj = NULL;
        }
    }
    {
        bool new_val = evalBooleanProperty(flowState, 82, 3, "Failed to evaluate Hidden flag");
        bool cur_val = lv_obj_has_flag(objects.logs_text, LV_OBJ_FLAG_HIDDEN);
        if (new_val != cur_val) {
            tick_value_change_obj = objects.logs_text;
            if (new_val) {
                lv_obj_add_flag(objects.logs_text, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_clear_flag(objects.logs_text, LV_OBJ_FLAG_HIDDEN);
            }
            tick_value_change_obj = NULL;
        }
    }
}

typedef void (*tick_screen_func_t)();
tick_screen_func_t tick_screen_funcs[] = {
    tick_screen_boot_screen,
    tick_screen_main,
    tick_screen_tab_group,
};
void tick_screen(int screen_index) {
    tick_screen_funcs[screen_index]();
}
void tick_screen_by_id(enum ScreensEnum screenId) {
    tick_screen_funcs[screenId - 1]();
}

//
// Fonts
//

ext_font_desc_t fonts[] = {
#if LV_FONT_MONTSERRAT_8
    { "MONTSERRAT_8", &lv_font_montserrat_8 },
#endif
#if LV_FONT_MONTSERRAT_10
    { "MONTSERRAT_10", &lv_font_montserrat_10 },
#endif
#if LV_FONT_MONTSERRAT_12
    { "MONTSERRAT_12", &lv_font_montserrat_12 },
#endif
#if LV_FONT_MONTSERRAT_14
    { "MONTSERRAT_14", &lv_font_montserrat_14 },
#endif
#if LV_FONT_MONTSERRAT_16
    { "MONTSERRAT_16", &lv_font_montserrat_16 },
#endif
#if LV_FONT_MONTSERRAT_18
    { "MONTSERRAT_18", &lv_font_montserrat_18 },
#endif
#if LV_FONT_MONTSERRAT_20
    { "MONTSERRAT_20", &lv_font_montserrat_20 },
#endif
#if LV_FONT_MONTSERRAT_22
    { "MONTSERRAT_22", &lv_font_montserrat_22 },
#endif
#if LV_FONT_MONTSERRAT_24
    { "MONTSERRAT_24", &lv_font_montserrat_24 },
#endif
#if LV_FONT_MONTSERRAT_26
    { "MONTSERRAT_26", &lv_font_montserrat_26 },
#endif
#if LV_FONT_MONTSERRAT_28
    { "MONTSERRAT_28", &lv_font_montserrat_28 },
#endif
#if LV_FONT_MONTSERRAT_30
    { "MONTSERRAT_30", &lv_font_montserrat_30 },
#endif
#if LV_FONT_MONTSERRAT_32
    { "MONTSERRAT_32", &lv_font_montserrat_32 },
#endif
#if LV_FONT_MONTSERRAT_34
    { "MONTSERRAT_34", &lv_font_montserrat_34 },
#endif
#if LV_FONT_MONTSERRAT_36
    { "MONTSERRAT_36", &lv_font_montserrat_36 },
#endif
#if LV_FONT_MONTSERRAT_38
    { "MONTSERRAT_38", &lv_font_montserrat_38 },
#endif
#if LV_FONT_MONTSERRAT_40
    { "MONTSERRAT_40", &lv_font_montserrat_40 },
#endif
#if LV_FONT_MONTSERRAT_42
    { "MONTSERRAT_42", &lv_font_montserrat_42 },
#endif
#if LV_FONT_MONTSERRAT_44
    { "MONTSERRAT_44", &lv_font_montserrat_44 },
#endif
#if LV_FONT_MONTSERRAT_46
    { "MONTSERRAT_46", &lv_font_montserrat_46 },
#endif
#if LV_FONT_MONTSERRAT_48
    { "MONTSERRAT_48", &lv_font_montserrat_48 },
#endif
};

//
//
//

void create_screens() {
    
    eez_flow_init_fonts(fonts, sizeof(fonts) / sizeof(ext_font_desc_t));

// Set default LVGL theme
    lv_disp_t *dispp = lv_disp_get_default();
    lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), true, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    
    // Initialize screens
    eez_flow_init_screen_names(screen_names, sizeof(screen_names) / sizeof(const char *));
    eez_flow_init_object_names(object_names, sizeof(object_names) / sizeof(const char *));
    
    // Create screens
    create_screen_boot_screen();
    create_screen_main();
    create_screen_tab_group();
}