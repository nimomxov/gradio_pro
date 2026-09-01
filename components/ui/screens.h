#ifndef EEZ_LVGL_UI_SCREENS_H
#define EEZ_LVGL_UI_SCREENS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

// Screens

enum ScreensEnum {
    _SCREEN_ID_FIRST = 1,
    SCREEN_ID_BOOT_SCREEN = 1,
    SCREEN_ID_MAIN = 2,
    SCREEN_ID_TAB_GROUP = 3,
    _SCREEN_ID_LAST = 3
};

typedef struct _objects_t {
    lv_obj_t *boot_screen;
    lv_obj_t *main;
    lv_obj_t *tab_group;
    lv_obj_t *boot_background;
    lv_obj_t *boot_spinner;
    lv_obj_t *boot_check_events;
    lv_obj_t *live_scan_button;
    lv_obj_t *scan3_dbutton;
    lv_obj_t *settings_button;
    lv_obj_t *low_battery_panel;
    lv_obj_t *tabview;
    lv_obj_t *live_scan_tab;
    lv_obj_t *live_chart;
    lv_obj_t *obj0;
    lv_obj_t *current_value_panel;
    lv_obj_t *baseline_panel;
    lv_obj_t *baseline_text;
    lv_obj_t *values_text;
    lv_obj_t *live_scan_chart;
    lv_obj_t *snr_panel;
    lv_obj_t *snr_progress_bar;
    lv_obj_t *snr_val;
    lv_obj_t *confidence_value;
    lv_obj_t *info_for_user;
    lv_obj_t *smart_detection;
    lv_obj_t *scan3_dtab;
    lv_obj_t *manual_scan_button;
    lv_obj_t *auto_scan_button;
    lv_obj_t *scan_mode;
    lv_obj_t *scan_directions;
    lv_obj_t *auto_scan_option_panel;
    lv_obj_t *steps_number;
    lv_obj_t *seconds_numbers;
    lv_obj_t *autoscan_start_button;
    lv_obj_t *auto_scan_start_button;
    lv_obj_t *up_seconds;
    lv_obj_t *down_seconds;
    lv_obj_t *up_steps;
    lv_obj_t *down_steps;
    lv_obj_t *auto_scan_process;
    lv_obj_t *auto_scan_cancel_button;
    lv_obj_t *manual_scan_process;
    lv_obj_t *manual_scan_cancel_button;
    lv_obj_t *state_infos;
    lv_obj_t *data_via_ble;
    lv_obj_t *settings_tab;
    lv_obj_t *manual_sens_label;
    lv_obj_t *manual_sensibility_switch;
    lv_obj_t *sensibility_settings;
    lv_obj_t *logs_button;
    lv_obj_t *view_logs;
    lv_obj_t *reset_device_calib;
    lv_obj_t *reset;
    lv_obj_t *advanced_settings_panel;
    lv_obj_t *kalman_filter;
    lv_obj_t *boost_mode;
    lv_obj_t *low_batt;
    lv_obj_t *kalman_spatial;
    lv_obj_t *stability_gate;
    lv_obj_t *calibration_progress_bar;
    lv_obj_t *calibration_button;
    lv_obj_t *calibration_label_statut;
    lv_obj_t *led_calibration_status;
    lv_obj_t *obj1;
    lv_obj_t *bluetooth_status_idle;
    lv_obj_t *bluetooth_status_failed;
    lv_obj_t *bluetooth_status_success;
    lv_obj_t *logs_text;
} objects_t;

extern objects_t objects;

void create_screen_boot_screen();
void tick_screen_boot_screen();

void create_screen_main();
void tick_screen_main();

void create_screen_tab_group();
void tick_screen_tab_group();

void tick_screen_by_id(enum ScreensEnum screenId);
void tick_screen(int screen_index);

void create_screens();

#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_SCREENS_H*/