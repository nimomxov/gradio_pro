# ─── ui component ─────────────────────────────────────────────────────────
# EEZ Studio generated frontend + ui_event_task bridge

idf_component_register(
    SRCS
        # EEZ Studio generated files
        "screens.c"
        "styles.c"
        "images.c"
        "ui.c"
        "eez-flow.cpp"
        # Image data
        "ui_image_background1.c"
        "ui_image_battry.c"
        "ui_image_bluetooth_failed.c"
        "ui_image_bluetooth_idle.c"
        "ui_image_bluetooth_success.c"
        "ui_image_down_steps.c"
        "ui_image_live_scan_button.c"
        "ui_image_low_battery.c"
        "ui_image_mainbackground.c"
        "ui_image_scan3d_button.c"
        "ui_image_scan3d_photo.c"
        "ui_image_settings_button.c"
        "ui_image_up_arrow.c"
        "ui_image_boot_screen.c"
        # Backend-Frontend bridge
        "actions.c" 
        "boot_selfcheck.c"
        "bar_chart.c"
        "ui_event_task.c"
    INCLUDE_DIRS
        "."
        "../"
        "../../main/"
    REQUIRES
        freertos
        esp_common
        lvgl
        display
        core
        drivers
        calibration
        device_calibration
        ble_logger
        modes
        system_monitor
)