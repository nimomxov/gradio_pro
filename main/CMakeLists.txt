# ─── main component ───────────────────────────────────────────────────────
idf_component_register(
    SRCS
        "app_main.c"
    INCLUDE_DIRS
        "."
        "../"
        "../components/display"
        "../components/device_calibration"
    REQUIRES
        nvs_flash
        driver
        esp_timer
        freertos
        esp_system
        # project components
        core
        drivers
        calibration
        modes
        system_monitor
        display
        device_calibration
        ble_logger
        ui
        # LVGL
        lvgl
)
