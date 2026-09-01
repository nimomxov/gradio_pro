# ─── ble_logger component ────────────────────────────────────────────────────
# BLE Data Logger — ESP32 internal BLE → Mobile (CSV notifications)
# Works in parallel with HC-05 → OKM Visualizer3D (UART).

idf_component_register(
    SRCS
        "ble_data_logger.c"
    INCLUDE_DIRS
        "."
        "../"
    REQUIRES
        bt
        nvs_flash
        freertos
        esp_common
        core
)
