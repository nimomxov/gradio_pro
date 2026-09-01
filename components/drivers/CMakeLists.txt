# ─── drivers component ────────────────────────────────────────────────────
# ADS1115 I2C driver, ADC acquisition task, HC-05 Bluetooth sender
idf_component_register(
    SRCS
        "ads1115_driver.c"
        "adc_task.c"
        "bluetooth_sender.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        freertos
        driver          # uart, i2c, gpio
        esp_timer
        esp_common
        core
        system_monitor
)
