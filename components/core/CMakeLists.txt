# ─── core component ───────────────────────────────────────────────────────
# gradiometer_types.h (header-only), queue_manager, signal_processor, signal_task
idf_component_register(
    SRCS
        "queue_manager.c"
        "signal_processor.c"
        "signal_task.c"
    INCLUDE_DIRS
        "."
        "../"
    REQUIRES
        freertos
        driver
        esp_common
        drivers           # signal_task uses BTSender_t (bluetooth_sender.h)
        calibration
        modes
        system_monitor
)