# ─── system_monitor component ─────────────────────────────────────────────
# Heap watchdog, stack overflow detection, fault code registry
idf_component_register(
    SRCS
        "system_monitor.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        freertos
        esp_common
        esp_system
        core
)