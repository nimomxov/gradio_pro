# ─── modes component ──────────────────────────────────────────────────────
# Sensitivity manager: coordinates PGA (hardware) + DSP filter (software)
idf_component_register(
    SRCS
        "sensitivity_manager.c"
    INCLUDE_DIRS
        "."
        "../"
    REQUIRES
        freertos
        esp_common
        core
        drivers         # ads1115_driver for PGA changes
)