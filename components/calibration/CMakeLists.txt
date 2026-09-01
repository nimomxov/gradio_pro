# ─── calibration component ────────────────────────────────────────────────
# 10-second calibration engine: Welford algorithm, Chauvenet criterion,
# soil classification, drift estimation
idf_component_register(
    SRCS
        "calib_engine.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        freertos
        esp_common
        core
)
