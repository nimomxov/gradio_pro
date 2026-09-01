cmake_minimum_required(VERSION 3.16)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# ─── Project-level CMakeLists.txt ─────────────────────────────────────────

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/components/core"
    "${CMAKE_CURRENT_SOURCE_DIR}/components/drivers"
    "${CMAKE_CURRENT_SOURCE_DIR}/components/calibration"
	"${CMAKE_CURRENT_SOURCE_DIR}/components/device_calibration"
    "${CMAKE_CURRENT_SOURCE_DIR}/components/modes"
    "${CMAKE_CURRENT_SOURCE_DIR}/components/system_monitor"
    "${CMAKE_CURRENT_SOURCE_DIR}/components/display"
    "${CMAKE_CURRENT_SOURCE_DIR}/components/ui"
)

# إخبار LVGL بمكان lv_conf.h
set(LV_CONF_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lv_conf.h" CACHE STRING "" FORCE)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(gradiometer_pro)
