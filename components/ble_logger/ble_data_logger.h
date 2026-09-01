#pragma once
/**
 * @file ble_data_logger.h
 * @brief BLE Data Logger — ESP32 internal BLE → Mobile (Android/iOS)
 *
 * ARCHITECTURE:
 *   يعمل بالتوازي مع HC-05 → OKM Visualizer3D تماماً.
 *   HC-05 يرسل format ثنائي لـ OKM.
 *   BLE الداخلي يرسل CSV لتطبيق الهاتف.
 *
 * GATT SERVICE:
 *   Service UUID  : 4AFAFADE-1234-11EF-8A1C-2B7E8F6A9C3E
 *   Char UUID     : 4AFAFADE-1235-11EF-8A1C-2B7E8F6A9C3E
 *   Properties    : READ + NOTIFY
 *   Advertisement : "Gradiometer BLE"
 *
 * CSV FORMAT (per scan point):
 *   timestamp_ms,scan_mode,step,gradient_raw,gradient_filtered,
 *   baseline,deviation,bt_value,snr,stability,soil_type,sensitivity,
 *   boost,kalman,heading,noise_floor,quality_note
 *
 * ACTIVATION:
 *   Sends ONLY when:
 *     1. data_via_ble switch is ON
 *     2. Scan is active (Manual or Auto)
 *     3. A BLE client is connected and subscribed to notifications
 *
 * TASK:
 *   ble_logger_task — Core 1, Priority 3
 *   Receives BleLogPoint_t via FreeRTOS queue from ui_event_task.
 */

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "core/gradiometer_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════
 * SCAN POINT — data sent to BLE logger queue
 * ═══════════════════════════════════════════════════════════════════ */

typedef struct {
    uint32_t timestamp_ms;
    uint8_t  scan_mode;          ///< 0=manual  1=auto
    uint16_t step;               ///< scan step number
    int16_t  gradient_raw;       ///< raw differential (LSB)
    float    gradient_filtered;  ///< after DSP pipeline
    float    baseline;
    float    deviation;
    uint16_t bt_value;           ///< [0..1024] sent to OKM
    float    snr;
    float    stability;          ///< 0-100
    uint8_t  soil_type;          ///< SoilType_t
    uint8_t  sensitivity;        ///< SensitivityMode_t index
    uint8_t  boost;              ///< 1 = boost mode ON
    uint8_t  kalman;             ///< 1 = kalman ON
    uint8_t  heading;            ///< 0=N 1=E 2=S 3=W 4=disabled
    float    noise_floor;        ///< from last calibration
} BleLogPoint_t;

/* ═══════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialize BLE stack and GATT server. Call once from app_main
 *        after nvs_flash_init().
 * @return ESP_OK on success.
 */
esp_err_t ble_logger_init(void);

/**
 * @brief Start the FreeRTOS task (Core 1, Priority 3).
 *        Call after ble_logger_init().
 */
void ble_logger_task_start(void);

/**
 * @brief Enable or disable logging. Controlled by data_via_ble switch.
 *        When disabled, enqueued points are silently dropped.
 */
void ble_logger_set_enabled(bool enabled);

/**
 * @brief Enqueue a scan point for BLE transmission.
 *        Non-blocking — drops point if queue is full.
 *        Call from ui_event_task at each scan step.
 *
 * @param pt  Pointer to scan point data (copied into queue).
 */
void ble_logger_enqueue(const BleLogPoint_t *pt);

/**
 * @brief Returns true if a BLE client is connected and subscribed.
 */
bool ble_logger_is_connected(void);

/**
 * @brief Returns true if logging is enabled via switch.
 */
bool ble_logger_is_enabled(void);

#ifdef __cplusplus
}
#endif
