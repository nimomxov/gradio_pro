/**
 * @file adc_task.h
 * @brief ADS1115 acquisition task — Core 0, Priority 6.
 *
 * RESPONSIBILITY:
 *  - Reads ADS1115 in continuous mode at 50Hz via I2C
 *  - Computes per-sample quality score [0-100]
 *  - Pushes AdcSample_t into adc_queue (non-blocking)
 *  - Reports consecutive errors to system_monitor
 *  - Exposes driver handle for PGA changes by sensitivity_manager
 *
 * TIMING:
 *  vTaskDelayUntil() for precise 20ms period.
 *  Each read takes ~1ms @ 400kHz I2C.
 *  19ms margin per cycle — more than enough for I2C retries.
 *
 * TASK CREATION (from app_main):
 *  xTaskCreatePinnedToCore(adc_task, "adc_task", 4096, NULL, 6, &h, 0);
 */

#pragma once

#include "ads1115_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief FreeRTOS task entry point — ADC acquisition.
 * @param arg  Unused (pass NULL).
 */
void adc_task(void *arg);

/**
 * @brief Get pointer to the shared ADS1115 driver instance.
 *
 * Called by sensitivity_manager to change PGA gain at runtime.
 * Thread-safe: driver fields protected by the task's own access pattern
 * (only adc_task writes to driver registers).
 *
 * @return Pointer to driver, or NULL if hardware not yet initialized.
 */
ADS1115Driver_t *adc_task_get_driver(void);

/**
 * @brief Set oversampling count for next scan points.
 *
 * NORMAL MODE:
 *   Live:          8 samples  @ 860SPS = 16ms/point   +9dB
 *   Manual/Auto:  32 samples  @ 860SPS = 64ms/point  +15dB
 *
 * BOOST MODE (max depth + SNR):
 *   Live:         64 samples  @ 860SPS = 128ms/point +18dB
 *   Manual/Auto: 256 samples  @ 860SPS = 512ms/point +24dB
 *
 * Thread-safe: volatile uint16_t ensures cross-core visibility (Core 0 <-> Core 1).
 * @param count  Number of samples to average (8 to 256). Clamped internally.
 */
void    adc_task_set_oversample(uint16_t count);
uint16_t adc_task_get_oversample(void);

#ifdef __cplusplus
}
#endif