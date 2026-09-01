#pragma once
/**
 * @file devcal_phase3c.h
 * @brief Phase 3C — Static Sensor Matching (DC Offset + Gain).
 *
 * Collects 4000 single-ended samples per sensor in the static Earth field
 * to compute the precise DC offset and gain ratio between AIN0 and AIN1.
 *
 * Why 4000 samples?
 *   Standard error of the mean = σ / √N
 *   Typical FLC100 noise floor σ ≈ 2–8 LSB.
 *   With N=4000: SE = 8/√4000 ≈ 0.13 LSB — sub-LSB accuracy.
 *   Phase 1 uses ~200 samples → SE ≈ 0.57 LSB (4× less accurate).
 *
 * Results stored in DeviceProfile_t:
 *   sensor_offset2 = mean_AIN0 − mean_AIN1  (DC difference, LSB)
 *   sensor_gain2   = std_AIN0  / std_AIN1   (sensitivity ratio)
 *
 * Applied in adc_task.c BEFORE queuing each decimated sample:
 *   corrected = (raw − sensor_offset2) × sensor_gain2
 */

#include "device_cal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Run Phase 3C sensor matching.
 *
 * Blocking — runs from ui_event_task (Core 1).
 * adc_task (Core 0) remains blocked on calibration event.
 * ADS1115 is temporarily switched to single-ended mode, then restored.
 *
 * @param p  Device profile to write results into.
 */
void devcal_run_phase3c(DeviceProfile_t *p);

#ifdef __cplusplus
}
#endif
