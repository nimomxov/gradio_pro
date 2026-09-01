#pragma once
#include "device_cal.h"
#ifdef __cplusplus
extern "C" {
#endif
/**
 * Phase 1 — Sensor Offset + Noise Floor
 * Duration: 30 seconds
 * Output:   nominal_offset, noise_floor, kalman_R
 */
void devcal_run_phase1(DeviceProfile_t *p);
#ifdef __cplusplus
}
#endif
