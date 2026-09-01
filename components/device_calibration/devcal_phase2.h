#pragma once
#include "device_cal.h"
#ifdef __cplusplus
extern "C" {
#endif
/**
 * Phase 2 — Sensor Matching + Dynamic Range
 * Uses a bar magnet from two directions (North/South poles)
 * Output: gradient_N, gradient_S, dynamic_range, asymmetry,
 *         sensitivity, kalman_Q
 */
void devcal_run_phase2(DeviceProfile_t *p);
#ifdef __cplusplus
}
#endif
