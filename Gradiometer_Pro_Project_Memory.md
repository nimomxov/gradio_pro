# Gradiometer Pro v2.0 — Project Memory File
> **Purpose:** Load this file at the start of any new session to instantly restore full context.
> **Last Updated:** May 2026 | **Status:** Near-Commercial Grade, Field-Ready

---

## 1. Project Identity

| Field | Value |
|---|---|
| **Device** | Gradiometer Pro — Archaeological Metal/Void/Tomb/Gold Scanner |
| **Hardware** | ESP32 Dual-Core + ADS1115 (16-bit ADC) + 2× FLC100 Fluxgate + ILI9341 320×240 SPI + HC-05 BT + BLE |
| **Framework** | ESP-IDF v5 + FreeRTOS + LVGL v8 + EEZ Studio UI |
| **BT Protocol** | HC-05 → OKM Visualizer3D.exe via `"value\r\n"` (uint16 [0..1024]) |


---

## 2. Architecture — Task Map

```
Core 0:  adc_task (P6) → signal_task (P5) → bt_sender_task (P4)
Core 1:  lvgl_task (P5) → ui_event_task (P4) → ble_logger_task (P3)

Queues:
  adc_queue    [16]  AdcSample_t (12B)     — raised from 4 to prevent drops
  result_queue [2]   ProcessedSample_t (32B) — overwrite semantics (latest wins)
  event_queue  [12]  SysEventMsg_t          — raised from 8

I2C Bus:
  Both cores access I2C (adc_task Core0, devcal Core1).
  g_lvgl_mutex protects LVGL.
  Phase 3C temporarily switches ADS1115 MUX for single-ended reads.
```

---

## 3. Signal Processing Pipeline (Core 0)

```
ADC decimation (n=8/32/64/256 oversamples)
  → Phase 3C Sensor Match Correction   ← NEW: corrected = raw + match_addend
  → Stage 1: Moving Average (window_size per mode)
  → Stage 2: Outlier Gate (σ-threshold, updates noise on RAW)
  → Stage 3: Kalman Filter (adaptive Q) OR EMA (if Kalman disabled)
  → Stage 3b: Spatial Filter (optional, scan mode only)
  → Stage 4: Baseline Subtract
       calibrated:   signal = filtered - baseline_adaptive
       uncalibrated: signal = filtered - initial_mean_warmup
       spatial:      signal = gradient (bypass baseline subtract)
  → Stage 5: Post-Kalman EMA (SKIPPED in VERY_HIGH/HIGH modes)
  → Stage 6: Dynamic Noise Gate = max(mode_floor×0.3, noise_rms×0.20)
  → Stage 7: Scale → output_scaled [0..1024]
  → Pipeline B → bt_value [0..1024]
```

### Pipeline B — 3 BT Modes
| Mode | Formula | Use |
|---|---|---|
| **RAW** | `signal + 512` (calibrated) OR `(raw+prev_raw)/2 - initial_mean + 512` (uncalibrated) | OKM Visualizer3D — zero processing |
| **NORMALIZED** | RAW + slow DC drift removal (BT_DRIFT_ALPHA=0.0005, τ≈100s) | Standard 3D scan |
| **ENHANCED** | NORMALIZED + range ×0.7 (tighter mapping = more contrast) | Weak deep targets |

**Spatial + RAW:** spatial gradient bypassed → absolute anomaly sent to BT (Visualizer3D needs amplitude for depth estimation).

---

## 4. Calibration System

### Soil Calibration (Main) — Optional
- **Mandatory before:** Nothing. User can skip — device enters **uncalibrated mode**.
- **Uncalibrated warmup:** Collects 100 stability-gated samples → computes `initial_mean` → freezes all 3 baselines. During warmup: output=512 neutral. UI shows "ثبّت الجهاز... N%".
- **After warmup:** `baseline_fixed = initial_mean`, signal centred correctly.
- **Algorithm:** Welford online + dual-track baseline (α_fast=0.001, α_slow=0.0002).

### Device Calibration Phases
| Phase | Description |
|---|---|
| Phase 1 | Basic sensor asymmetry (nominal_offset) |
| Phase 2 | Bar magnet — dynamic response test |
| Phase 3A | Horizontal compass — Earth field heading compensation |
| Phase 3B | Vertical rotation — body iron correction |
| **Phase 3C** ← NEW | Static sensor matching — 4000 GATED samples/channel via single-ended MUX |
| Phase 4 | Tilt compensation |
| Phase 5 | Final verification |

### Phase 3C Critical Details
```c
// ADS1115 MUX switching (only during 3C, adc_task blocked):
// AIN0-GND: mux_mask = 0x40  (upper FLC100)
// AIN1-GND: mux_mask = 0x50  (lower FLC100)

// Welford double-precision accumulation for N=4000
// Gate: |sample - running_mean| < 3σ  (rejects motion & targets)
// Bootstrap: first 100 ungated samples → estimate noise_floor for gate threshold

// Results:
sensor_offset2 = mean_AIN0 - mean_AIN1           // DC mismatch
sensor_gain2   = std_AIN0 / std_AIN1             // sensitivity ratio
ain1_dc_level  = mean_AIN1                        // for addend computation

// Precomputed runtime correction (O(1), applied in adc_task after decimation):
match_addend = (1 - gain2) × ain1_dc + offset2 × gain2
corrected_diff = raw_diff + match_addend
// Clamped to ±200 LSB (prevents extreme bias from tilt/gradient)

// Fallback if 3C not run: offset-only (nominal_offset), gain=1
// Never apply uncalibrated gain (could worsen signal)
```

---

## 5. Files Modified (vs Original)

### Critical Bug Fixes (Session 1)
| File | Fix |
|---|---|
| `gradiometer_types.h` | Queue depths ↑, SYS_EVT_CALIB_PHASE2, SignalNotifyFlag_t, ProcessedSample_t +confidence +signal_flags |
| `queue_manager.c` | Dead code removed, esp_timer timestamps, ISR auto-stamp |
| `signal_processor.c/.h` | bt_drift→struct (race condition), pipeline_bt receives `signal` not `filtered`, spatial bypass baseline |
| `signal_task.c` | sentinel -1.0f→SYS_EVT_CALIB_PHASE2, retry→TickType_t wraparound-safe |
| `bluetooth_sender.c/.h` | stop_flag safe deinit, uart_installed flag, scan point retry |
| `adc_task.c` | Phase 3C correction applied, div-by-zero guard, quality warmup ×2, esp_timer |
| `sensitivity_manager.c` | Deadlock fix: qm_event_send AFTER mutex release |
| `calib_engine.c` | INT16_MIN sentinel → bool valid[] (ADS1115 can legitimately return -32768) |
| `devcal_common.c` | All lv_timer_handler() → lvgl_refresh_safe() with g_lvgl_mutex |

### New Features (Sessions 2-5)
| File | Feature |
|---|---|
| `bar_chart.c/.h` | LV_EVENT_DRAW_MAIN custom widget, 8 colors, dead-zone, proper delete_cb |
| `devcal_phase3c.c/.h` | Phase 3C sensor matching (4000 gated samples, Welford, addend precompute) |
| `device_cal.c/.h` | devcal_apply_sensor_match(), sensor_offset2/gain2/match_addend in DeviceProfile_t |
| `ads1115_driver.c/.h` | ads1115_read_single_ended() for Phase 3C MUX switching |
| `signal_processor.c` | sp_apply_uncalibrated(), Dynamic Noise Gate, Post-Kalman EMA skip VERY_HIGH/HIGH |
| `signal_processor.c` | Spike Classifier (raw-based d/dt), Drift Reporter (fast-slow), Coherence, Confidence 0-90% |
| `signal_task.c` | Uncalibrated warmup (stability-gated), signal_task_is_calibrated(), get_warmup_pct() |
| `ui_event_task.c` | notifier_init/update (yellow triangle, Arabic messages, priority, auto-hide 3.5s) |

---

## 6. DSP Improvements Applied

### DeepSeek-Recommended (Accepted)
```c
// Dynamic Noise Gate (replaces fixed table values)
effective_gate = max(mode_min_gate × 0.3, sqrtf(variance) × 0.20)

// Post-Kalman EMA reduced
// VERY_HIGH/HIGH modes: SKIP entirely (max responsiveness for deep targets)
// Others: alpha × 0.15 (was × 0.30)

// Confidence Score (unified, replaces conf = SNR × 20)
snr_score [0..50] + dev_score [0..30] + mom_score [-8..+15] + stab_score [0..5]
× coherence_multiplier [0.3..1.0]
cap = 90% max
```

### ChatGPT/DeepSeek Rejected
| Suggestion | Reason |
|---|---|
| `noise_scale = 1000/variance` in ENHANCED | Pristine soil → scale=2000× → full saturation |
| `(lv_event_cb_t)free` as delete callback | Frees lv_event_t struct, not user data → crash |
| 3 layers in bar chart (shadow+highlight) | 240 lv_draw_rect/refresh → ESP32 CPU overload |
| `raw_differential + 512` in RAW | DC offset ~15000 LSB → immediate clip to 1024 |
| Spatial in BT RAW mode | Visualizer3D needs absolute amplitude for depth |
| Confidence score in 3D scan filtering | Observer must NEVER modify DSP pipeline |
| `gain2 × raw_diff` formula (wrong) | Amplifies target signal — only s2 should be corrected |
| Adaptive sensor matching during scan | Overlaps with stage_baseline_subtract → signal cancellation |

---

## 7. Signal Notification System

### Flags (bitmask in ProcessedSample_t.signal_flags)
```c
SIGNAL_FLAG_SPIKE      = 0x01  // |Δraw| > 5×noise_rms in 1 sample, disappears ≤2 samples
SIGNAL_FLAG_DRIFT      = 0x04  // |baseline_fast - baseline_slow| > 2 LSB/s (fast-slow method)
SIGNAL_FLAG_HIGH_NOISE = 0x04  // noise_rms > max(3×calibrated_floor, 1.0 LSB)
SIGNAL_FLAG_UNSTABLE   = 0x08  // stability_score < SP_STABILITY_THRESHOLD × 0.5
```

### Spike vs Target Classifier
```c
// Raw-based (NOT filtered — Kalman buries spikes before comparison)
d_raw = |raw[n] - raw[n-1]|
if (d_raw > 5×noise_rms):
    spike_consec++
    if spike_consec <= 2 → SPIKE_FLAG  (vanished quickly = EMI)
    if spike_consec >= 3 → no flag     (sustained = real target)

spike_rate_ema = 0.9 × prev + 0.1 × spike_event  // sustained EMI environment
```

### Confidence Formula
```c
conf = 100
conf -= spike_rate_ema × 30
conf -= max(0, (noise_rms/calibrated_floor - 1) × 20)  // capped 40
conf -= |baseline_fast - baseline_slow| / 3 × 10       // capped 20
conf -= (100 - stability_score) × 0.3                  // capped 20
conf × coherence = 1 - (var_short/var_long)            // [0.3, 1.0]
conf = min(conf, 90)  // never 100 — always leave margin
```

### UI Notifier (upper-left yellow triangle)
| Priority | Flag | Message | Border |
|---|---|---|---|
| 1 | SPIKE | تداخل كهربائي / EMI | Red |
| 2 | HIGH_NOISE | تربة مُعدِّنة — اخفض الحساسية | Orange |
| 3 | DRIFT | الجهاز يتكيف مع التربة | Yellow |
| 4 | UNSTABLE | حرِّك الجهاز ببطء أكثر | Amber |
| — | uncal warmup | ثبّت الجهاز... N% | Amber |
| — | uncal stable | Baseline تلقائي (512) | Grey |
- Auto-hides after 3.5s when flags clear
- Shows `[confidence%]` next to message
- 0xFE = warmup sentinel, 0xFF = uncal-stable sentinel

---

## 8. Bar Chart (Live Scan Display)

```
Widget: LV_EVENT_DRAW_MAIN on plain lv_obj (NOT lv_chart)
Architecture: bar_chart_create() inside objects.live_chart
              objects.live_scan_chart (EEZ widget) is hidden

Per sample: bar_chart_add_value(obj, output_scaled, deviation)
  → memmove(slots, slots+1) shifts all bars left
  → lv_obj_invalidate() triggers one full redraw

Dead zone: bars < 3px not drawn (noise suppression)

Colors (METAL ↑ / VOID ↓):
  METAL: dark-green→green→orange→red  (dev < 10 / < 30 / < 70 / ≥ 70 LSB)
  VOID:  pale-blue→light-blue→blue→deep-blue (same thresholds)
```

---

## 9. Countdown Timer (3D Scan)

```
3 layers:
  lv_arc  — shrinks with time (stopwatch style, green→yellow→red)
  lv_label — zoom+fade via lv_anim_t ONLY on second change (not every 20ms)
  lv_bar  — thin linear bar at bottom (fallback visual)

Animation callbacks:
  _anim_zoom_cb: 333→256 (130%→100%) in 220ms, ease_out
  _anim_opa_cb:  0→255 in 180ms, ease_in_out

Triggered ONLY when secs != s_cd_last_sec (avoids per-frame CPU waste)
```

---

## 10. Uncalibrated Mode Flow

```
User skips calibration → signal_task detects ST_IDLE + !calib_valid
  → sp_apply_uncalibrated() called ONCE
  → Warmup begins (100 stability-gated samples)
     Gate: stability_score < 40 → reject sample (bypass for first 20)
     Output: 512 neutral during warmup
     UI: "ثبّت الجهاز... N%"
  → Warmup done: baseline_fixed = initial_mean, Kalman seeded
     UI: "Baseline تلقائي (512)"
  → Normal processing with baseline = initial_mean
     output_scaled = clamp(signal + 512, 0, 1024)
  → RAW BT: (prev_raw + raw)/2 - initial_mean + 512  (2-tap FIR, no Kalman)

Fallback chain:
  Phase 3C done: corrected = raw + match_addend (offset + gain)
  Phase 3C skipped: corrected = raw - nominal_offset (offset only, gain=1)
  Not calibrated at all: raw passes through unchanged
```

---

## 11. Key Constants & Thresholds

```c
// Queue depths
QUEUE_DEPTH_ADC_SAMPLES = 16   // 250ms headroom at 62Hz
QUEUE_DEPTH_EVENTS      = 12

// Signal processor
SP_STABILITY_THRESHOLD  = 80.0f
SP_STABILITY_FAST       = 0.15f
SP_ADAPTIVE_UPDATE_RATE = 0.001f   // α_fast
BT_DRIFT_ALPHA          = 0.0005f  // τ≈100s

// Phase 3C
P3C_SAMPLES_TARGET      = 4000
P3C_GATE_SIGMA          = 3.0f
P3C_BOOTSTRAP_N         = 100
P3C_MAX_GAIN_RATIO      = 1.15f   // reject if > 15% mismatch
P3C_GAIN_DEADBAND       = 0.002f  // 0.2%
P3C_MATCH_ADDEND_CLAMP  = ±200 LSB

// Spike detection
SPIKE_K                 = 5.0f    // threshold multiplier
SPIKE_CONSEC_MAX        = 2       // spike if jump disappears ≤2 samples
TARGET_CONSEC_MIN       = 3       // target if sustained ≥3 samples
spike_rate_ema α        = 0.10f   // EMA for sustained EMI environment

// Noise gate
GATE_K                  = 0.20f   // 20% of noise_rms
ABSOLUTE_MIN_NOISE_RMS  = 1.0 LSB // prevents false HIGH_NOISE in pristine soil

// Confidence
CONFIDENCE_MAX          = 90%     // never show 100%
COHERENCE_MIN           = 0.3f    // minimum coherence multiplier

// Warmup
WARMUP_N                = 100
WARMUP_STABILITY_MIN    = 40.0f
WARMUP_STAB_BYPASS      = 20      // first 20 samples ungated

// UI Notifier
NOTIF_AUTO_HIDE_MS      = 3500
WARMUP_SENTINEL         = 0xFE
UNCAL_STABLE_SENTINEL   = 0xFF
```

---

## 12. Known Rejected Approaches (DO NOT REVISIT)

1. **`s_bt_drift` as static global** — fixed by moving into `SignalProcessor_t.bt_drift`
2. **`stability = -1.0f` sentinel** — fixed by `SYS_EVT_CALIB_PHASE2` event
3. **`INT16_MIN` sentinel in remove_outliers** — fixed by `bool valid[]` array
4. **`(lv_event_cb_t)free` delete callback** — fixed by `bar_chart_delete_cb()`
5. **`bt_val = pipeline_bt(sp, filtered)`** — fixed to pass `signal` (baseline-subtracted)
6. **Spatial filter subtracting baseline** — spatial gradient is already zero-centered
7. **`xTaskGetTickCount() * portTICK_PERIOD_MS`** — replaced with `esp_timer_get_time()/1000`
8. **`qm_event_send` inside mutex** — always release mutex BEFORE sending event
9. **`vTaskDelete` before queue cleanup** — use stop_flag + wait for self-delete
10. **Confidence > 90%** — cap at 90%, always leave margin

---

## 13. Future Roadmap (Agreed Not Yet Implemented)

| Feature | Trigger | Notes |
|---|---|---|
| IMU Motion Correlation | Add ICM-20948 | Replace scan_active flag surrogate |
| Temperature Compensation | NTC sensor | FLC100 offset drifts ~0.1 nT/°C |
| Scan Quality Controller | Next session | Pause scan if moving too fast / confidence too low |
| Spatial Pattern Analyzer | Advanced | Wide anomaly=deep, narrow=shallow |
| Kalman constant-velocity model | After IMU | Replaces random-walk model |

---

## 14. OKM Visualizer3D Integration Notes

- **Protocol:** `"512\r\n"` per scan step (uint16, UART via HC-05)
- **Baseline building:** Visualizer normalizes first 5-10 points as soil reference
- **Color mapping:** Values > baseline → yellow/red (metal), < baseline → blue (void)
- **Depth estimation:** Based on signal amplitude and scan width — do NOT compress RAW
- **TRUE RAW requirement:** No bt_range compression. Any scaling = destroys Visualizer's own normalization
- **Scan modes:** Manual (button per step) + Auto (countdown timer per step)
- **BLE:** Secondary channel for phone monitoring — can be disabled by user switch

---

*End of project memory file. Load at session start for instant context restoration.*
