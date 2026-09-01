# Gradiometer Pro v2.0 — Session Summary
*للاستخدام في بداية جلسات جديدة لاسترجاع السياق الكامل*

---

## 1. الهوية والهدف

مرساد يطور **Gradiometer Pro v2.0** — جهاز مسح جيوفيزيائي احترافي للكشف عن المعادن والفراغات تحت الأرض.

- **Hardware:** ESP32 dual-core · 2× FLC100 fluxgate (45cm) · ADS1115 differential · ILI9341 320×240 TFT · XPT2046 touch
- **Connectivity:** HC-05 UART → OKM Visualizer3D (PC) · ESP32 BLE → mobile CSV
- **Framework:** ESP-IDF v5.5.3 · FreeRTOS · LVGL v8 · EEZ Studio · Windows toolchain
- **Build:** ~567KB / 1MB flash (45% free) · Bootloader 9%

---

## 2. معمارية الـ Tasks

```
CORE 0:                           CORE 1:
adc_task    Prio6  4096B          lvgl_task      Prio5  6144B
signal_task Prio5  4096B          ui_event_task  Prio4  4096B
bt_task     Prio4  2048B          ble_logger     Prio3  4096B
```

**Queues:**
- `adc_queue`    depth=4   AdcSample_t (12B)
- `result_queue` depth=2   ProcessedSample_t (30B) — overwrite mode
- `event_queue`  depth=8   SysEventMsg_t
- `bt_queue`     depth=32  BTMessage_t
- `ble_queue`    depth=8   BleLogPoint_t

---

## 3. هيكل الملفات الأساسية

```
components/
  core/
    gradiometer_types.h    ← zero deps, جميع الأنواع
    signal_processor.c/h   ← DSP pipeline كامل
    signal_task.c/h        ← orchestrator + volatile flags
    queue_manager.c/h

  drivers/
    adc_task.c/h           ← 860SPS + boost mode
    bluetooth_sender.c/h   ← HC-05 priority queue
    ads1115_driver.c/h

  calibration/calib_engine.c/h     ← 2-phase Welford soil cal
  device_calibration/
    device_cal.c/h         ← 5-phase NVS + CRC32 atomic save
    devcal_common.c/h + devcal_phase1-5.c/h

  ble_logger/ble_data_logger.c/h   ← GATT server + CSV

  modes/sensitivity_manager.c/h    ← AUTO engine + FSM + mutex

  ui/
    ui_event_task.c/h      ← 2440 lines: Audio + SmartDetect + SNR
    actions.c/h            ← 19 registered callbacks
    screens.c/h            ← EEZ Studio generated

main/app_main.c            ← g_bt_sender + g_sens_mgr globals
```

---

## 4. DSP Pipeline (signal_processor.c)

### Pipeline A — Screen/Buzzer
```
ADC raw → MA (O(1) running sum, window 4-64)
        → Outlier Gate (σ from noise_variance using RAW ← bug fixed)
        → Kalman 1D (adaptive Q 1-8×) OR EMA
        → Spatial Filter (pure gradient dG ← bug fixed, scan only)
        → Baseline Subtract (dual-track adaptive)
        → Post-Kalman EMA (light, α×0.3)
        → Noise Gate
        → Scale [0..1024]
```

### Pipeline B — HC-05/BT
```
Kalman output → Adaptive DC drift (α=0.0005)
              → k × noise_floor range (k: PRISTINE=3..EXTREME=7)
              → clamp [0..1] → ×1024 → uint16 → UART
```

### Bugs Fixed This Session (signal_processor.c)
1. **ma_sum not reset** in `sp_set_sensitivity()` → added `sp->ma_sum = 0.0f`
2. **update_noise(gated)** feedback loop → changed to `update_noise(raw, ma_out)`
3. **Spatial blend** `alpha*signal + (1-alpha)*dG` halved baseline → pure gradient `output = dG_sym`

---

## 5. sensitivity_manager.c — AUTO Engine

**معمارية صحيحة (مُهمة):**
- `sensitivity_manager` = decision layer فقط — لا يلمس ADC/DSP
- `signal_task` يستدعي `sens_manager_auto_update()` بعد كل `sp_process()`
- `g_sens_mgr` مُعرَّف في `app_main.c` كـ global

**خوارزمية القرار:**
```c
if (noise_ema >= 30) → VERY_LOW      // حماية أولاً
if (noise_ema >= 12) → LOW
if (SNR >= 6.0 && stability >= 85) → VERY_HIGH
if (SNR >= 3.0 && stability >= 70) → HIGH
else                               → MEDIUM
```

**FSM Step-down (لا قفز بأكثر من مستوى):**
- VERY_HIGH → HIGH فقط
- HIGH → MEDIUM فقط
- LOW → MEDIUM فقط
- MEDIUM → HIGH فقط (لا قفز لـ VERY_HIGH مباشرة)

**Hysteresis thresholds:**
- UP HIGH: SNR ≥ 3.0 | DOWN from HIGH: SNR < 2.0
- UP VERY_HIGH: SNR ≥ 6.0 | DOWN from VERY_HIGH: SNR < 4.5

**Protections:**
- EMA on noise_floor: alpha=0.05 (TC≈4s @50Hz)
- noise_floor clamp ≥ 0.02 (يمنع SNR انفجاري)
- Stability gate: `if (!stable) return` قبل القرار
- Time lock: 1200ms بين تغييرَين
- **Mutex** في `apply_mode()` يحمي من Core0/Core1 race

**Bugs Fixed:**
- `static s_auto` not reset → `memset(&s_auto, 0)` in `sens_manager_init()`
- SENS_MODE_INFO table كانت [4] → مُوسَّعة لـ [6] (VERY_HIGH + VERY_LOW)
- FSM exit logic خاطئ في VERY_HIGH → مُصحَّح
- race condition بلا mutex → أضفنا `SemaphoreHandle_t lock` في struct

---

## 6. SNR Panel + Smart Detection (ui_event_task.c)

### SNR Panel Colors
| SNR | Color | Label | Confidence |
|-----|-------|-------|------------|
| <1.0 | Green #2ECC71 | Clear soil | 0-20% |
| 1-2 | Yellow #F1C40F | Weak signal | 20-40% |
| 2-4 | Orange #E67E22 | Possible target | 40-80% |
| >4 | Red #E74C3C | Strong target! | 80-100% |
| !stable | Grey #7F8C8D | Moving... | 0% |

**Peak Hold:** `powf(0.98, dt×10)` per 100ms · Jerk: adaptive threshold = clamp(2×noise_floor, 0.5, 5.0)

### Smart Detection (5 طبقات)
```
A. Momentum EMA:     α=0.30, decay ×0.20 when !stable, ceiling ±3.0, dead-band 0.05
B. Void consecutive: 3 samples quiet / 4 samples noisy (noise_floor > 5 LSB)
C. Conf hysteresis:  لا Metal↔Void switch < 40% confidence
D. Time hysteresis:  لا switch قبل 300ms
E. Temporal Hold:    120-550ms (scales with confidence) + EMA fade ×0.92 + floor 10%
```

---

## 7. Audio Engine (ui_event_task.c)

**LEDC init once** — التحديثات بـ `ledc_set_duty()` فقط (~5μs بدل ~50μs).

| Mode | Freq | Pattern | Duty |
|------|------|---------|------|
| METAL | 800Hz | Pulse متقطع | f(confidence) 800-5120 |
| VOID | 200Hz | Continuous | EMA smoothed 600-4000 |
| SILENCE | — | Off | 0 |

**Pulse interval:** `500 - SNR²×15 - vel×50` ms · clamp [80..500ms]
**Velocity:** `vel = SNR - prev_SNR` · clamp [0..1.5]
**Void soft attack:** EMA α=0.15 · snap-to-zero before EMA on mute
**Silence hysteresis:** ON at |dev|>12 LSB · OFF at |dev|<8 LSB

---

## 8. bluetooth_sender.c — Data Integrity Fix

**الخطأ الأصلي:** `current_step++` يحدث دائماً حتى لو فشل الإرسال.
**الإصلاح:** `current_step++` فقط بعد تأكيد الإرسال. `return false` عند الفشل.
**`ui_event_task`** يتعامل مع `false` بإعادة العداد التنازلي (retry loop في `auto_scan_tick`).

---

## 9. signal_task.c — Fix Applied

**الخطأ:** `s_sig.bt = arg` قبل `memset` → يُمسح فوراً.
**الإصلاح:** حذف السطر المكرر. التسلسل الصحيح: `memset → bt = arg → sp_init`.

**TECHNICAL DEBT مُعلَّق:** Phase2 transition يُرسَل عبر `ProcessedSample_t.stability = -1.0f` كـ sentinel. يجب تحويله لـ `SYS_EVT_CALIB_PHASE2` عبر `event_queue` في ريفاكتور مستقبلي.

---

## 10. gradiometer_types.h — Fixes

- `GRAD_CALIB_DURATION_MS`: **10000 → 60000** (كان خطأ حرجاً — Phase1:20s + Phase2:40s)
- `AdcSample_t`: إعادة ترتيب 4B أولاً (timestamp_ms قبل ain0/ain1)
- `ProcessedSample_t`: إعادة ترتيب floats/uint32 أولاً، int16 أخيراً (Xtensa LX6 alignment)

---

## 11. ui_event_task.c — Fix Applied

**الخطأ:** `vTaskDelay(180ms)` داخل `handle_live_result()` التي تُستدعى داخل `UI_LOCK()` → يجمّد LVGL.
**الإصلاح:** النبضة الأولى داخل `handle_live_result`، النبضة الثانية في الـ main loop خارج `UI_LOCK()` عبر `phase2_beep` flag.

**Removed:** `#include "modes/sensitivity_manager.h"` (لم يكن مستخدماً — compilation guard).

---

## 12. Calibration Systems

### Soil (2-phase, before scan)
- Phase1 (20s still): baseline_fixed + noise_static → beep
- Phase2 (40s walk): noise_dynamic = std(signal − MA_16) → double beep
- noise_floor_final = max(static, dynamic)

### Device Cal (5-phase, GPIO16)
- Phase1 (60s): noise_floor + kalman_R
- Phase2 (160 samples): gradient range + kalman_Q
- Phase3 (16s×4): heading corrections N/E/S/W
- Phase4 (24s×2): tilt NS/EW
- Phase5 (30s): kalman_Q_final + detection_limit

**NVS atomic save:** blob → CRC32 → version → device_valid=1 (gate)
**Touch cal:** namespace "touch_cal"/"coeffs_v2" — NEVER touched by device cal reset

---

## 13. Pending Items (لا تنساها)

| Item | Priority |
|------|----------|
| ICM-20948 IMU integration | Medium |
| seconds_numbers → max 20s | Low |
| soil_type getter for BLE | Low |
| SYS_EVT_SENS_USER_REQ (distinguish UI from Auto engine) | Low |
| Phase2 sentinel → proper event queue | Low (Tech Debt) |
| bt_sender_deinit safe shutdown | Low (Tech Debt) |
| Field Validation Layer (VALID/UNCERTAIN/NOISE) | Future |
| Spatial Pattern Engine (width tracking) | Future |
| ADS1256 24-bit upgrade | Future |

---
