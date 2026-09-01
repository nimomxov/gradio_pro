#pragma once
/**
 * @file gradiometer_types.h
 * @brief Central data type definitions for the Gradiometer Pro system.
 *
 * DESIGN RULES:
 *  - This file has ZERO dependencies on any other project file.
 *  - All inter-task data structures are defined here.
 *  - Never include FreeRTOS or ESP-IDF headers here ? types only.
 *  - All structs must be explicitly sized (no padding surprises).
 *
 * FIXES APPLIED (v2.1):
 *  1. QUEUE_DEPTH_ADC_SAMPLES raised from 4 to 16.
 *     Rationale: LVGL can hold Core1 for up to ~30ms during heavy redraws.
 *     At 62Hz ADC output, 30ms = ~2 samples. Depth 16 gives 250ms headroom
 *     ? covers any realistic UI spike without dropping archaeology scan data.
 *  2. Added SYS_EVT_CALIB_PHASE2 event ? replaces stability=-1.0f sentinel hack.
 *  3. GRAD_MIN_HEAP_BYTES increased to 12288 for BLE+BT coexistence safety.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* =========================================================================
 * SYSTEM CONSTANTS
 * ========================================================================= */

#define GRAD_ADC_SAMPLE_RATE_HZ     50u     ///< Target ADC reads per second
#define GRAD_BT_SEND_RATE_HZ        20u     ///< Bluetooth output rate
#define GRAD_CALIB_DURATION_MS      10000u  ///< 10-second calibration window
#define GRAD_CALIB_SAMPLES_TARGET   400u    ///< Samples collected in 10s @ 40Hz
#define GRAD_SLEEP_TIMEOUT_MS       30000u  ///< Auto-sleep after 30s idle
#define GRAD_OUTPUT_RANGE           1024u   ///< Full-scale output range
#define GRAD_OUTPUT_CENTER          512u    ///< Zero-signal center point

#define GRAD_FILTER_MAX_WINDOW      64u     ///< Maximum moving-average window
#define GRAD_CALIB_RING_SIZE        512u    ///< Ring buffer for calib samples

/* =========================================================================
 * ENUMERATIONS
 * ========================================================================= */

/**
 * @brief System operational state machine.
 */
typedef enum {
    SYS_STATE_BOOT          = 0,
    SYS_STATE_SELF_TEST     = 1,
    SYS_STATE_IDLE          = 2,
    SYS_STATE_CALIBRATING   = 3,
    SYS_STATE_ACTIVE        = 4,
    SYS_STATE_SLEEP         = 5,
    SYS_STATE_FAULT         = 6,
} SystemState_t;

/**
 * @brief Calibration phase sub-states.
 */
typedef enum {
    CALIB_PHASE_NONE        = 0,
    CALIB_PHASE_COLLECTING  = 1,
    CALIB_PHASE_COMPUTING   = 2,
    CALIB_PHASE_DONE        = 3,
    CALIB_PHASE_EXPIRED     = 4,
} CalibPhase_t;

/**
 * @brief Soil classification derived from calibration noise analysis.
 */
typedef enum {
    SOIL_TYPE_UNKNOWN       = 0,
    SOIL_TYPE_PRISTINE      = 1,  ///< std < 1.0  ? deepest target detection
    SOIL_TYPE_CLEAN         = 2,  ///< std 1-4
    SOIL_TYPE_MINERAL       = 3,  ///< std 4-12   ? common field condition
    SOIL_TYPE_NOISY         = 4,  ///< std 12-30  ? high iron content
    SOIL_TYPE_EXTREME       = 5,  ///< std > 30   ? volcanic/industrial
} SoilType_t;

/**
 * @brief Sensitivity mode ? controls filter aggressiveness.
 *
 * VERY_HIGH : ???? ?????? ? ???? ????? ????? ????? ????? ????
 * HIGH      : ?????? ????? ? ???? ?????? ????? ?????
 * MEDIUM    : ?????? ? ????????? ?????
 * LOW       : ?????? ?????? ? ???? ????????
 * VERY_LOW  : ???? ?????? ? ?????? ???? ????
 */
typedef enum {
    SENS_MODE_AUTO          = 0,
    SENS_MODE_VERY_HIGH     = 1,
    SENS_MODE_HIGH          = 2,
    SENS_MODE_MEDIUM        = 3,
    SENS_MODE_LOW           = 4,
    SENS_MODE_VERY_LOW      = 5,
} SensitivityMode_t;

/**
 * @brief Scan trigger source.
 */
typedef enum {
    SCAN_TRIGGER_MANUAL     = 0,
    SCAN_TRIGGER_TOUCH      = 1,
    SCAN_TRIGGER_AUTO       = 2,
} ScanTrigger_t;

/**
 * @brief Bluetooth connection state.
 */
typedef enum {
    BT_STATE_OFF            = 0,
    BT_STATE_CONNECTING     = 1,
    BT_STATE_CONNECTED      = 2,
    BT_STATE_ERROR          = 3,
} BtConnectionState_t;

/**
 * @brief Fault codes for hardware errors.
 */
typedef enum {
    FAULT_NONE              = 0x00,
    FAULT_ADC_NOT_FOUND     = 0x01,
    FAULT_ADC_READ_ERROR    = 0x02,
    FAULT_BT_INIT_FAILED    = 0x04,
    FAULT_QUEUE_OVERFLOW    = 0x08,
    FAULT_STACK_OVERFLOW    = 0x10,
    FAULT_LOW_MEMORY        = 0x20,
} FaultCode_t;

/* =========================================================================
 * CORE DATA STRUCTURES
 * ========================================================================= */

/**
 * @brief Raw ADC sample ? produced by ADC Task, consumed by Signal Task.
 * Kept small: FreeRTOS queue copy must be fast (real-time constraint).
 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;   ///< esp_timer_get_time()/1000 at capture       (4B)
    int16_t  ain0;           ///< ADS1115 channel 0 (raw counts)             (2B)
    int16_t  ain1;           ///< ADS1115 channel 1 (raw counts)             (2B)
    int16_t  differential;   ///< ain1 - ain0 (pre-computed)                 (2B)
    uint8_t  oversamples;    ///< Number of HW reads averaged                (1B)
    uint8_t  quality;        ///< 0-100 quality estimate                     (1B)
} AdcSample_t;               // 12 bytes total

/**
 * @brief Signal notification flags ? bitmask output from sp_process().
 *
 * Designed as a READ-ONLY observer ? these flags never modify the DSP pipeline.
 * They classify conditions for the UI notifier only.
 *
 * SPIKE:      |?signal| > 5?noise_rms in a single sample = hardware/EMI artifact
 *             Distinguished from TARGET which requires ?3 sustained samples.
 * DRIFT:      baseline drift rate > threshold = soil condition change or
 *             temperature drift. Notifies user: "???? ????? ?? ??????"
 * HIGH_NOISE: noise_rms > 3?calibrated_floor = very mineralized soil or EMI area.
 * UNSTABLE:   stability_score < SP_STABILITY_THRESHOLD ? sensor moving too fast.
 */
typedef enum {
    SIGNAL_FLAG_NONE       = 0x00,
    SIGNAL_FLAG_SPIKE      = 0x01,  ///< Single-sample spike ? EMI/hardware artifact
    SIGNAL_FLAG_DRIFT      = 0x02,  ///< Baseline drifting ? soil change / temperature
    SIGNAL_FLAG_HIGH_NOISE = 0x04,  ///< Excessive noise ? mineralized soil or EMI
    SIGNAL_FLAG_UNSTABLE   = 0x08,  ///< Signal unstable ? moving too fast
} SignalNotifyFlag_t;

/**
 * @brief Processed signal result ? produced by Signal Task, consumed by UI Task.
 */
typedef struct __attribute__((packed)) {
    float    filtered_value;   ///< After all DSP processing                 (4B)
    float    baseline;         ///< Current effective baseline               (4B)
    float    deviation;        ///< filtered_value - baseline                (4B)
    float    snr;              ///< Signal-to-noise ratio estimate           (4B)
    float    stability;        ///< 0.0-100.0 stability score               (4B)
    float    noise_rms;        ///< sqrtf(noise_variance) ? for smart float precision (4B)
    float    bt_float_value;   ///< Pipeline B output as float (VIS3D_FLOAT mode)     (4B)
    uint32_t timestamp_ms;     ///< Time of processing                      (4B)
    uint16_t output_scaled;    ///< Mapped to [0, 1024] ? Screen only       (2B)
    uint16_t bt_value;         ///< Pipeline B output [0..1024] ? BT/BLE    (2B)
    int16_t  raw_differential; ///< Original differential before filtering   (2B)
    uint8_t  signal_flags;     ///< SignalNotifyFlag_t bitmask               (1B)
    uint8_t  confidence;       ///< Signal confidence score 0?100            (1B)
} ProcessedSample_t;           // 40 bytes total

/**
 * @brief Calibration statistics ? computed at end of calibration window.
 */
typedef struct {
    float    mean;
    float    std_dev;
    float    variance;
    float    min_val;
    float    max_val;
    float    peak_to_peak;
    float    drift_rate;

    SoilType_t        soil_type;
    SensitivityMode_t recommended_mode;
    uint16_t          samples_collected;

    uint8_t  window_size;
    float    alpha_smooth;
    float    outlier_sigma;

    uint32_t timestamp_ms;
    bool     is_valid;
} CalibResult_t;

/**
 * @brief Scan point ? one measurement in a scan session.
 */
typedef struct {
    uint16_t        output_value;
    float           deviation;
    float           snr;
    ScanTrigger_t   trigger;
    uint32_t        timestamp_ms;
} ScanPoint_t;

/**
 * @brief System event types for inter-task signaling.
 *
 * NOTE: SYS_EVT_CALIB_PHASE2 replaces the old stability=-1.0f sentinel hack.
 * It is sent by signal_task when calibration enters the walking phase (Phase 2),
 * and received by ui_event_task to update the calibration screen accordingly.
 */
typedef enum {
    SYS_EVT_NONE            = 0,
    SYS_EVT_BTN_SCAN        = 1,  ///< Physical scan button pressed (from ISR)
    SYS_EVT_TOUCH_CALIB     = 2,  ///< User requested calibration from UI
    SYS_EVT_TOUCH_SCAN      = 3,  ///< Touch-UI scan button
    SYS_EVT_CALIB_DONE      = 4,  ///< Calibration engine finished (data=recommended_mode)
    SYS_EVT_SENS_CHANGE     = 5,  ///< User changed sensitivity mode
    SYS_EVT_SLEEP_REQ       = 6,  ///< Power manager requesting sleep
    SYS_EVT_WAKE_REQ        = 7,  ///< Wake from sleep
    SYS_EVT_FAULT           = 8,  ///< Hardware fault detected
    SYS_EVT_CALIB_PHASE2    = 9,  ///< Calibration entered Phase 2 (walking phase)
} SystemEvent_t;

/**
 * @brief System event message ? sent via event queue.
 */
typedef struct {
    SystemEvent_t   event;
    uint32_t        data;    ///< Event-specific payload (cast as needed)
    uint32_t        timestamp_ms;
} SysEventMsg_t;

/* =========================================================================
 * QUEUE CONFIGURATION
 *
 * QUEUE_DEPTH_ADC_SAMPLES = 16:
 *   ADC outputs at ~62Hz (Live) or ~2Hz (Boost). Signal task processes at
 *   same rate. LVGL on Core1 can spike up to 30ms. At 62Hz, 30ms = ~2 samples.
 *   Depth 16 = 250ms headroom at 62Hz. Prevents ANY drop during archaeology
 *   scans, even with heavy UI redraws. Memory cost: 16 ? 12B = 192 bytes.
 * ========================================================================= */

#define QUEUE_DEPTH_ADC_SAMPLES     16u   /* raised from 4 ? prevents drops during UI spikes */
#define QUEUE_DEPTH_PROCESSED       2u    /* UI needs only latest value */
#define QUEUE_DEPTH_EVENTS          12u   /* raised from 8 ? handles BT+BLE+button bursts */

/* =========================================================================
 * SAFETY LIMITS
 * ========================================================================= */

#define GRAD_MIN_HEAP_BYTES         12288u  ///< Alert if heap drops below this (BLE+BT need margin)
#define GRAD_ADC_TIMEOUT_MS         50u
#define GRAD_I2C_RETRY_COUNT        3u
#define GRAD_CALIB_MIN_VALID_RATIO  0.90f
