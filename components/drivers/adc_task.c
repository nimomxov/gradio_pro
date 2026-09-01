/**
 * @file adc_task.c
 * @brief ADC acquisition task — Core 0, Priority 6.
 *
 * RESPONSIBILITIES:
 *  1. Initialize ADS1115 driver (self-contained, no dependency on app_main)
 *  2. Read differential samples at 860SPS with software decimation
 *  3. Compute a quality score for each decimated sample
 *  4. Package into AdcSample_t and push to ADC queue
 *  5. Handle hardware faults gracefully (retry, report, continue)
 *
 * DECIMATION STRATEGY per mode:
 *   Live   (n=8):  8 × 2ms = 16ms/point  → 62.5Hz output, +9dB SNR
 *   Auto   (n=32): 32 × 2ms = 64ms/point → 15.6Hz output, +15dB SNR
 *   Manual (n=64): 64 × 2ms = 128ms/point→ 7.8Hz output,  +18dB SNR
 *   Boost  (n=256):256× 2ms = 512ms/point→ 1.9Hz output,  +24dB SNR
 *
 * 50/60Hz REJECTION:
 *   At 860SPS, each 2ms sample covers different phase of 50Hz cycle.
 *   Averaging N samples across full cycle(s) → 50Hz cancels out.
 *   n=8:  covers 16ms → almost full 50Hz cycle (20ms) → ~20dB rejection
 *   n=32: covers 64ms → 3+ full cycles → >40dB rejection
 */

#include "adc_task.h"
#include "ads1115_driver.h"
#include "core/queue_manager.h"
#include "system_monitor/system_monitor.h"
#include "device_calibration/device_cal.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "ADC_Task";

/* =========================================================================
 * TASK CONFIGURATION
 * ========================================================================= */

#define CONSECUTIVE_FAIL_LIMIT  10u
#define RECOVERY_PAUSE_MS       500u
#define QUALITY_WINDOW          8u     /* Samples for rolling quality estimate */
#define CLAMP_SIGMA             4.0f   /* Flag outlier if > 4σ from local mean */

/* =========================================================================
 * PRIVATE STATE
 * ========================================================================= */

static struct {
    ADS1115Driver_t adc;

    /* Quality estimation — rolling window */
    int16_t  quality_buf[QUALITY_WINDOW];
    uint8_t  quality_head;
    float    quality_mean;
    float    quality_std;
    bool     quality_ready;

    /* Oversampling 
     * uint16_t volatile: 
     * - uint16_t prevents wraparound to 0 when UI sends 256 (Boost Mode).
     * - volatile ensures cross-core visibility (written by Core 1 UI, 
     *          read by Core 0 ADC loop) without compiler caching. */
    volatile uint16_t oversample_count;   

    /* Fault tracking */
    uint32_t consecutive_errors;
    uint32_t total_samples;
    uint32_t total_errors;
    int16_t  last_good_value;
} s_adc = {0};

/* =========================================================================
 * QUALITY SCORING
 * ========================================================================= */

/**
 * Update rolling statistics and return quality score for a new sample.
 * Score is 0-100 where 100 = perfect quality.
 *
 * FIX: Warmup extended to 2×QUALITY_WINDOW.
 * With only QUALITY_WINDOW warmup, the buffer has zeros mixed with real
 * samples → std is inflated by (value-0) differences → sigma miscalculated
 * → valid strong signals scored as outliers in the first ~0.5 seconds.
 * 2× window ensures the buffer is fully populated with real samples.
 */
static uint8_t compute_quality(int16_t value)
{
    /* Populate rolling window */
    s_adc.quality_buf[s_adc.quality_head] = value;
    s_adc.quality_head = (s_adc.quality_head + 1) % QUALITY_WINDOW;

    /* FIX: Use 2× window as warmup — buffer must be fully populated first */
    if (s_adc.total_samples < (QUALITY_WINDOW * 2u)) {
        return 80u;  /* Stable default during warmup — not penalizing valid data */
    }

    s_adc.quality_ready = true;

    /* Compute local mean */
    int32_t sum = 0;
    for (uint8_t i = 0; i < QUALITY_WINDOW; i++) {
        sum += s_adc.quality_buf[i];
    }
    s_adc.quality_mean = (float)sum / (float)QUALITY_WINDOW;

    /* Compute local std dev */
    float var = 0.0f;
    for (uint8_t i = 0; i < QUALITY_WINDOW; i++) {
        float d = (float)s_adc.quality_buf[i] - s_adc.quality_mean;
        var += d * d;
    }
    s_adc.quality_std = sqrtf(var / (float)QUALITY_WINDOW);

    /* If signal is extremely stable (flat sensor), give perfect score */
    if (s_adc.quality_std < 0.1f) {
        return 100u;
    }

    /* Score based on deviation from local mean in sigma units */
    float deviation = fabsf((float)value - s_adc.quality_mean);
    float sigma     = deviation / s_adc.quality_std;

    if (sigma < 1.0f)      return 100u;
    else if (sigma < 2.0f) return 85u;
    else if (sigma < 3.0f) return 65u;
    else if (sigma < 4.0f) return 40u;
    else                   return 10u;  /* Spike — likely EMI or movement artifact */
}

/* =========================================================================
 * HARDWARE INITIALIZATION (runs inside task)
 * ========================================================================= */

static bool init_hardware(void)
{
    ads1115_driver_init(&s_adc.adc, I2C_NUM_0);

    esp_err_t ret = ads1115_driver_start(&s_adc.adc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADS1115 init failed — entering fault state");
        sysmon_report_fault(FAULT_ADC_NOT_FOUND, "ADC hardware init failed");
        return false;
    }

    ESP_LOGI(TAG, "ADC hardware ready. Starting 860SPS acquisition loop");
    return true;
}

/* =========================================================================
 * MAIN ACQUISITION LOOP
 * ========================================================================= */

#define ADC_INTER_SAMPLE_MS   2u    /* 860SPS: 1.16ms conv + I2C overhead */

static void acquisition_loop(void)
{
    uint16_t  prev_n = 0;   /* Detect mode change for throwaway */

    while (1) {
        uint16_t  n   = s_adc.oversample_count; 
        int32_t   acc = 0;
        uint16_t  good = 0; 
        int16_t   diff = 0;
        esp_err_t ret;

        /* ── Throwaway sample on mode change (MUX settling) ── */
        if (n != prev_n) {
            ads1115_read_differential(&s_adc.adc, &diff, NULL, NULL);
            vTaskDelay(pdMS_TO_TICKS(ADC_INTER_SAMPLE_MS));
            prev_n = n;
            ESP_LOGD(TAG, "Mode change n=%u — throwaway done", n);
        }

        /* ── Decimation: collect N samples @ 860SPS ── */
        for (uint16_t i = 0; i < n; i++) {
            ret = ads1115_read_differential(&s_adc.adc, &diff, NULL, NULL);
            if (ret == ESP_OK) {
                acc += diff;
                good++;
            } else {
                s_adc.total_errors++;
            }
            /* Wait for next 860SPS conversion.
             * NOTE: This vTaskDelay IS the pacing mechanism for ALL modes.
             * The total loop time dictates the final output Hz. */
            vTaskDelay(pdMS_TO_TICKS(ADC_INTER_SAMPLE_MS));
        }

        /* ── Acceptance: require at least half of reads to succeed ── */
        if (n == 0) {
            /* Defensive: oversample_count should never be 0 (clamped at set),
             * but guard here to prevent division by zero crash in field. */
            ESP_LOGW(TAG, "oversample_count=0 — resetting to 8");
            s_adc.oversample_count = 8;
            continue;
        }

        if (good >= (n / 2u) && good > 0u) {
            int16_t decimated = (int16_t)(acc / (int32_t)good);

            /* ── Phase 3C Sensor Match Correction ──────────────────────
             * Apply BEFORE quality scoring and queue send.
             * Removes the static DC offset between AIN0 and AIN1 sensors,
             * and corrects the gain ratio (typically within 0–2%).
             *
             * Formula: corrected = (raw − offset2) × gain2
             *   offset2 ≈ mean_AIN0 − mean_AIN1  (4000-sample calibration)
             *   gain2   ≈ std_AIN0  / std_AIN1   (sensitivity ratio)
             *
             * If Phase 3C not run, devcal_apply_sensor_match() falls back
             * to Phase 1 nominal_offset subtraction — safe in all cases.
             * ────────────────────────────────────────────────────────── */
            {
                int32_t corrected = devcal_apply_sensor_match((int32_t)decimated);
                /* Clamp to int16 range — extreme sensor mismatch guard */
                if      (corrected >  32767) corrected =  32767;
                else if (corrected < -32768) corrected = -32768;
                decimated = (int16_t)corrected;
            }

            s_adc.consecutive_errors = 0;
            s_adc.last_good_value    = decimated;
            s_adc.total_samples++;

            uint8_t quality = compute_quality(decimated);
            
            /* Boost quality score — prevent cascade using else-if */
            if (n >= 64 && quality < 95) {
                quality = (uint8_t)(quality * 1.1f);  
            } else if (n >= 32 && quality < 90) {
                quality = (uint8_t)(quality * 1.05f); 
            }
            if (quality > 100) quality = 100;

            AdcSample_t sample = {
                .ain0         = 0,
                .ain1         = 0,
                .differential = decimated,
                /* FIX: esp_timer_get_time()/1000 — immune to TickType_t overflow */
                .timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000LL),
                .oversamples  = (uint8_t)good,
                .quality      = quality,
            };

            qm_adc_send(&sample, 0); /* Non-blocking */

        } else {
            /* Too many failures in this decimation window */
            s_adc.consecutive_errors++;
            s_adc.total_errors++;

            if (s_adc.consecutive_errors >= CONSECUTIVE_FAIL_LIMIT) {
                ESP_LOGE(TAG, "%lu consecutive decimation failures — pausing %dms",
                         (unsigned long)s_adc.consecutive_errors,
                         RECOVERY_PAUSE_MS);
                sysmon_report_fault(FAULT_ADC_READ_ERROR,
                                    "Decimation: too many I2C failures");
                vTaskDelay(pdMS_TO_TICKS(RECOVERY_PAUSE_MS));
                s_adc.consecutive_errors = 0;
                prev_n = 0;  /* Force throwaway on recovery */
                ads1115_driver_start(&s_adc.adc);
            }
        }
        
        /* BUGFIX: Removed vTaskDelayUntil logic here. 
         * The inner vTaskDelay(2ms) * N perfectly handles the timing 
         * for both Live mode and Scan modes. Adding outer loop delays 
         * caused dead-code and timing skew. */
    }
}

/* =========================================================================
 * TASK ENTRY POINT
 * ========================================================================= */

void adc_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "ADC task started on Core %d, priority %d",
             (int)xPortGetCoreID(),
             (int)uxTaskPriorityGet(NULL));

    memset(&s_adc, 0, sizeof(s_adc));
    s_adc.oversample_count = 8;  /* default: live mode */

    /* Initialize hardware — retry up to 3 times on cold boot */
    bool hw_ok = false;
    for (int attempt = 0; attempt < 3; attempt++) {
        if (init_hardware()) {
            hw_ok = true;
            break;
        }
        ESP_LOGW(TAG, "Init attempt %d/3 failed — retrying in 1s", attempt + 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (!hw_ok) {
        ESP_LOGE(TAG, "ADC hardware failed to initialize after 3 attempts.");
        ESP_LOGE(TAG, "Check I2C wiring: SDA=GPIO%d SCL=GPIO%d", 21, 22);
        /* Task stays alive but does nothing — prevents watchdog/restart loops */
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    /* Run acquisition */
    acquisition_loop();

    vTaskDelete(NULL); /* Unreachable */
}

/* =========================================================================
 * PUBLIC ACCESSORS
 * ========================================================================= */

ADS1115Driver_t *adc_task_get_driver(void)
{
    return &s_adc.adc;
}

/* =========================================================================
 * OVERSAMPLE CONTROL — called by ui_event_task before scan
 * ========================================================================= */

void adc_task_set_oversample(uint16_t count)
{
    /* Proper clamping to prevent 256->0 wraparound bug */
    if (count < 8) count = 8;     /* minimum = 8 (live mode)         */
    if (count > 256) count = 256; /* maximum = 256 (boost mode)      */
    
    s_adc.oversample_count = count;
    float snr_db = 10.0f * log10f((float)count);
    float time_ms = count * 2.0f;
    ESP_LOGI("ADC", "Decimation: %u samples @ 860SPS = %.0fms/point  +%.1fdB SNR%s",
             count, time_ms, snr_db,
             count >= 128 ? "  [BOOST]" : "");
}

uint16_t adc_task_get_oversample(void)
{
    return s_adc.oversample_count;
}