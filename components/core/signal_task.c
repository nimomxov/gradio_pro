/**
 * @file signal_task.c
 * @brief Signal processing task ? Core 0, Priority 5 (Production Level).
 *
 * RESPONSIBILITIES:
 *  1. Block on ADC queue waiting for samples (efficient, no busy-wait)
 *  2. Route samples: calibration engine OR signal processor depending on state
 *  3. Forward processed results to UI via result queue
 *  4. Handle system events: sensitivity change, calibration request, sleep
 *
 * ARCHITECTURE (Single Writer Principle):
 *  - signal_task is the ONLY task allowed to write to SensitivityManager.
 *  - UI task requests sensitivity changes via SYS_EVT_SENS_CHANGE event.
 *  - Loop-prevention: guard checks effective_mode before calling apply_mode.
 *
 * STATE MACHINE:
 *   BOOT ? [init] ? IDLE
 *   IDLE ? [calib request] ? CALIBRATING ? [done] ? ACTIVE
 *   ACTIVE ? [calib request] ? CALIBRATING ? [done] ? ACTIVE
 *   ACTIVE/IDLE ? [sleep event] ? SLEEPING ? [wake event] ? ACTIVE
 *   ANY ? [fault] ? FAULT (log + continue where possible)
 */

#include "signal_task.h"
#include "signal_processor.h"
#include "modes/sensitivity_manager.h"
#include "calibration/calib_engine.h"
#include "queue_manager.h"
#include "system_monitor/system_monitor.h"
#include "drivers/bluetooth_sender.h"
#include "drivers/adc_task.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

static const char *TAG = "SigTask";

/* ???? ?? SensitivityManager ????????? ?? app_main */
extern SensitivityManager_t g_sens_mgr;

/* =========================================================================
 * CALIBRATION RETRY CONFIGURATION
 * ========================================================================= */
#define CALIB_MAX_RETRIES        3u      /* ???? ??? ??????? ????? ????????     */
#define CALIB_RETRY_DELAY_MS     2000u   /* delay ??? ????????? (ms)             */
#define CALIB_RETRY_MAX_DELAY_MS 10000u  /* ???? delay (exponential backoff)     */

/* =========================================================================
 * TASK STATE
 * ========================================================================= */

typedef enum {
    ST_IDLE        = 0,
    ST_CALIBRATING = 1,
    ST_ACTIVE      = 2,
    ST_SLEEPING    = 3,
    ST_FAULT       = 4,
} SignalTaskState_t;

static struct {
    SignalTaskState_t state;
    SignalProcessor_t processor;
    CalibEngine_t     calib_engine;
    CalibResult_t     last_calib;
    bool              calib_valid;
    SensitivityMode_t pending_mode;
    bool              mode_change_pending;
    /* -- scan -- */
    BTSender_t       *bt;
    bool              auto_scan_active;
    ProcessedSample_t last_result;
    bool              last_result_valid;
    bool              boost_mode;
    /* -- calibration retry -- */
    uint8_t           calib_retry_count;
    uint32_t          calib_retry_delay_ms;   /* ?????? ?? ?? ?????? ????? */
    bool              calib_waiting;           /* true = ????? ADC ??? ????? ??? delay */
    TickType_t        calib_retry_after_tick;  /* FIX: TickType_t (not ms) ? safe wraparound comparison */
} s_sig = {0};

/* -- Filter control flags (written by UI task, read by signal task) --
 * Using volatile for cross-core visibility on ESP32.
 * Single bool writes are atomic on Xtensa ? no mutex needed.     */
static volatile bool    s_kalman_req        = false;
static volatile bool    s_kalman_req_valid  = false;
static volatile bool    s_spatial_req       = false;
static volatile bool    s_spatial_req_valid = false;
static volatile bool    s_scan_mode_req     = false;
static volatile bool    s_scan_mode_valid   = false;
static volatile uint8_t s_bt_mode_req       = 0;
static volatile bool    s_bt_mode_valid     = false;
static volatile uint8_t s_heading_req       = 4;
static volatile bool    s_boost_req         = false;
static volatile bool    s_boost_valid       = false;
static volatile bool    s_heading_valid     = false;
static volatile bool    s_heading_comp_req  = false;
static volatile bool    s_heading_comp_val  = false;

/* =========================================================================
 * EVENT HANDLING
 * ========================================================================= */

static void handle_events(void)
{
    SysEventMsg_t msg;

    while (qm_event_receive(&msg, 0)) {
        switch (msg.event) {

            case SYS_EVT_TOUCH_CALIB:
                if (s_sig.state == ST_ACTIVE || s_sig.state == ST_IDLE) {
                    s_sig.calib_waiting        = false;   /* ????? ?? retry wait ???? */
                    calib_engine_start(&s_sig.calib_engine);
                    s_sig.state = ST_CALIBRATING;
                    s_sig.calib_retry_count    = 0;
                    s_sig.calib_retry_delay_ms = CALIB_RETRY_DELAY_MS;
                    ESP_LOGI(TAG, "? CALIBRATING");
                }
                break;

            case SYS_EVT_BTN_SCAN:
                if (!s_sig.auto_scan_active &&
                    s_sig.last_result_valid  &&
                    s_sig.bt != NULL         &&
                    bt_is_ready(s_sig.bt)) {

                    if (sp_get_bt_mode(&s_sig.processor) == BT_MODE_VIS3D_FLOAT) {
                        bt_enqueue_vis3d_scan(s_sig.bt,
                                              s_sig.last_result.bt_float_value,
                                              s_sig.last_result.noise_rms);
                    } else {
                        uint16_t scan_val = s_sig.last_result.bt_value;
                        if (s_sig.processor.heading_comp_enabled &&
                            s_sig.processor.scan_heading < 4) {
                            float corr = s_sig.processor.heading_corrections[
                                         s_sig.processor.scan_heading];
                            int32_t v = (int32_t)scan_val + (int32_t)(corr * 512.0f /
                                        (s_sig.processor.bt_range_fixed > 0 ?
                                         s_sig.processor.bt_range_fixed : 50.0f));
                            if (v < 0) v = 0;
                            if (v > 1024) v = 1024;
                            scan_val = (uint16_t)v;
                        }
                        bt_enqueue_scan(s_sig.bt, scan_val);
                    }
                    ESP_LOGI(TAG, "Manual scan: bt_val=%u  screen_val=%u",
                             (unsigned)s_sig.last_result.bt_value,
                             (unsigned)s_sig.last_result.output_scaled);
                } else if (s_sig.auto_scan_active) {
                    ESP_LOGD(TAG, "BTN_SCAN ignored ? auto scan active");
                }
                break;

            case SYS_EVT_TOUCH_SCAN:
                if (msg.data == 0xFFFFFFFFu) {
                    s_sig.auto_scan_active = false;
                    ESP_LOGI(TAG, "Scan cancelled");
                } else if (msg.data == 0) {
                    if (s_sig.last_result_valid &&
                        s_sig.bt != NULL        &&
                        bt_is_ready(s_sig.bt)) {

                        if (sp_get_bt_mode(&s_sig.processor) == BT_MODE_VIS3D_FLOAT) {
                            bt_enqueue_vis3d_scan(s_sig.bt,
                                                  s_sig.last_result.bt_float_value,
                                                  s_sig.last_result.noise_rms);
                        } else {
                            uint16_t scan_val2 = s_sig.last_result.bt_value;
                            if (s_sig.processor.heading_comp_enabled &&
                                s_sig.processor.scan_heading < 4) {
                                float corr2 = s_sig.processor.heading_corrections[
                                              s_sig.processor.scan_heading];
                                int32_t v2 = (int32_t)scan_val2 + (int32_t)(corr2 * 512.0f /
                                             (s_sig.processor.bt_range_fixed > 0 ?
                                              s_sig.processor.bt_range_fixed : 50.0f));
                                if (v2 < 0) v2 = 0;
                                if (v2 > 1024) v2 = 1024;
                                scan_val2 = (uint16_t)v2;
                            }
                            bt_enqueue_scan(s_sig.bt, scan_val2);
                        }
                        ESP_LOGI(TAG, "Touch manual scan: bt_val=%u  screen_val=%u",
                                 (unsigned)s_sig.last_result.bt_value,
                                 (unsigned)s_sig.last_result.output_scaled);
                    }
                } else {
                    s_sig.auto_scan_active = true;
                    ESP_LOGI(TAG, "Auto scan started");
                }
                break;

            /* ===========================================================
             * Sensitivity Change ? ?? UI ?? ?? Auto Engine
             *
             * Loop-prevention: ??? ??? ????? ??????? = ????? ??????
             * (effective_mode)? ?????? apply_mode ???? ?????? ????
             * ?? ???? ????? apply_mode ???? SYS_EVT_SENS_CHANGE ??????.
             * =========================================================== */
            case SYS_EVT_SENS_CHANGE:
                if ((SensitivityMode_t)msg.data != sens_manager_get_effective(&g_sens_mgr)) {
                    if (msg.data == (uint32_t)SENS_MODE_AUTO) {
                        sens_manager_set_auto(&g_sens_mgr);
                        ESP_LOGI(TAG, "Sensitivity ? AUTO");
                    } else {
                        sens_manager_set_user(&g_sens_mgr, (SensitivityMode_t)msg.data);
                        ESP_LOGI(TAG, "Sensitivity ? %d", (int)msg.data);
                    }
                }
                /* ?????? ????? ??? DSP ?????? ??????? */
                s_sig.pending_mode        = (SensitivityMode_t)msg.data;
                s_sig.mode_change_pending = true;
                break;

            case SYS_EVT_SLEEP_REQ:
                if (s_sig.state == ST_CALIBRATING) {
                    ESP_LOGW(TAG, "Sleep requested during calibration ? aborting");
                    /* ????? ??????? ?? ?????? ??????? */
                    s_sig.calib_retry_count = CALIB_MAX_RETRIES;
                }
                s_sig.state = ST_SLEEPING;
                ESP_LOGI(TAG, "? SLEEPING");
                break;

            case SYS_EVT_WAKE_REQ:
                if (s_sig.state == ST_SLEEPING) {
                    s_sig.state = s_sig.calib_valid ? ST_ACTIVE : ST_IDLE;
                    ESP_LOGI(TAG, "? %s", s_sig.calib_valid ? "ACTIVE" : "IDLE");
                }
                break;

            case SYS_EVT_FAULT:
                ESP_LOGW(TAG, "Fault event received: 0x%lX", (unsigned long)msg.data);
                break;

            default:
                break;
        }
    }

    /* Apply pending sensitivity change to DSP */
    if (s_sig.mode_change_pending && s_sig.state == ST_ACTIVE) {
        sp_set_sensitivity(&s_sig.processor,
                           s_sig.pending_mode,
                           s_sig.calib_valid ? &s_sig.last_calib : NULL);
        s_sig.mode_change_pending = false;
    }
}

/* =========================================================================
 * CALIBRATION ROUTING
 * ========================================================================= */

static void process_calibrating(const AdcSample_t *sample)
{
    /* -- Non-blocking retry delay ----------------------------------
     * ??? ??? ?? ??? ????????? ???? ?? ????? ?????.
     * ??? ?? ????? ? ???? ?????? ????. ??? task ?? ?????.
     * ??? ?????   ? ??? ????? calib_engine ????.
     * ----------------------------------------------------------- */
    if (s_sig.calib_waiting) {
        /* FIX: Use subtraction-based comparison ? safe across TickType_t wraparound.
         * Old code: xTaskGetTickCount() < calib_retry_after_ms
         *   Problem: mixes ticks and ms, and < breaks at rollover (~49 days).
         * New code: (now - deadline) cast to signed ? negative means not yet elapsed.
         * This is the standard FreeRTOS-recommended pattern. */
        TickType_t now  = xTaskGetTickCount();
        TickType_t diff = now - s_sig.calib_retry_after_tick;
        if ((int32_t)diff < 0) {
            return;   /* delay not yet elapsed ? skip this sample, keep UI responsive */
        }
        ESP_LOGI(TAG, "Calibration retry delay elapsed ? restarting");
        s_sig.calib_waiting = false;
        calib_engine_start(&s_sig.calib_engine);
    }

    CalibEngineStatus_t status = calib_engine_feed(&s_sig.calib_engine, sample);

    switch (status) {

        case CALIB_STATUS_IN_PROGRESS: {
            uint8_t pct = calib_engine_get_progress(&s_sig.calib_engine);

            /* FIX: SYS_EVT_CALIB_PHASE2 replaces the old stability=-1.0f sentinel.
             * The sentinel was an anti-pattern: stability is a float [0..100],
             * using -1.0f as a protocol signal broke type safety and could cause
             * ui_event_task to display garbage stability values on receipt.
             * Now we use a proper event through the event queue. The result queue
             * still gets a progress update so the calibration bar keeps moving. */
            if (calib_engine_phase2_started(&s_sig.calib_engine)) {
                qm_event_send(SYS_EVT_CALIB_PHASE2, 0);
                ESP_LOGI(TAG, "? Calibration Phase 2 started ? walk slowly");
                /* Also send a progress update so bar advances to ~50% */
                ProcessedSample_t progress_update = {
                    .output_scaled = 50u,
                    .stability     = 50.0f,   /* valid range ? no sentinel */
                    .timestamp_ms  = sample->timestamp_ms,
                };
                qm_result_send_overwrite(&progress_update);
                break;
            }

            ProcessedSample_t progress_update = {
                .output_scaled = (uint16_t)pct,
                .stability     = (float)pct,
                .timestamp_ms  = sample->timestamp_ms,
            };
            qm_result_send_overwrite(&progress_update);
            break;
        }

        case CALIB_STATUS_DONE: {
            calib_engine_get_result(&s_sig.calib_engine, &s_sig.last_calib);
            s_sig.calib_valid = s_sig.last_calib.is_valid;

            if (s_sig.calib_valid) {
                s_sig.calib_retry_count    = 0;
                s_sig.calib_retry_delay_ms = CALIB_RETRY_DELAY_MS;
                s_sig.calib_waiting        = false;
                sp_apply_calibration(&s_sig.processor, &s_sig.last_calib);
                s_sig.state = ST_ACTIVE;

                ESP_LOGI(TAG, "=== Calibration Complete ===");
                ESP_LOGI(TAG, "  Baseline:    %.2f", s_sig.last_calib.mean);
                ESP_LOGI(TAG, "  Noise ?:     %.3f", s_sig.last_calib.std_dev);
                ESP_LOGI(TAG, "  Soil type:   %d",   (int)s_sig.last_calib.soil_type);
                ESP_LOGI(TAG, "  Recommended: %d",   (int)s_sig.last_calib.recommended_mode);

                /* ????? CALIB_DONE ?? ????? ??? sensitivity */
                qm_event_send(SYS_EVT_CALIB_DONE,
                              (uint32_t)s_sig.last_calib.recommended_mode);
            } else {
                ESP_LOGE(TAG, "Calibration result invalid ? staying IDLE");
                s_sig.state = ST_IDLE;
            }
            break;
        }

        case CALIB_STATUS_ERROR:
            s_sig.calib_retry_count++;
            if (s_sig.calib_retry_count <= CALIB_MAX_RETRIES) {
                ESP_LOGW(TAG, "Calibration error ? retry %u/%u after %lums",
                         s_sig.calib_retry_count,
                         CALIB_MAX_RETRIES,
                         (unsigned long)s_sig.calib_retry_delay_ms);

                /* Non-blocking delay: ???? ??? ????????.
                 * process_calibrating ??????? ??????? ??? ????? ?????.
                 * handle_events() ????? ?? ????? ? UI ?? ?????. */
                s_sig.calib_waiting        = true;
                s_sig.calib_retry_after_tick = xTaskGetTickCount()
                                             + pdMS_TO_TICKS(s_sig.calib_retry_delay_ms);

                /* Exponential backoff: ???? ??? delay ???????? ??????? */
                s_sig.calib_retry_delay_ms *= 2u;
                if (s_sig.calib_retry_delay_ms > CALIB_RETRY_MAX_DELAY_MS) {
                    s_sig.calib_retry_delay_ms = CALIB_RETRY_MAX_DELAY_MS;
                }
                /* calib_engine_start ??????? ??? ?????? ??? delay ?? process_calibrating */
            } else {
                ESP_LOGE(TAG, "Calibration failed after %u retries ? reverting to IDLE",
                         CALIB_MAX_RETRIES);
                s_sig.calib_retry_count    = 0;
                s_sig.calib_retry_delay_ms = CALIB_RETRY_DELAY_MS;
                s_sig.calib_waiting        = false;   /* ????? ???????? */
                s_sig.state                = ST_IDLE;
            }
            break;

        default:
            break;
    }
}

/* =========================================================================
 * NORMAL PROCESSING ROUTING
 * ========================================================================= */

static void process_active(const AdcSample_t *sample)
{
    ProcessedSample_t result;
    bool valid = sp_process(&s_sig.processor, sample, &result);

    if (valid) {
        s_sig.last_result       = result;
        s_sig.last_result_valid = true;

        qm_result_send_overwrite(&result);

        /* -- AUTO Sensitivity Engine ----------------------------------
         * ??????? ????? Boost Mode:
         *   ?? Boost (256 samples) ??? noise ????? ??????? (~18dB).
         *   ?? ????? Auto ????? ???? noise ????? ???? ????? ??? sensitivity
         *   ?? VERY_HIGH ??? ?? ???????? ???? ???? ????? ?? ???????.
         * ----------------------------------------------------------- */
        if (!s_sig.boost_mode) {
            uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            sens_manager_auto_update(
                &g_sens_mgr,
                sqrtf(s_sig.processor.noise_variance > 0.0f
                      ? s_sig.processor.noise_variance : 0.0f),
                sp_get_snr(&s_sig.processor),
                sp_get_stability(&s_sig.processor),
                sp_is_stable(&s_sig.processor),
                now_ms);
        }
    }
}

/* =========================================================================
 * FILTER CONTROL API ? called from UI task (Core 1)
 * Single bool/uint8 writes are atomic on Xtensa ? no mutex needed.
 * ========================================================================= */

void signal_task_set_kalman(bool enable)
{
    s_kalman_req       = enable;
    s_kalman_req_valid = true;
}

void signal_task_set_spatial(bool enable)
{
    s_spatial_req       = enable;
    s_spatial_req_valid = true;
    if (enable) {
        s_kalman_req       = true;
        s_kalman_req_valid = true;
    }
}

void signal_task_set_scan_mode(bool scanning)
{
    s_scan_mode_req   = scanning;
    s_scan_mode_valid = true;
}

bool signal_task_kalman_enabled(void)  { return sp_kalman_is_enabled(&s_sig.processor);  }
bool signal_task_spatial_enabled(void) { return sp_spatial_is_enabled(&s_sig.processor); }

void signal_task_set_heading(uint8_t heading)
{
    s_heading_req   = (heading <= 4) ? heading : 4;
    s_heading_valid = true;
}

void signal_task_set_heading_comp(bool enable)
{
    s_heading_comp_req = enable;
    s_heading_comp_val = true;
}

void signal_task_set_boost(bool enable)
{
    s_boost_req   = enable;
    s_boost_valid = true;
}

bool signal_task_boost_enabled(void) { return s_sig.boost_mode; }

void signal_task_load_heading_corrections(const float corrections[4])
{
    if (corrections) {
        sp_load_heading_corrections(&s_sig.processor, corrections);
    }
}

void signal_task_set_bt_mode(uint8_t mode)
{
    if (mode > 2) mode = 0;
    s_bt_mode_req   = mode;
    s_bt_mode_valid = true;
}

uint8_t signal_task_get_bt_mode(void) { return (uint8_t)sp_get_bt_mode(&s_sig.processor); }

bool signal_task_is_calibrated(void) { return s_sig.calib_valid; }

uint8_t signal_task_get_warmup_pct(void)
{
    if (s_sig.calib_valid) return 100u;   /* calibrated: warmup irrelevant */
    if (s_sig.processor.uncalib_warmup_done) return 100u;
    /* During warmup: return percentage 0..99 */
    uint8_t n = s_sig.processor.uncalib_warmup_n;
    return (uint8_t)((n >= 100u) ? 99u : n);
}

/* -- Apply pending filter requests (called from inside signal_task loop) -- */
static void apply_pending_filter_requests(void)
{
    if (s_kalman_req_valid) {
        sp_set_kalman(&s_sig.processor, s_kalman_req);
        s_kalman_req_valid = false;
    }
    if (s_spatial_req_valid) {
        sp_set_spatial(&s_sig.processor, s_spatial_req);
        s_spatial_req_valid = false;
    }
    if (s_scan_mode_valid) {
        sp_set_scan_mode(&s_sig.processor, s_scan_mode_req);
        s_scan_mode_valid = false;
    }
    if (s_bt_mode_valid) {
        sp_set_bt_mode(&s_sig.processor, (BtOutputMode_t)s_bt_mode_req);
        s_bt_mode_valid = false;
    }
    if (s_heading_valid) {
        sp_set_heading(&s_sig.processor, s_heading_req);
        s_heading_valid = false;
    }
    if (s_heading_comp_val) {
        sp_set_heading_comp(&s_sig.processor, s_heading_comp_req);
        s_heading_comp_val = false;
    }
    if (s_boost_valid) {
        s_sig.boost_mode = s_boost_req;
        s_boost_valid    = false;

        /* ????? ??? ADC oversampling ????? ??? ????? Boost */
        adc_task_set_oversample(s_sig.boost_mode ? 256u : 8u);

        ESP_LOGI(TAG, "Boost mode: %s", s_sig.boost_mode ? "ON (256 samples)" : "OFF (8 samples)");
    }
}

/* =========================================================================
 * TASK ENTRY POINT
 * ========================================================================= */

void signal_task(void *arg)
{
    ESP_LOGI(TAG, "Signal task started on Core %d, priority %d",
             (int)xPortGetCoreID(), (int)uxTaskPriorityGet(NULL));

    /* Initialize all state to zero, then set non-zero fields.
     * s_sig.bt must be assigned AFTER memset ? not before.
     * Any assignment before memset would be silently overwritten. */
    memset(&s_sig, 0, sizeof(s_sig));
    s_sig.bt                   = (BTSender_t *)arg;
    s_sig.calib_retry_delay_ms = CALIB_RETRY_DELAY_MS;
    sp_init(&s_sig.processor);
    calib_engine_init(&s_sig.calib_engine);
    s_sig.state = ST_IDLE;

    ESP_LOGI(TAG, "Ready. Waiting for calibration or samples.");

    while (1) {
        handle_events();
        apply_pending_filter_requests();

        if (s_sig.state == ST_SLEEPING) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        AdcSample_t sample;
        bool got_sample = qm_adc_receive(&sample, 100);

        if (!got_sample) {
            continue;
        }

        switch (s_sig.state) {

            case ST_CALIBRATING:
                process_calibrating(&sample);
                break;

            case ST_ACTIVE:
                process_active(&sample);
                break;

            case ST_IDLE:
                /* -- Uncalibrated mode ---------------------------------------
                 * User skipped calibration ? apply safe uncalibrated defaults
                 * ONCE on first sample, then process normally.
                 * baseline=0 ? output_scaled = signal+512 (centred at midscale)
                 * This lets users test the device and compare with/without
                 * calibration using Visualizer3D or the live bar chart.
                 * ----------------------------------------------------------- */
                if (!s_sig.calib_valid && !s_sig.processor.calibrated) {
                    sp_apply_uncalibrated(&s_sig.processor);
                    ESP_LOGI(TAG, "? ACTIVE (uncalibrated ? baseline=0, output=signal+512)");
                }
                process_active(&sample);
                break;

            case ST_FAULT:
                break;   /* Drain queue without processing */

            default:
                break;
        }
    }

    vTaskDelete(NULL);
}