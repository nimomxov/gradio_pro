/**
 * @file sensitivity_manager.c
 * @brief Sensitivity mode management — implementation (BUGFIXED).
 */

#include "sensitivity_manager.h"
#include "core/queue_manager.h"
#include "freertos/FreeRTOS.h"    
#include "freertos/semphr.h"           
#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static const char *TAG = "SensMgr";


/* ── حالة Engine (داخلية) ── */
typedef struct {
    uint32_t          last_change_ms;
    SensitivityMode_t last_applied;
    float             noise_ema;
    bool              initialised;
} AutoEngineState_t;

static AutoEngineState_t s_auto = {0};
/* =========================================================================
 * MODE INFO TABLE
 * ========================================================================= */
const SensModeInfo_t SENS_MODE_INFO[6] = {
    /* [0] AUTO */
    { .mode = SENS_MODE_AUTO, .name = "AUTO", .description = "Adaptive — engine selects based on soil noise + SNR", .pga = ADS1115_PGA_2048MV, .lsb_uv = 62.5f },
    /* [1] VERY_HIGH */
    { .mode = SENS_MODE_VERY_HIGH, .name = "VERY HIGH", .description = "Pristine soil only — std < 1.0 — deepest targets", .pga = ADS1115_PGA_1024MV, .lsb_uv = 31.25f },
    /* [2] HIGH */
    { .mode = SENS_MODE_HIGH, .name = "HIGH", .description = "Clean soil — deep targets and voids", .pga = ADS1115_PGA_1024MV, .lsb_uv = 31.25f },
    /* [3] MEDIUM */
    { .mode = SENS_MODE_MEDIUM, .name = "MEDIUM", .description = "General purpose — balanced noise and depth", .pga = ADS1115_PGA_2048MV, .lsb_uv = 62.5f },
    /* [4] LOW */
    { .mode = SENS_MODE_LOW, .name = "LOW", .description = "Mineralized soil — std 12–30", .pga = ADS1115_PGA_4096MV, .lsb_uv = 125.0f },
    /* [5] VERY_LOW */
    { .mode = SENS_MODE_VERY_LOW, .name = "VERY LOW", .description = "Extreme interference — volcanic/industrial — std > 30", .pga = ADS1115_PGA_4096MV, .lsb_uv = 125.0f },
};

/* =========================================================================
 * PRIVATE HELPERS
 * ========================================================================= */

static esp_err_t apply_mode(SensitivityManager_t *mgr, SensitivityMode_t mode)
{
    if (!mgr->initialized) return ESP_ERR_INVALID_STATE;

    SensitivityMode_t effective = mode;
    if (effective == SENS_MODE_AUTO) {
        effective = mgr->recommended_mode;
        if (effective == SENS_MODE_AUTO) {
            effective = SENS_MODE_MEDIUM;
        }
    }
    if ((uint8_t)effective >= 6) effective = SENS_MODE_MEDIUM;

    const SensModeInfo_t *info = &SENS_MODE_INFO[(uint8_t)effective];

    ESP_LOGI(TAG, "Applying mode: %s — PGA ±%.3fV (%.2f µV/LSB)",
             info->name,
             info->pga == ADS1115_PGA_1024MV ? 1.024f :
             info->pga == ADS1115_PGA_2048MV ? 2.048f : 4.096f,
             info->lsb_uv);

    /* Apply hardware PGA change while holding mutex — I2C op, safe from signal_task */
    if (mgr->adc_driver) {
        esp_err_t ret = ads1115_set_pga(mgr->adc_driver, info->pga);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set PGA: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    /* Update struct state while still holding mutex */
    mgr->current_mode   = mode;
    mgr->effective_mode = effective;

    /* FIX: Capture event data BEFORE releasing mutex, then send AFTER.
     *
     * Old code called qm_event_send() while holding mgr->lock.
     * qm_event_send() is non-blocking (timeout=0) so it CANNOT deadlock
     * by itself. However, if the event queue is full, signal_task may be
     * trying to acquire mgr->lock (via sens_manager_auto_update) while
     * holding the event queue slot — creating a lock-order inversion.
     *
     * Pattern: copy data → release lock → send event.
     * This is the standard "lock then copy, release then act" pattern. */
    return ESP_OK;
    /* Caller sends the event after releasing lock — see wrappers below */
}

/* =========================================================================
 * LIFECYCLE
 * ========================================================================= */

void sens_manager_init(SensitivityManager_t *mgr, ADS1115Driver_t *adc_driver)
{
    memset(mgr, 0, sizeof(SensitivityManager_t));
    mgr->current_mode     = SENS_MODE_MEDIUM;
    mgr->recommended_mode = SENS_MODE_MEDIUM;
    mgr->effective_mode   = SENS_MODE_MEDIUM;
    mgr->user_override    = false;
    mgr->adc_driver       = adc_driver;
    
    /* BUGFIX: Initialize RTOS Mutex (Requires SemaphoreHandle_t lock; in .h file) */
    if (mgr->lock == NULL) {
        mgr->lock = xSemaphoreCreateMutex();
    }

    mgr->initialized      = true;

    /* BUGFIX: Reset static auto-engine state on re-init to prevent stale data */
    memset(&s_auto, 0, sizeof(AutoEngineState_t));
    s_auto.last_applied = SENS_MODE_MEDIUM; /* Match default */

    ESP_LOGI(TAG, "Initialized. Default: MEDIUM");
}

/* =========================================================================
 * MODE CONTROL
 * ========================================================================= */

esp_err_t sens_manager_apply_recommendation(SensitivityManager_t *mgr, SensitivityMode_t mode)
{
    if (xSemaphoreTake(mgr->lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "apply_recommendation: mutex timeout — skipping");
        return ESP_ERR_TIMEOUT;
    }

    mgr->recommended_mode = mode;
    esp_err_t ret          = ESP_OK;
    SensitivityMode_t eff  = mgr->effective_mode;  /* capture before release */

    if (mgr->user_override) {
        ESP_LOGI(TAG, "Calibration recommends %s — keeping user override: %s",
                 SENS_MODE_INFO[(uint8_t)mode].name,
                 SENS_MODE_INFO[(uint8_t)mgr->current_mode].name);
    } else {
        ret = apply_mode(mgr, mode);
        eff = mgr->effective_mode;  /* updated by apply_mode */
    }

    xSemaphoreGive(mgr->lock);  /* FIX: release BEFORE event send — prevents lock-order inversion */

    if (ret == ESP_OK && !mgr->user_override) {
        bool sent = qm_event_send(SYS_EVT_SENS_CHANGE, (uint32_t)eff);
        if (!sent) {
            ESP_LOGW(TAG, "Event queue full — sensitivity event delayed");
        }
    }
    return ret;
}

esp_err_t sens_manager_set_user(SensitivityManager_t *mgr, SensitivityMode_t mode)
{
    if (xSemaphoreTake(mgr->lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "set_user: mutex timeout");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "User override: %s → %s",
             SENS_MODE_INFO[(uint8_t)mgr->current_mode].name,
             mode == SENS_MODE_AUTO ? "AUTO" : SENS_MODE_INFO[(uint8_t)mode].name);

    mgr->user_override = (mode != SENS_MODE_AUTO);
    esp_err_t ret = apply_mode(mgr, mode);
    SensitivityMode_t eff = mgr->effective_mode;

    if (ret == ESP_OK && mgr->user_override) {
        s_auto.last_applied   = mgr->effective_mode;
        s_auto.last_change_ms = (uint32_t)(esp_timer_get_time() / 1000LL);
    }

    xSemaphoreGive(mgr->lock);  /* FIX: release before event send */

    if (ret == ESP_OK) {
        bool sent = qm_event_send(SYS_EVT_SENS_CHANGE, (uint32_t)eff);
        if (!sent) {
            ESP_LOGW(TAG, "Event queue full — user sensitivity event delayed");
        }
    }
    return ret;
}

esp_err_t sens_manager_set_auto(SensitivityManager_t *mgr)
{
    if (xSemaphoreTake(mgr->lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "set_auto: mutex timeout");
        return ESP_ERR_TIMEOUT;
    }

    mgr->user_override = false;
    ESP_LOGI(TAG, "Returning to AUTO (recommended: %s)",
             SENS_MODE_INFO[(uint8_t)mgr->recommended_mode].name);
    s_auto.last_applied = mgr->effective_mode;

    esp_err_t ret = apply_mode(mgr, SENS_MODE_AUTO);
    SensitivityMode_t eff = mgr->effective_mode;

    xSemaphoreGive(mgr->lock);  /* FIX: release before event send */

    if (ret == ESP_OK) {
        qm_event_send(SYS_EVT_SENS_CHANGE, (uint32_t)eff);
    }
    return ret;
}

/* =========================================================================
 * AUTO ENGINE
 * ========================================================================= */

#define AUTO_NOISE_THRESH_LOW       12.0f
#define AUTO_NOISE_THRESH_VERY_LOW  30.0f
#define AUTO_NOISE_FLOOR_MIN         0.02f

#define AUTO_SNR_UP_HIGH             3.0f
#define AUTO_SNR_UP_VERY_HIGH        6.0f
#define AUTO_SNR_DOWN_FROM_HIGH      2.0f
#define AUTO_SNR_DOWN_FROM_VHIGH     4.5f

#define AUTO_STAB_THRESH_MED        70.0f
#define AUTO_STAB_THRESH_HIGH       85.0f

#define AUTO_HYSTERESIS_MS         1200u
#define AUTO_NOISE_EMA_ALPHA         0.05f



static SensitivityMode_t auto_decide_mode(float noise_ema, float snr,
                                          float stability,
                                          SensitivityMode_t current)
{
    /* ── 1. Hard Protection: Force down if noise is dangerous ── */
    if (noise_ema >= AUTO_NOISE_THRESH_VERY_LOW) return SENS_MODE_VERY_LOW;
    if (noise_ema >= AUTO_NOISE_THRESH_LOW)      return SENS_MODE_LOW;

    /* ── 2. Safe Soil (noise < 12 LSB) — FSM with True Hysteresis ── */
    
    if (current == SENS_MODE_VERY_HIGH) {
        /* EXIT condition: Did we drop below the DOWN threshold? */
        if (snr < AUTO_SNR_DOWN_FROM_VHIGH || stability < AUTO_STAB_THRESH_HIGH) {
            /* Step down to HIGH if conditions still permit it */
            if (snr >= AUTO_SNR_DOWN_FROM_HIGH && stability >= AUTO_STAB_THRESH_MED) {
                return SENS_MODE_HIGH;
            }
            /* Panic fallback: Drop straight to MEDIUM */
            return SENS_MODE_MEDIUM;
        }
        return SENS_MODE_VERY_HIGH; /* Stay */
    }

    if (current == SENS_MODE_HIGH) {
        /* EXIT condition: Did we drop below the DOWN threshold? */
        if (snr < AUTO_SNR_DOWN_FROM_HIGH || stability < AUTO_STAB_THRESH_MED) {
            return SENS_MODE_MEDIUM;
        }
        /* UP condition: Can we jump to VERY_HIGH? */
        if (snr >= AUTO_SNR_UP_VERY_HIGH && stability >= AUTO_STAB_THRESH_HIGH) {
            return SENS_MODE_VERY_HIGH;
        }
        return SENS_MODE_HIGH; /* Stay */
    }

    /* ── 3. Currently in MEDIUM or LOW — Can we step UP? ── */
    /* BUGFIX: If we are in LOW, we can ONLY step up to MEDIUM. Never jump to HIGH directly. */
    if (current == SENS_MODE_LOW) {
        return SENS_MODE_MEDIUM; /* Safe 1-step transition */
    }

    /* We are in MEDIUM. Check UP conditions. */
    if (snr >= AUTO_SNR_UP_VERY_HIGH && stability >= AUTO_STAB_THRESH_HIGH) {
        return SENS_MODE_VERY_HIGH;
    }
    if (snr >= AUTO_SNR_UP_HIGH && stability >= AUTO_STAB_THRESH_MED) {
        return SENS_MODE_HIGH;
    }

    return SENS_MODE_MEDIUM; /* Fallback */
}

void sens_manager_auto_update(SensitivityManager_t *mgr,
                              float noise_floor,
                              float snr,
                              float stability,
                              bool  stable,
                              uint32_t now_ms)
{
    /* Early exit without locking if obviously not applicable */
    if (!mgr->initialized) return;

    /* BUGFIX: Lock mutex. Auto-update runs in a task, protect against UI task changes. */
    if (xSemaphoreTake(mgr->lock, 0) != pdTRUE) {
        return; /* Couldn't get lock, skip this cycle (don't block the signal task) */
    }

    if (mgr->user_override || mgr->current_mode != SENS_MODE_AUTO) {
        xSemaphoreGive(mgr->lock);
        return;
    }

    /* ── Init state on first call ── */
    if (!s_auto.initialised) {
        s_auto.noise_ema     = noise_floor;
        s_auto.last_applied  = mgr->effective_mode;
        s_auto.last_change_ms = now_ms;
        s_auto.initialised   = true;
        xSemaphoreGive(mgr->lock);
        return;   
    }

    /* ── EMA Filtering ── */
    s_auto.noise_ema = (1.0f - AUTO_NOISE_EMA_ALPHA) * s_auto.noise_ema
                     + AUTO_NOISE_EMA_ALPHA * noise_floor;
    if (s_auto.noise_ema < AUTO_NOISE_FLOOR_MIN) {
        s_auto.noise_ema = AUTO_NOISE_FLOOR_MIN;
    }

    /* ── Gates ── */
    if (!stable) {
        xSemaphoreGive(mgr->lock);
        return;
    }
    if ((now_ms - s_auto.last_change_ms) < AUTO_HYSTERESIS_MS) {
        xSemaphoreGive(mgr->lock);
        return;
    }

    /* ── Decision ── */
    SensitivityMode_t target = auto_decide_mode(s_auto.noise_ema, snr,
                                                stability,
                                                s_auto.last_applied);

    if (target == s_auto.last_applied) {
        xSemaphoreGive(mgr->lock);
        return;
    }

    ESP_LOGI(TAG, "AUTO engine: %s → %s (noise_ema=%.1f snr=%.1f)",
             SENS_MODE_INFO[(uint8_t)s_auto.last_applied].name,
             SENS_MODE_INFO[(uint8_t)target].name,
             (double)s_auto.noise_ema,
             (double)snr);

    esp_err_t ret = apply_mode(mgr, target);
    if (ret == ESP_OK) {
        s_auto.last_applied   = target;
        s_auto.last_change_ms = now_ms;
    } else {
        ESP_LOGW(TAG, "AUTO engine: apply_mode failed: %s", esp_err_to_name(ret));
    }

    xSemaphoreGive(mgr->lock);  /* FIX (Phase 1 #4a): release before event send ?
                                 * same lock-order-inversion avoidance pattern used
                                 * by apply_recommendation/set_user/set_auto below. */

    /* BUGFIX (Phase 1 #4a): unlike apply_recommendation/set_user/set_auto,
     * this internal AUTO-engine transition never told signal_task about the
     * new effective mode. PGA hardware was switched by apply_mode() above,
     * but sp_set_sensitivity() was never called ? DSP filter params AND the
     * ADC-count baseline scale (Phase 1 #4b) stayed pinned to the previous
     * mode for the rest of the AUTO session. Send the same event the other
     * three callers send, using the identical pattern. */
    if (ret == ESP_OK) {
        bool sent = qm_event_send(SYS_EVT_SENS_CHANGE, (uint32_t)target);
        if (!sent) {
            ESP_LOGW(TAG, "AUTO engine: event queue full — sensitivity event delayed");
        }
    }
}

/* =========================================================================
 * QUERIES (Added Mutex protection for safe reads from UI)
 * ========================================================================= */

SensitivityMode_t sens_manager_get_current(const SensitivityManager_t *mgr)
{
    /* Cast away const for lock, safe RTOS pattern for getters */
    xSemaphoreTake(((SensitivityManager_t*)mgr)->lock, portMAX_DELAY);
    SensitivityMode_t mode = mgr->current_mode;
    xSemaphoreGive(((SensitivityManager_t*)mgr)->lock);
    return mode;
}

SensitivityMode_t sens_manager_get_effective(const SensitivityManager_t *mgr)
{
    xSemaphoreTake(((SensitivityManager_t*)mgr)->lock, portMAX_DELAY);
    SensitivityMode_t mode = mgr->effective_mode;
    xSemaphoreGive(((SensitivityManager_t*)mgr)->lock);
    return mode;
}

bool sens_manager_is_user_override(const SensitivityManager_t *mgr)
{
    xSemaphoreTake(((SensitivityManager_t*)mgr)->lock, portMAX_DELAY);
    bool override = mgr->user_override;
    xSemaphoreGive(((SensitivityManager_t*)mgr)->lock);
    return override;
}

/* (Other getters left un-locked if they are only called during init, 
   but locked them is safer if called from UI tasks) */
