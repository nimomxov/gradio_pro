/**
 * @file sensitivity_manager.c
 * @brief Sensitivity mode management — implementation.
 */

#include "sensitivity_manager.h"
#include "core/queue_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static const char *TAG = "SensMgr";

/* =========================================================================
 * MODE INFO TABLE
 *
 * Maps each sensitivity mode to its hardware (PGA) and description.
 * Index must match SensitivityMode_t enum values.
 * ========================================================================= */

/* مؤشر الجدول يجب أن يتطابق مع قيم SensitivityMode_t:
 * AUTO=0, VERY_HIGH=1, HIGH=2, MEDIUM=3, LOW=4, VERY_LOW=5
 * PGA: VERY_HIGH/HIGH → ±1.024V | MEDIUM → ±2.048V | LOW/VERY_LOW → ±4.096V
 */
const SensModeInfo_t SENS_MODE_INFO[6] = {
    /* [0] AUTO */
    {
        .mode        = SENS_MODE_AUTO,
        .name        = "AUTO",
        .description = "Adaptive — engine selects based on soil noise + SNR",
        .pga         = ADS1115_PGA_2048MV,
        .lsb_uv      = 62.5f,
    },
    /* [1] VERY_HIGH */
    {
        .mode        = SENS_MODE_VERY_HIGH,
        .name        = "VERY HIGH",
        .description = "Pristine soil only — std < 1.0 — deepest targets",
        .pga         = ADS1115_PGA_1024MV,   /* ±1.024V, 31.25µV/LSB */
        .lsb_uv      = 31.25f,
    },
    /* [2] HIGH */
    {
        .mode        = SENS_MODE_HIGH,
        .name        = "HIGH",
        .description = "Clean soil — deep targets and voids",
        .pga         = ADS1115_PGA_1024MV,   /* ±1.024V, 31.25µV/LSB */
        .lsb_uv      = 31.25f,
    },
    /* [3] MEDIUM */
    {
        .mode        = SENS_MODE_MEDIUM,
        .name        = "MEDIUM",
        .description = "General purpose — balanced noise and depth",
        .pga         = ADS1115_PGA_2048MV,   /* ±2.048V, 62.5µV/LSB */
        .lsb_uv      = 62.5f,
    },
    /* [4] LOW */
    {
        .mode        = SENS_MODE_LOW,
        .name        = "LOW",
        .description = "Mineralized soil — std 12–30",
        .pga         = ADS1115_PGA_4096MV,   /* ±4.096V, 125µV/LSB */
        .lsb_uv      = 125.0f,
    },
    /* [5] VERY_LOW */
    {
        .mode        = SENS_MODE_VERY_LOW,
        .name        = "VERY LOW",
        .description = "Extreme interference — volcanic/industrial — std > 30",
        .pga         = ADS1115_PGA_4096MV,   /* ±4.096V, 125µV/LSB */
        .lsb_uv      = 125.0f,
    },
};

/* =========================================================================
 * PRIVATE HELPERS
 * ========================================================================= */

static esp_err_t apply_mode(SensitivityManager_t *mgr, SensitivityMode_t mode)
{
    if (!mgr->initialized) return ESP_ERR_INVALID_STATE;

    /* Resolve AUTO → current recommended */
    SensitivityMode_t effective = mode;
    if (effective == SENS_MODE_AUTO) {
        effective = mgr->recommended_mode;
        if (effective == SENS_MODE_AUTO) {
            effective = SENS_MODE_MEDIUM;  /* Fallback if no calibration yet */
        }
    }

    /* Clamp to valid index (0..5 = AUTO..VERY_LOW) */
    if ((uint8_t)effective >= 6) effective = SENS_MODE_MEDIUM;

    const SensModeInfo_t *info = &SENS_MODE_INFO[(uint8_t)effective];

    ESP_LOGI(TAG, "Applying mode: %s — PGA ±%.3fV (%.2f µV/LSB)",
             info->name,
             info->pga == ADS1115_PGA_1024MV ? 1.024f :
             info->pga == ADS1115_PGA_2048MV ? 2.048f : 4.096f,
             info->lsb_uv);

    /* Step 1: Hardware — change PGA on ADS1115 */
    if (mgr->adc_driver) {
        esp_err_t ret = ads1115_set_pga(mgr->adc_driver, info->pga);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set PGA: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    /* Step 2: Software — notify signal_task via event queue */
    bool sent = qm_event_send(SYS_EVT_SENS_CHANGE, (uint32_t)effective, false);
    if (!sent) {
        ESP_LOGW(TAG, "Event queue full — software params may not update immediately");
    }

    mgr->current_mode   = mode;
    mgr->effective_mode = effective;   /* الوضع الفعلي المُطبَّق (AUTO مُحلَّل) */
    return ESP_OK;
}

/* =========================================================================
 * LIFECYCLE
 * ========================================================================= */

void sens_manager_init(SensitivityManager_t *mgr, ADS1115Driver_t *adc_driver)
{
    memset(mgr, 0, sizeof(SensitivityManager_t));
    mgr->current_mode     = SENS_MODE_MEDIUM;
    mgr->recommended_mode = SENS_MODE_MEDIUM; /* افتراضي حتى المعايرة */
    mgr->effective_mode   = SENS_MODE_MEDIUM;
    mgr->user_override    = false;
    mgr->adc_driver       = adc_driver;
    mgr->initialized      = true;

    ESP_LOGI(TAG, "Initialized. Default: MEDIUM");
}

/* =========================================================================
 * MODE CONTROL
 * ========================================================================= */

esp_err_t sens_manager_apply_recommendation(SensitivityManager_t *mgr,
                                            SensitivityMode_t mode)
{
    mgr->recommended_mode = mode;

    /* Don't override if user has manually selected a mode */
    if (mgr->user_override) {
        ESP_LOGI(TAG, "Calibration recommends %s — keeping user override: %s",
                 SENS_MODE_INFO[(uint8_t)mode].name,
                 SENS_MODE_INFO[(uint8_t)mgr->current_mode].name);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Applying calibration recommendation: %s",
             SENS_MODE_INFO[(uint8_t)mode].name);

    return apply_mode(mgr, mode);
}

esp_err_t sens_manager_set_user(SensitivityManager_t *mgr, SensitivityMode_t mode)
{
    ESP_LOGI(TAG, "User override: %s → %s",
             SENS_MODE_INFO[(uint8_t)mgr->current_mode].name,
             mode == SENS_MODE_AUTO ? "AUTO" : SENS_MODE_INFO[(uint8_t)mode].name);

    mgr->user_override = (mode != SENS_MODE_AUTO);

    return apply_mode(mgr, mode);
}

esp_err_t sens_manager_set_auto(SensitivityManager_t *mgr)
{
    mgr->user_override = false;
    ESP_LOGI(TAG, "Returning to AUTO (recommended: %s)",
             SENS_MODE_INFO[(uint8_t)mgr->recommended_mode].name);
    return apply_mode(mgr, SENS_MODE_AUTO);
}


/* =========================================================================
 * AUTO ENGINE — sens_manager_auto_update()
 * =========================================================================
 *
 * يُستدعى من signal_task بعد كل sp_process() — يأخذ مقاييس الإشارة
 * ويُقرر تلقائياً أفضل وضع حساسية يناسب التربة الحالية.
 *
 * لا يلمس ADC samples ولا DSP pipeline — decision layer فقط.
 *
 * ─────────────────────────────────────────────────────────────────
 * خوارزمية القرار (FSM مع hysteresis):
 *
 *  مدخلات:
 *    noise_floor  — انحراف معياري من معايرة التربة [LSB]
 *    snr          — signal_variance / noise_variance من sp_get_snr()
 *    stability    — stability_score [0..100] من sp_get_stability()
 *    now_ms       — الوقت الحالي [ms]
 *
 *  منطق الاختيار (مُرتَّب من الأهم):
 *
 *    1. noise_floor > NOISE_THRESH_VERY_LOW  → VERY_LOW  (تربة متطرفة)
 *    2. noise_floor > NOISE_THRESH_LOW       → LOW       (تربة معدنية)
 *    3. noise_floor > NOISE_THRESH_MEDIUM    → MEDIUM    (تربة عادية)
 *    4. snr > SNR_THRESH_VERY_HIGH
 *       && stability ≥ STAB_THRESH_HIGH      → VERY_HIGH (تربة نقية جداً)
 *    5. snr > SNR_THRESH_HIGH
 *       && stability ≥ STAB_THRESH_MED       → HIGH      (تربة نظيفة)
 *    6. fallback                             → MEDIUM
 *
 *  حماية الاستقرار:
 *    - Stability gate: لا تغيير إذا stability < STAB_THRESH_CHANGE
 *    - Time lock: لا تغيير قبل مرور AUTO_HYSTERESIS_MS من آخر تغيير
 *    - EMA على noise_floor: يمنع استجابة التغييرات الآنية (noise spikes)
 *    - لا تغيير إذا mode == manual (user_override = true)
 *
 *  ثوابت مُعايَرة للـ FLC100 + ADS1115 + تربة شمال أفريقيا:
 *    NOISE_THRESH_MEDIUM   = 4.0  LSB  (فاصل MEDIUM/HIGH)
 *    NOISE_THRESH_LOW      = 12.0 LSB  (فاصل LOW/MEDIUM)
 *    NOISE_THRESH_VERY_LOW = 30.0 LSB  (فاصل VERY_LOW/LOW)
 *    SNR_THRESH_HIGH       = 3.0       (فاصل HIGH/MEDIUM)
 *    SNR_THRESH_VERY_HIGH  = 6.0       (فاصل VERY_HIGH/HIGH)
 * ========================================================================= */

/* ── عتبات قرار AUTO ENGINE ── */

/* noise_floor [LSB] — حماية: تحدد الحد الأقصى للحساسية الآمنة */
#define AUTO_NOISE_THRESH_LOW       12.0f  /* noise ≥ 12 → LOW                 */
#define AUTO_NOISE_THRESH_VERY_LOW  30.0f  /* noise ≥ 30 → VERY_LOW            */
#define AUTO_NOISE_FLOOR_MIN         0.02f /* clamp أدنى → يمنع SNR انفجاري   */

/* SNR — عتبات الصعود (UP) مختلفة عن النزول (DOWN) = hysteresis حقيقي */
#define AUTO_SNR_UP_HIGH             3.0f  /* صعود إلى HIGH                    */
#define AUTO_SNR_UP_VERY_HIGH        6.0f  /* صعود إلى VERY_HIGH               */
#define AUTO_SNR_DOWN_FROM_HIGH      2.0f  /* نزول من HIGH → MEDIUM            */
#define AUTO_SNR_DOWN_FROM_VHIGH     4.5f  /* نزول من VERY_HIGH → HIGH         */

/* stability [0..100] — عتبات للترقية فقط */
#define AUTO_STAB_THRESH_MED        70.0f  /* أدنى stability للـ HIGH          */
#define AUTO_STAB_THRESH_HIGH       85.0f  /* أدنى stability للـ VERY_HIGH     */

/* زمن ونعومة */
#define AUTO_HYSTERESIS_MS         1200u   /* ms lockout بين تغييرَين          */
#define AUTO_NOISE_EMA_ALPHA         0.05f /* EMA على noise_floor (TC≈4s@50Hz) */

/* ── حالة Engine (داخلية) ── */
typedef struct {
    uint32_t          last_change_ms;    /* وقت آخر تغيير للمود              */
    SensitivityMode_t last_applied;      /* آخر مود طُبِّق                    */
    float             noise_ema;         /* EMA على noise_floor               */
    bool              initialised;
} AutoEngineState_t;

static AutoEngineState_t s_auto = {0};

/**
 * @brief يحدد الوضع الأمثل بناءً على المقاييس الحالية.
 *
 * الترتيب الصحيح:
 *   1. حماية أولاً: noise شديد → نزول إلزامي بغض النظر عن SNR
 *   2. Hysteresis: إذا كنا في HIGH أو VERY_HIGH، نستخدم عتبات نزول
 *      مختلفة (أدنى) عن عتبات الصعود → يمنع oscillation
 *   3. MEDIUM/LOW: هل نرتقي؟ نستخدم عتبات الصعود (أعلى)
 *
 * @param noise_ema    EMA مُنعَّم لـ noise_floor [LSB]
 * @param snr          نسبة إشارة/ضوضاء من sp_get_snr()
 * @param stability    stability_score [0..100] من sp_get_stability()
 * @param current      المود الحالي للـ hysteresis
 */
static SensitivityMode_t auto_decide_mode(float noise_ema, float snr,
                                          float stability,
                                          SensitivityMode_t current)
{
    /* ── 1. حماية: noise شديد — نزول قسري (أولوية مطلقة) ── */
    if (noise_ema >= AUTO_NOISE_THRESH_VERY_LOW) return SENS_MODE_VERY_LOW;
    if (noise_ema >= AUTO_NOISE_THRESH_LOW)      return SENS_MODE_LOW;

    /* ── 2. تربة هادئة (noise < 12 LSB) — Hysteresis FSM ──
     *
     * الفرق بين عتبات الصعود والنزول يمنع oscillation:
     *   HIGH:      صعود عند SNR≥3.0  |  نزول عند SNR<2.0
     *   VERY_HIGH: صعود عند SNR≥6.0  |  نزول عند SNR<4.5
     */
    if (current == SENS_MODE_VERY_HIGH) {
        if (snr >= AUTO_SNR_DOWN_FROM_VHIGH && stability >= AUTO_STAB_THRESH_HIGH) {
            return SENS_MODE_VERY_HIGH;   /* ثابت — لم نخرج من نطاق VERY_HIGH */
        }
        /* خرجنا → انزل إلى HIGH إذا مؤهل أو MEDIUM */
        if (snr >= AUTO_SNR_UP_HIGH && stability >= AUTO_STAB_THRESH_MED) {
            return SENS_MODE_HIGH;
        }
        return SENS_MODE_MEDIUM;
    }

    if (current == SENS_MODE_HIGH) {
        if (snr < AUTO_SNR_DOWN_FROM_HIGH || stability < AUTO_STAB_THRESH_MED) {
            return SENS_MODE_MEDIUM;      /* نزول من HIGH */
        }
        /* صعود من HIGH → VERY_HIGH إذا ارتفع SNR بما يكفي */
        if (snr >= AUTO_SNR_UP_VERY_HIGH && stability >= AUTO_STAB_THRESH_HIGH) {
            return SENS_MODE_VERY_HIGH;
        }
        return SENS_MODE_HIGH;            /* ثابت */
    }

    /* ── 3. MEDIUM أو LOW: هل نرتقي؟ عتبات الصعود (أعلى من النزول) ── */
    if (snr >= AUTO_SNR_UP_VERY_HIGH && stability >= AUTO_STAB_THRESH_HIGH) {
        return SENS_MODE_VERY_HIGH;
    }
    if (snr >= AUTO_SNR_UP_HIGH && stability >= AUTO_STAB_THRESH_MED) {
        return SENS_MODE_HIGH;
    }
    return SENS_MODE_MEDIUM;   /* fallback آمن */
}

void sens_manager_auto_update(SensitivityManager_t *mgr,
                              float noise_floor,
                              float snr,
                              float stability,
                              bool  stable,
                              uint32_t now_ms)
{
    /* ── guard: فقط في AUTO mode وبدون تدخل يدوي ── */
    if (!mgr->initialized)    return;
    if (mgr->user_override)   return;
    if (mgr->current_mode != SENS_MODE_AUTO) return;

    /* ── تهيئة state في أول استدعاء ── */
    if (!s_auto.initialised) {
        s_auto.noise_ema     = noise_floor;
        s_auto.last_applied  = mgr->effective_mode;
        s_auto.last_change_ms = now_ms;
        s_auto.initialised   = true;
        return;   /* دورة أولى فقط للتهيئة */
    }

    /* ── EMA على noise_floor ──
     * alpha=0.05 → TC ≈ 20 استدعاء ≈ 4 ثوانٍ @ 50Hz
     * clamp أدنى: يمنع noise_ema ≈ 0 → SNR انفجاري → VERY_HIGH وهمي
     */
    s_auto.noise_ema = (1.0f - AUTO_NOISE_EMA_ALPHA) * s_auto.noise_ema
                     + AUTO_NOISE_EMA_ALPHA * noise_floor;
    if (s_auto.noise_ema < AUTO_NOISE_FLOOR_MIN) {
        s_auto.noise_ema = AUTO_NOISE_FLOOR_MIN;
    }

    /* ── Stability gate: BEFORE القرار ──
     * EMA يستمر أثناء الحركة (يتتبع التربة الحقيقية)
     * لكن القرار مُعلَّق حتى يستقر الجهاز
     */
    if (!stable) return;

    /* ── Time lock ── */
    if ((now_ms - s_auto.last_change_ms) < AUTO_HYSTERESIS_MS) return;

    /* ── اتخاذ القرار مع Hysteresis بين الصعود والنزول ──
     * نُمرِّر last_applied و stability الفعلية لاستخدام عتبات مستقلة
     */
    SensitivityMode_t target = auto_decide_mode(s_auto.noise_ema, snr,
                                                stability,
                                                s_auto.last_applied);

    /* ── تطبيق التغيير فقط إذا اختلف عن المود الحالي ── */
    if (target == s_auto.last_applied) return;

    ESP_LOGI(TAG, "AUTO engine: %s → %s (noise_ema=%.1f snr=%.1f)",
             SENS_MODE_INFO[(uint8_t)s_auto.last_applied].name,
             SENS_MODE_INFO[(uint8_t)target].name,
             (double)s_auto.noise_ema,
             (double)snr);

    /* apply_mode يُحدِّث PGA + يُرسل SYS_EVT_SENS_CHANGE → signal_task */
    esp_err_t ret = apply_mode(mgr, target);
    if (ret == ESP_OK) {
        s_auto.last_applied   = target;
        s_auto.last_change_ms = now_ms;
    } else {
        ESP_LOGW(TAG, "AUTO engine: apply_mode failed: %s", esp_err_to_name(ret));
    }
}

/* =========================================================================
 * QUERIES
 * ========================================================================= */

SensitivityMode_t sens_manager_get_current(const SensitivityManager_t *mgr)
{
    return mgr->current_mode;
}

SensitivityMode_t sens_manager_get_recommended(const SensitivityManager_t *mgr)
{
    return mgr->recommended_mode;
}

bool sens_manager_is_user_override(const SensitivityManager_t *mgr)
{
    return mgr->user_override;
}

SensitivityMode_t sens_manager_get_effective(const SensitivityManager_t *mgr)
{
    return mgr->effective_mode;
}

const SensModeInfo_t *sens_manager_get_info(SensitivityMode_t mode)
{
    if ((uint8_t)mode >= 6) return &SENS_MODE_INFO[SENS_MODE_MEDIUM];
    return &SENS_MODE_INFO[(uint8_t)mode];
}

void sens_manager_get_status_str(const SensitivityManager_t *mgr,
                                 char *buf, size_t buf_len)
{
    const char *name = SENS_MODE_INFO[(uint8_t)mgr->current_mode].name;
    const char *src  = mgr->user_override ? "user" : "auto";
    snprintf(buf, buf_len, "%s (%s)", name, src);
}
