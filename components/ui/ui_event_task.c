/**
 * @file ui_event_task.c
 * @brief الجسر الكامل بين Backend و EEZ Studio Frontend.
 *
 * ═══════════════════════════════════════════════════════════════════
 *  FEATURES — LIVE SCAN TAB
 * ═══════════════════════════════════════════════════════════════════
 *  live_scan_chart        → خط بلا خلفية، يملأ live_chart container
 *  baseline_text          → قيمة baseline المُحسوبة
 *  values_text            → قيمة ADS1115 المُفلترة (ملوَّنة)
 *  calibration_button     → يُطلق المعايرة + يختفي
 *  calibration_progress_bar → يملأ تدريجياً (progress حقيقي)
 *  led_calibration_status → #477f53 نجاح / #cd4755 فشل
 *
 * ═══════════════════════════════════════════════════════════════════
 *  FEATURES — 3D SCAN TAB
 * ═══════════════════════════════════════════════════════════════════
 *  auto_scan_process:
 *    - countdown تنازلي من seconds_numbers
 *    - عند الصفر: scan + buzzer + تأثير بصري + BT إرسال
 *    - إنقاص خطوة من steps_number → تكرار
 *    - عند اكتمال الخطوات: dialog [Next Line] / [Finish]
 *  auto_scan_cancel_button / manual_scan_cancel_button → إلغاء فوري
 *
 *  bluetooth icons:
 *    CONNECTED → bluetooth_status_success فقط
 *    ERROR     → bluetooth_status_failed  فقط
 *    OFF/CONN  → bluetooth_status_idle    فقط
 *
 * ═══════════════════════════════════════════════════════════════════
 *  FEATURES — SETTINGS TAB
 * ═══════════════════════════════════════════════════════════════════
 *  manual_sensibility_switch → AUTO ↔ Manual
 *  sensibility_settings      → Very Low..Very High → SENS_MODE_*
 *  state_infos               → أحداث Backend الجارية (device events)
 *  logs_text                 → سجل الأخطاء والأحداث المهمة فقط
 */

#include "ui_event_task.h"
#include "screens.h"
#include "actions.h"
#include "boot_selfcheck.h"
#include "bar_chart.h"

#include "core/queue_manager.h"
#include "core/gradiometer_types.h"
//#include "modes/sensitivity_manager.h"
#include "drivers/bluetooth_sender.h"
#include "drivers/adc_task.h"
#include "core/signal_task.h"
#include "device_calibration/devcal_common.h"
#include "device_calibration/device_cal.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "lvgl.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* ═══════════════════════════════════════════════════════════════════
 * CONSTANTS
 * ═══════════════════════════════════════════════════════════════════ */

static const char *TAG = "UIEvt";

#define TASK_PERIOD_MS       20u    /* 50 Hz task rate               */
#define BT_CHECK_PERIOD_MS   1000u  /* BT icon refresh period        */
#define CHART_POINTS         80u    /* نقاط الـ live chart            */
#define LOG_BUF_BYTES        480u   /* حجم بفر الـ logs (≤ heap safe) */

/* ── Buzzer (from hardware_config.h via gradiometer_types.h) ── */
#ifndef HW_BUZZER_GPIO
#  define HW_BUZZER_GPIO       GPIO_NUM_26
#  define HW_BUZZER_LEDC_CHAN  LEDC_CHANNEL_0
#  define HW_BUZZER_LEDC_TIMER LEDC_TIMER_0
#  define HW_BUZZER_DUTY       4096u
#endif

/* ═══════════════════════════════════════════════════════════════════
 * PRIVATE STATE
 * ═══════════════════════════════════════════════════════════════════ */

/* ── globals shared with app_main ── */
static BTSender_t       *s_bt = NULL;
extern SemaphoreHandle_t g_lvgl_mutex;

/* ── calibration ── */
static bool s_calibrating = false;

/* ── auto scan state machine ── */
typedef enum {
    AS_IDLE      = 0,
    AS_COUNTDOWN = 1,
    AS_LINE_DONE = 2,
} AutoScanState_t;

static AutoScanState_t s_as_state     = AS_IDLE;
static uint8_t         s_steps_total  = 5;
static uint8_t         s_steps_left   = 5;
static uint8_t         s_secs_total   = 3;
static int32_t         s_cd_ms        = 0;   /* countdown remaining ms */

/* ── countdown UI objects (created dynamically) ── */
static lv_obj_t *s_lbl_cd_secs  = NULL;   /* رقم الثواني (كبير، وسط)     */
static lv_obj_t *s_lbl_cd_steps = NULL;   /* "Step N/M"                    */
static lv_obj_t *s_bar_cd       = NULL;   /* شريط تقدم خطي (سفلي)         */
static lv_obj_t *s_arc_cd       = NULL;   /* arc دائري احترافي             */
static lv_obj_t *s_dialog       = NULL;   /* Next/Finish dialog            */
static int32_t   s_cd_last_sec  = -1;     /* تتبع آخر ثانية للـ animation  */

/* ── manual sensitivity ── */
static bool s_manual_sens = false;

/* ── SNR Panel — ثوابت الألوان والحدود ──────────────────────────────
 *
 * نظام الألوان:
 *   SNR < 1.0  → أخضر   0x2ECC71  (تربة عادية / لا هدف)
 *   SNR 1-2    → أصفر   0xF1C40F  (إشارة ضعيفة)
 *   SNR 2-4    → برتقالي 0xE67E22 (هدف محتمل)
 *   SNR > 4    → أحمر   0xE74C3C  (هدف واضح)
 *   غير مستقر  → رمادي  0x7F8C8D  (أثناء الحركة)
 *
 * Confidence = تقدير نسبة الثقة بالهدف [0..100%]
 *   = clamp(snr / 5.0 × 100, 0, 100)
 * ─────────────────────────────────────────────────────────────────── */
#define SNR_COLOR_GREEN    0x2ECC71u
#define SNR_COLOR_YELLOW   0xF1C40Fu
#define SNR_COLOR_ORANGE   0xE67E22u
#define SNR_COLOR_RED      0xE74C3Cu
#define SNR_COLOR_GREY     0x7F8C8Du

#define SNR_STABLE_WINDOW_MS  800u   /* ms استقرار قبل اعتبار SNR صحيح */
static uint32_t s_snr_stable_since_ms = 0u;
static float    s_snr_last            = 0.0f;
static bool     s_snr_is_stable       = false;

/* ── Stability Gate — Adaptive Sampling ─────────────────────────────
 * يعمل في Auto Scan فقط.
 * يراقب تذبذب الإشارة قبل أخذ العينة.
 * k=1.5 × noise_floor من device calibration = threshold الاستقرار.
 * ─────────────────────────────────────────────────────────────────── */
#define STAB_K              1.5f      /* معامل threshold                */
#define STAB_MIN_MS         1500u     /* أدنى وقت انتظار               */
#define STAB_MAX_MS         5000u     /* أقصى وقت انتظار ثم إجبار     */
static uint32_t s_stab_start_ms  = 0u;
static bool     s_stab_waiting   = false;
static bool     s_stab_gate_on   = true;   /* تتحكم فيها stability_gate checkbox */

/* ── BT icon caching ── */
static BtConnectionState_t s_last_bt = (BtConnectionState_t)0xFF;

/* ── log ring buffer ── */
static char s_log[LOG_BUF_BYTES];
static bool s_log_dirty = false;

/* ── last live sample ── */
static ProcessedSample_t s_last_sample;
static bool              s_has_sample = false;

/* ═══════════════════════════════════════════════════════════════════
 * LVGL LOCK MACROS
 * ═══════════════════════════════════════════════════════════════════ */

#define UI_LOCK()    (xSemaphoreTake(g_lvgl_mutex, pdMS_TO_TICKS(15)) == pdTRUE)
#define UI_UNLOCK()   xSemaphoreGive(g_lvgl_mutex)

/* ═══════════════════════════════════════════════════════════════════
 * LOG HELPER — يُسجِّل الأخطاء والأحداث المهمة فقط
 * ═══════════════════════════════════════════════════════════════════ */

static void log_push(const char *msg)
{
    /* دوران البفر إذا امتلأ — احذف أقدم 128 حرف */
    size_t cur = strlen(s_log);
    size_t msg_len = strlen(msg);
    size_t needed  = msg_len + 2;   /* +'\n'+'\0' */

    if (cur + needed >= LOG_BUF_BYTES) {
        uint16_t drop = (LOG_BUF_BYTES / 4);
        memmove(s_log, s_log + drop, LOG_BUF_BYTES - drop);
        memset(s_log + LOG_BUF_BYTES - drop, 0, drop);
        cur = strlen(s_log);
    }

    strncat(s_log, msg,  LOG_BUF_BYTES - cur - 2);
    strcat(s_log, "\n");
    s_log_dirty = true;
}

/* ═══════════════════════════════════════════════════════════════════
 * BUZZER — Professional Audio System
 *
 * ثلاثة أوضاع:
 *
 *  1. buzzer_beep_nb(freq, ms)
 *     beep واحد non-blocking عبر esp_timer — للـ scan/calib
 *
 *  2. buzzer_live_tone(deviation)
 *     نغمة مستمرة تتغير مع الإشارة — للـ live scan
 *     - صامت إذا deviation < LIVE_DEAD_OFF (hysteresis)
 *     - نغمة عميقة (فراغ)  إذا deviation سالبة
 *     - نغمة عالية (معدن) إذا deviation موجبة وترتفع مع القيمة
 *     - تُحدَّث كل LIVE_UPDATE_MS لسلاسة الصوت
 *
 *  3. buzzer_live_stop()
 *     يوقف النغمة المستمرة عند مغادرة Live Scan tab
 *
 * ═══════════════════════════════════════════════════════════════════
 *  تصميم الأصوات (مستوحى من أجهزة Minelab / Garrett):
 *
 *  METAL  (deviation > 0):
 *    baseline 900Hz → يرتفع خطياً حتى 3200Hz عند deviation=200
 *    duty 35% — صوت حاد واضح
 *
 *  VOID   (deviation < 0، قيمة مطلقة > DEAD_ZONE):
 *    ثابت 180Hz — نبضة عميقة منخفضة
 *    duty 25% — صوت خافت غير مزعج
 *
 *  QUIET  (|deviation| < LIVE_DEAD_OFF):
 *    صامت تماماً
 * ═══════════════════════════════════════════════════════════════════ */

#include "esp_timer.h"

/* ═══════════════════════════════════════════════════════════════════
 * PROFESSIONAL AUDIO ENGINE — Live Scan
 * ═══════════════════════════════════════════════════════════════════
 *
 * تصميم مستوحى من أجهزة Minelab / Garrett / OKM مع تكيف للـ ESP32 LEDC.
 *
 * ثلاثة أوضاع صوتية واضحة ومميزة:
 *
 *  1. METAL  (deviation > 0)
 *     نغمة متقطعة (pulse) تتسارع مع الاقتراب من الهدف:
 *       - التردد   : 800Hz ثابت (نبرة حادة مميزة للمعدن)
 *       - معدل التقطيع: يُخفَّف مع SNR (بعيد=بطيء، قريب=سريع)
 *       - duty      : يرتفع مع confidence (إشارة ضعيفة → صوت خافت)
 *     الهدف: "beep-beep-beep" يشبه أجهزة metal detector تجارية
 *
 *  2. VOID   (deviation < 0)
 *     نغمة مستمرة منخفضة ومميزة:
 *       - التردد   : 200Hz (عميق ومختلف تماماً عن المعدن)
 *       - duty      : ثابت (لا حاجة للتمييز الشدة — النوع واضح)
 *     الهدف: صوت "هوة" لا لبس فيه
 *
 *  3. SILENCE (|deviation| < LIVE_DEAD_OFF/ON (hysteresis) أو غير مستقر)
 *     صمت تام — لا هدر في الصوت = تربة نظيفة
 *
 * ─────────────────────────────────────────────────────────────────
 * LEDC optimization:
 *   ledc_timer_config يُستدعى مرة واحدة فقط عند التهيئة.
 *   التحديثات تستخدم ledc_set_duty فقط (لا timer reconfig).
 *   هذا يُقلل الحمل من ~50μs إلى ~5μs لكل تحديث.
 * ═══════════════════════════════════════════════════════════════════ */

/* ── ثوابت النظام الصوتي ── */

/* Dead zone — أدنى deviation لتفعيل الصوت */

/* Metal pulse tone */
#define LIVE_METAL_FREQ         800u    /* Hz ثابت — نبرة المعدن              */
#define LIVE_METAL_DUTY_MIN     800u    /* duty خافت (إشارة ضعيفة / conf=0%)  */
#define LIVE_METAL_DUTY_MAX     5120u   /* duty أقصى (conf=100%) ~62% of 8192 */

/* معدل التقطيع (pulse rate) للمعدن:
 *   interval = BASE - (SNR × SNR_STEP) - (vel × VEL_STEP)
 *   مُقيَّد بـ [MIN..BASE]
 *
 *   SNR=0, vel=0 → 500ms  (بعيد / لا تغيير)
 *   SNR=3, vel=0 → 335ms  (متوسط، ثابت)
 *   SNR=3, vel=1 → 285ms  (يقترب ← يتسارع فجأة)
 *   SNR=5, vel=0 → 225ms  (قريب، ثابت)
 *   SNR≥8        → 80ms   (أقصى سرعة — clamp)
 *
 * LIVE_PULSE_MIN_MS = 80ms (بدل 60) لمنع machine gun مزعج
 */
#define LIVE_PULSE_BASE_MS      500u    /* أبطأ معدل (ms بين نبضتين)          */
#define LIVE_PULSE_MIN_MS       80u     /* أسرع مقبول — أقل يصبح مزعج        */
#define LIVE_PULSE_MAX_MS       500u    /* أبطأ مقبول — مرادف للـ BASE (safety)*/
#define LIVE_PULSE_SNR_STEP     45.0f   /* ms تُخفَّف لكل وحدة SNR            */
#define LIVE_PULSE_VEL_STEP     50.0f   /* ms إضافية لكل وحدة velocity        */
#define LIVE_PULSE_ON_MS        35u     /* مدة كل نبضة (ms) — beep واضح       */

/* Void tone:
 *   duty يتكيف مع confidence → يعكس قوة الفراغ
 *   VOID_DUTY_MIN = ~7%  (فراغ ضعيف أو بعيد)
 *   VOID_DUTY_MAX = ~49% (فراغ كبير واضح)
 */
#define LIVE_VOID_FREQ          200u    /* Hz منخفض مميز للفراغ               */
#define LIVE_VOID_DUTY_MIN      600u    /* duty خافت عند conf=0%              */
#define LIVE_VOID_DUTY_MAX      4000u   /* duty أقصى عند conf=100%            */

/* Silence hysteresis — يمنع تقطيع الصوت في حدود الـ dead zone:
 *   |dev| < DEAD_OFF → صمت (enter silence)
 *   |dev| > DEAD_ON  → صوت (exit silence)
 *   الهامش بين ON/OFF = 4 LSB
 */
#define LIVE_DEAD_OFF           8.0f    /* عتبة الإسكات (أدنى)               */
#define LIVE_DEAD_ON            12.0f   /* عتبة التفعيل (أعلى)               */

/* معدل تحديث الصوت */
#define LIVE_UPDATE_MS          20u     /* كل 20ms (50Hz) — سلاسة كافية       */

/* ── حالة النظام الصوتي ── */
static esp_timer_handle_t s_buzzer_timer    = NULL; /* timer لإيقاف الـ beep  */
static bool               s_live_active     = false;
static uint32_t           s_last_live_upd   = 0u;
static bool               s_ledc_ready      = false; /* تم تهيئة LEDC مرة واحدة */

/* حالة المعدن التقطيع (pulse state machine) */
static uint32_t s_pulse_last_ms   = 0u;   /* وقت آخر بداية نبضة               */
static bool     s_pulse_on        = false; /* هل النبضة مفعّلة حالياً؟         */

/* حالة الـ velocity و silence hysteresis */
static float    s_audio_prev_snr  = 0.0f; /* SNR السابق لحساب velocity         */
static bool     s_audio_muted     = true; /* حالة الصمت (hysteresis gate)      */
static float    s_void_duty_smooth = 0.0f; /* EMA لتليين الـ duty للفراغ       */


/* widget handle — valid after chart_init_series 
 * ── Signal Notification Widget ───────────────────────────────────────
 * Yellow triangle with "!" and short Arabic message.
 * Appears upper-left corner of the live screen.
 * Auto-hides after NOTIF_AUTO_HIDE_MS if no new flags.
 * ─────────────────────────────────────────────────────────────────── */
static lv_obj_t *s_notif_container = NULL;  /* parent overlay — hidden by default */
static lv_obj_t *s_notif_label     = NULL;  /* message text */
static uint8_t   s_notif_flags_last = 0;    /* last flags shown — prevents flicker */
static uint32_t  s_notif_show_ms    = 0;    /* timestamp when notifier appeared */
#define NOTIF_AUTO_HIDE_MS   3500u           /* hide after 3.5s if flags gone */

/* ─────────────────────────────────────────────────────────────────
 * LEDC — تهيئة مرة واحدة + helpers منفصلة للتردد والـ duty
 * ─────────────────────────────────────────────────────────────────*/

/**
 * تهيئة LEDC مرة واحدة عند أول استخدام.
 * يحفظ ~45μs لكل تحديث بعدها.
 */
static void ledc_init_once(void)
{
    if (s_ledc_ready) return;

    ledc_timer_config_t tc = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num       = HW_BUZZER_LEDC_TIMER,
        .freq_hz         = LIVE_METAL_FREQ,   /* تردد أولي — يُعدَّل لاحقاً */
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&tc);

    ledc_channel_config_t cc = {
        .gpio_num   = HW_BUZZER_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = HW_BUZZER_LEDC_CHAN,
        .timer_sel  = HW_BUZZER_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&cc);

    s_ledc_ready = true;
}

/** تغيير التردد فقط — بدون إعادة تهيئة LEDC الكاملة */
static void ledc_change_freq(uint32_t freq_hz)
{
    ledc_set_freq(LEDC_LOW_SPEED_MODE, HW_BUZZER_LEDC_TIMER, freq_hz);
}

/** تغيير duty فقط */
static void ledc_change_duty(uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, HW_BUZZER_LEDC_CHAN, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, HW_BUZZER_LEDC_CHAN);
}

/** إيقاف الصوت فوراً */
static void ledc_silence(void)
{
    ledc_change_duty(0);
}

/* Compatibility wrapper — يُستخدم من buzzer_beep_nb فقط 
static void ledc_set_freq_duty(uint32_t freq_hz, uint32_t duty)
{
    ledc_init_once();
    if (freq_hz == 0) { ledc_silence(); return; }
    ledc_change_freq(freq_hz);
    ledc_change_duty(duty);
}
*/

/* ─────────────────────────────────────────────────────────────────
 * ONE-SHOT BEEP — للمعايرة والـ scan steps (non-blocking)
 * ─────────────────────────────────────────────────────────────────*/

static void buzzer_off_cb(void *arg)
{
    (void)arg;
    ledc_silence();
    s_live_active = false;
}

static void buzzer_init_once(void)
{
    ledc_init_once();
    if (s_buzzer_timer) return;
    const esp_timer_create_args_t args = {
        .callback              = buzzer_off_cb,
        .arg                   = NULL,
        .name                  = "buz_off",
        .dispatch_method       = ESP_TIMER_TASK,
        .skip_unhandled_events = true,
    };
    esp_timer_create(&args, &s_buzzer_timer);
}

/** beep واحد non-blocking — يوقف أي نغمة live */
static void buzzer_beep_nb(uint32_t freq_hz, uint32_t ms)
{
    buzzer_init_once();
    s_live_active  = false;
    s_pulse_on     = false;

    ledc_change_freq(freq_hz);
    ledc_change_duty(HW_BUZZER_DUTY);

    esp_timer_stop(s_buzzer_timer);
    esp_timer_start_once(s_buzzer_timer, (uint64_t)ms * 1000ULL);
}

/* ═══════════════════════════════════════════════════════════════════
 * LIVE AUDIO ENGINE — النواة الرئيسية
 * ═══════════════════════════════════════════════════════════════════
 *
 * @param deviation   filtered_value - baseline  (موجب=معدن، سالب=فراغ)
 * @param snr         نسبة الإشارة/الضوضاء       [0..∞)
 * @param confidence  ثقة الكشف من smart_detection [0..100]
 *
 * يُستدعى من handle_live_result — يُقيِّد نفسه بـ LIVE_UPDATE_MS.
 * ═══════════════════════════════════════════════════════════════════ */
static void buzzer_live_tone(float deviation, float snr, float confidence)
{
    ledc_init_once();

    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if ((now - s_last_live_upd) < LIVE_UPDATE_MS) return;
    s_last_live_upd = now;

    float abs_dev = (deviation < 0.f) ? -deviation : deviation;

    /* ══════════════════════════════════════════════════════
     * SILENCE HYSTERESIS — يمنع تقطيع الصوت عند حدود الـ dead zone
     *   muted=true  → يبقى صامتاً حتى يتجاوز DEAD_ON
     *   muted=false → يُسكت فقط إذا انخفض عن DEAD_OFF
     * الهامش 4 LSB يمنع الـ flicker تماماً.
     * ══════════════════════════════════════════════════════ */
    if (s_audio_muted) {
        if (abs_dev < LIVE_DEAD_ON) {
            /* لم نتجاوز عتبة التفعيل بعد — صمت */
            if (s_live_active || s_pulse_on) {
                ledc_silence();
                s_live_active = false;
                s_pulse_on    = false;
            }
            s_audio_prev_snr = snr;
            return;
        }
        s_audio_muted = false;   /* تجاوزنا DEAD_ON → فعّل الصوت */
    } else {
        if (abs_dev < LIVE_DEAD_OFF) {
            /* انخفضنا تحت DEAD_OFF → عد للصمت */
            s_audio_muted = true;
            ledc_silence();
            s_live_active = false;
            s_pulse_on    = false;
            s_audio_prev_snr = snr;
            return;
        }
    }

    /* ══════════════════════════════════════════════════════
     * METAL — نبضات متقطعة تتسارع مع SNR + velocity
     * ══════════════════════════════════════════════════════ */
    if (deviation > 0.0f) {

        /* ── حساب velocity (معدل تغيّر SNR) ──
         * موجب فقط: نقترب من الهدف → يُقصِّر الـ interval (تسارع مفاجئ)
         * سالب: نبتعد → لا تأثير على السرعة
         *
         * clamp [0..1.5]: يمنع spike ضجيجي من توليد vel كبير
         * اختيار 1.5 (بدل 5.0): تسارع طبيعي دون glitch صوتي
         */
        float vel = snr - s_audio_prev_snr;
        if (vel < 0.0f)  vel = 0.0f;
        if (vel > 1.5f)  vel = 1.5f;   /* clamp محكم — يمنع jump مفاجئ */
        s_audio_prev_snr = snr;

        /* ── حساب معدل التقطيع — Quadratic SNR ──
         *
         * interval = BASE - (SNR² × SNR_STEP) - (vel × VEL_STEP)
         *
         * SNR² بدل SNR → استجابة غير خطية تشبه الإدراك الفيزيائي للقرب:
         *   SNR=1 → snr_sq=1   → تأثير خفيف
         *   SNR=3 → snr_sq=9   → تسارع ملحوظ
         *   SNR=5 → snr_sq=25  → تسارع قوي
         *   SNR=8 → يصل للـ MIN (clamp)
         *
         * VEL_STEP=50ms لكل وحدة velocity → تسارع فوري عند الاقتراب
         */
        float snr_c   = (snr < 0.0f) ? 0.0f : (snr > 8.0f) ? 8.0f : snr;
        float snr_sq  = snr_c * snr_c;   /* quadratic — أكثر طبيعية من خطي */
        int32_t interval_ms = (int32_t)LIVE_PULSE_BASE_MS
                            - (int32_t)(snr_sq * (LIVE_PULSE_SNR_STEP / 3.0f))
                            - (int32_t)(vel    *  LIVE_PULSE_VEL_STEP);
        if (interval_ms < (int32_t)LIVE_PULSE_MIN_MS) {
            interval_ms = (int32_t)LIVE_PULSE_MIN_MS;   /* 80ms — anti machine-gun */
        }
        if (interval_ms > (int32_t)LIVE_PULSE_MAX_MS) {
            interval_ms = (int32_t)LIVE_PULSE_MAX_MS;   /* upper clamp — لا beep أبطأ من 500ms */
        }

        /* ── حساب duty بناءً على confidence ──
         * conf=0%   → DUTY_MIN (خافت — إشارة ضعيفة)
         * conf=100% → DUTY_MAX (قوي — هدف واضح)
         */
        float conf_n = confidence / 100.0f;
        if (conf_n < 0.0f) conf_n = 0.0f;
        if (conf_n > 1.0f) conf_n = 1.0f;
        uint32_t duty = LIVE_METAL_DUTY_MIN
                      + (uint32_t)(conf_n * (float)(LIVE_METAL_DUTY_MAX
                                                    - LIVE_METAL_DUTY_MIN));

        /* ── Pulse state machine ──
         * ON  → صوت LIVE_PULSE_ON_MS
         * OFF → صمت (interval - ON_MS)
         */
        uint32_t elapsed = now - s_pulse_last_ms;

        if (!s_pulse_on) {
            if (elapsed >= (uint32_t)interval_ms) {
                s_pulse_last_ms = now;
                s_pulse_on      = true;
                ledc_change_freq(LIVE_METAL_FREQ);
                ledc_change_duty(duty);
                s_live_active = true;
            }
        } else {
            if (elapsed >= LIVE_PULSE_ON_MS) {
                s_pulse_on = false;
                ledc_silence();
            } else {
                ledc_change_duty(duty);  /* تحديث شدة النبضة */
            }
        }
        return;
    }

    /* ══════════════════════════════════════════════════════
     * VOID — نغمة مستمرة منخفضة، amplitude تتكيف مع confidence
     * ══════════════════════════════════════════════════════ */
    s_pulse_on       = false;
    s_audio_prev_snr = snr;

    /* ── duty يعكس قوة الفراغ مع soft attack (EMA) ──
     * بدل القفز المباشر لـ duty: نُليِّنه بـ EMA (alpha=0.15)
     *   smooth = 0.85 × prev + 0.15 × target
     * النتيجة: صوت ناعم لا يقفز — يشبه فتح صمام تدريجياً
     */
    float conf_n = confidence / 100.0f;
    if (conf_n < 0.0f) conf_n = 0.0f;
    if (conf_n > 1.0f) conf_n = 1.0f;
    float target_duty = (float)LIVE_VOID_DUTY_MIN
                      + conf_n * (float)(LIVE_VOID_DUTY_MAX - LIVE_VOID_DUTY_MIN);

    /* Snap-to-zero قبل EMA: يضمن لا ghost duty عند العودة من الصمت */
    if (s_audio_muted) {
        s_void_duty_smooth = 0.0f;
    } else {
        s_void_duty_smooth = 0.85f * s_void_duty_smooth + 0.15f * target_duty;
    }

    if (!s_live_active) {
        ledc_change_freq(LIVE_VOID_FREQ);
        s_live_active = true;
    }
    ledc_change_duty((uint32_t)s_void_duty_smooth);
}

/** يوقف كل صوت live فوراً (عند مغادرة Live Scan tab) */
static void buzzer_live_stop(void)
{
    if (s_live_active || s_pulse_on) {
        ledc_silence();
        s_live_active = false;
        s_pulse_on    = false;
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * SIGNAL NOTIFICATION WIDGET
 * ═══════════════════════════════════════════════════════════════════
 *
 * Visual design:
 *   ┌──────────────────────────────┐
 *   │  ▲ !  تداخل كهربائي          │  ← upper-left corner, live screen
 *   └──────────────────────────────┘
 *
 * The triangle is drawn as an LVGL canvas with yellow fill + black "!".
 * The message label is right of the triangle.
 * The container uses LV_OBJ_FLAG_IGNORE_LAYOUT so it floats over the chart.
 * Z-order: created last → always on top.
 *
 * Called from: chart_init_series (creates widget)
 *              notifier_update()  (updates per sample, inside UI_LOCK)
 * ═══════════════════════════════════════════════════════════════════ */

static void notifier_init(void)
{
    lv_obj_t *screen = lv_scr_act();
    if (!screen) return;

    /* Container: transparent bg, yellow border, rounded, upper-left */
    s_notif_container = lv_obj_create(screen);
    lv_obj_set_size(s_notif_container, 175, 28);
    lv_obj_align(s_notif_container, LV_ALIGN_TOP_LEFT, 4, 4);
    lv_obj_set_style_bg_color (s_notif_container, lv_color_hex(0x1A1D26u), LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa   (s_notif_container, LV_OPA_90,               LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(s_notif_container, lv_color_hex(0xF1C40Fu), LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(s_notif_container, 1,                    LV_STATE_DEFAULT);
    lv_obj_set_style_radius   (s_notif_container, 5,                       LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all  (s_notif_container, 0,                       LV_STATE_DEFAULT);
    lv_obj_clear_flag(s_notif_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag  (s_notif_container, LV_OBJ_FLAG_HIDDEN);  /* hidden initially */

    /* Triangle icon: bold yellow "▲!" label on the left */
    lv_obj_t *icon = lv_label_create(s_notif_container);
    lv_label_set_text(icon, " " LV_SYMBOL_WARNING);
    lv_obj_set_style_text_color(icon, lv_color_hex(0xF1C40Fu), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font (icon, &lv_font_montserrat_14,  LV_STATE_DEFAULT);
    lv_obj_align(icon, LV_ALIGN_LEFT_MID, 2, 0);

    /* Message label: white text, right of icon */
    s_notif_label = lv_label_create(s_notif_container);
    lv_label_set_text(s_notif_label, "");
    lv_obj_set_style_text_color(s_notif_label, lv_color_hex(0xECF0F1u), LV_STATE_DEFAULT);
    lv_obj_set_style_text_font (s_notif_label, &lv_font_montserrat_12,  LV_STATE_DEFAULT);
    lv_obj_align(s_notif_label, LV_ALIGN_LEFT_MID, 24, 0);
    lv_label_set_long_mode(s_notif_label, LV_LABEL_LONG_CLIP);
    lv_obj_set_width(s_notif_label, 145);
}

/**
 * @brief Update the notification widget based on signal_flags from sp_process.
 *
 * Priority order (highest first — only one message shown at a time):
 *   SPIKE      → "تداخل كهربائي / EMI"          (red border)
 *   HIGH_NOISE → "تربة مُعدِّنة — اخفض الحساسية" (orange border)
 *   DRIFT      → "الجهاز يتكيف مع التربة"        (yellow border, dimmer)
 *   UNSTABLE   → "حرِّك الجهاز ببطء أكثر"        (yellow border)
 *
 * Must be called inside UI_LOCK() from handle_live_result.
 */
static void notifier_update(uint8_t flags, uint8_t confidence)
{
    if (!s_notif_container || !s_notif_label) return;

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000LL);

    /* Auto-hide: if flags cleared and shown long enough */
    if (flags == SIGNAL_FLAG_NONE) {
        if (s_notif_flags_last != SIGNAL_FLAG_NONE &&
            (now_ms - s_notif_show_ms) >= NOTIF_AUTO_HIDE_MS) {
            lv_obj_add_flag(s_notif_container, LV_OBJ_FLAG_HIDDEN);
            s_notif_flags_last = SIGNAL_FLAG_NONE;
        }
        return;
    }

    /* No change and already showing — avoid unnecessary redraws */
    if (flags == s_notif_flags_last && !lv_obj_has_flag(s_notif_container, LV_OBJ_FLAG_HIDDEN)) {
        return;
    }

    /* Priority order: SPIKE > HIGH_NOISE > DRIFT > UNSTABLE */
    const char *msg;
    uint32_t    border_color;

    if (flags & SIGNAL_FLAG_SPIKE) {
        msg          = "تداخل كهربائي / EMI";
        border_color = 0xE74C3Cu;   /* red */
    } else if (flags & SIGNAL_FLAG_HIGH_NOISE) {
        msg          = "تربة مُعدِّنة — اخفض الحساسية";
        border_color = 0xE67E22u;   /* orange */
    } else if (flags & SIGNAL_FLAG_DRIFT) {
        msg          = "الجهاز يتكيف مع التربة";
        border_color = 0xF1C40Fu;   /* yellow */
    } else {
        msg          = "حرِّك الجهاز ببطء أكثر";
        border_color = 0xF39C12u;   /* amber */
    }

    /* Append confidence score to message: "EMI [conf: 45%]" */
    char full_msg[80];
    snprintf(full_msg, sizeof(full_msg), "%s  [%u%%]", msg, (unsigned)confidence);

    lv_label_set_text(s_notif_label, full_msg);
    lv_obj_set_style_border_color(s_notif_container,
                                   lv_color_hex(border_color),
                                   LV_STATE_DEFAULT);

    lv_obj_clear_flag(s_notif_container, LV_OBJ_FLAG_HIDDEN);
    s_notif_flags_last = flags;
    s_notif_show_ms    = now_ms;
}


 /*
 * الـ bar_chart widget (bar_chart.c) يستخدم LV_EVENT_DRAW_MAIN على
 * plain lv_obj — رسم كامل في callback واحد بدل per-point callbacks.
 *
 * التكامل:
 *  - objects.live_scan_chart (lv_chart من EEZ) يُخفى تماماً
 *  - bar chart widget يُنشأ كـ sibling داخل objects.live_chart
 *  - handle_live_result يستدعي bar_chart_add_value() لكل عينة
 * ═══════════════════════════════════════════════════════════════════ */

static lv_obj_t *s_bar_chart_obj = NULL; 


static void chart_init_series(void)
{
    lv_obj_t *box = objects.live_chart;       /* EEZ container */
    lv_obj_t *old = objects.live_scan_chart;  /* EEZ lv_chart — we hide this */
    if (!box) return;

    /* Hide the EEZ-generated lv_chart — we replace it visually */
    if (old) lv_obj_add_flag(old, LV_OBJ_FLAG_HIDDEN);

    /* Destroy previous bar chart if chart_init_series called again */
    if (s_bar_chart_obj) {
        lv_obj_del(s_bar_chart_obj);
        s_bar_chart_obj = NULL;
    }

    /* Create our custom bar chart widget inside the container */
    s_bar_chart_obj = bar_chart_create(box);
    if (!s_bar_chart_obj) {
        ESP_LOGE(TAG, "bar_chart_create failed — OOM?");
        return;
    }

    ESP_LOGI(TAG, "BarChart ready inside live_chart (%dx%d)",
             (int)lv_obj_get_width(box),
             (int)lv_obj_get_height(box));

    /* Create the signal notification widget (floating top-left overlay) */
    notifier_init();
}



/* ═══════════════════════════════════════════════════════════════════
 * LIVE SCAN — result handler
 * ══════════════════════════════════════════════════════════════════
 * ═══════════════════════════════════════════════════════════════════
 * SNR PANEL UPDATE
 * ═══════════════════════════════════════════════════════════════════
 *
 * يُحدِّث ثلاثة عناصر في snr_panel:
 *   snr_progress_bar → قيمة [0..10] مع لون ديناميكي
 *   snr_val          → نص "X.X"
 *   info_for_user    → نص وصفي + لون
 *
 * stability: إذا false → كل شيء رمادي + "SNR not stable"
 *
 * Confidence = min(snr/5 × 100, 100)%
 * ═══════════════════════════════════════════════════════════════════ 
 * ═══════════════════════════════════════════════════════════════════
 * SNR PANEL UPDATE — Professional real-time indicator
 * ═══════════════════════════════════════════════════════════════════
 *
 * LAYOUT (108×48px panel):
 *   [SNR] ████████░░ [X.X]     ← snr_progress_bar + snr_val
 *   [Confidence]      [XX%]    ← static label   + confidence_value
 *         [info text]          ← info_for_user (recolor)
 *
 * PERFORMANCE OPTIMIZATIONS:
 *   1. Throttle: panel يُحدَّث بحد أقصى مرة كل 150ms (< 7 updates/sec)
 *      كافٍ للعين البشرية — يوفر ~80% من استدعاءات lv_* غير الضرورية
 *   2. Dirty check: لا تُحدِّث إلا إذا تغيرت القيمة بأكثر من 0.1
 *      يمنع إعادة الرسم المتكررة على نفس القيمة
 *   3. Bar: LV_ANIM_OFF للسرعة — الـ bar يُحرَّك بواسطة القيمة نفسها
 *
 * COLOR SCHEME:
 *   SNR < 1.0  → Green  #2ECC71  Clear soil / no target
 *   SNR 1–2    → Yellow #F1C40F  Weak signal / noise
 *   SNR 2–4    → Orange #E67E22  Possible target
 *   SNR > 4    → Red    #E74C3C  Strong target confirmed
 *   Unstable   → Grey   #7F8C8D  Device moving
 *
 * CONFIDENCE formula (piecewise linear, clamped [0..100]):
 *   SNR < 1  → 0–20%
 *   SNR 1–2  → 20–40%
 *   SNR 2–4  → 40–80%
 *   SNR > 4  → 80–100%
 * ═══════════════════════════════════════════════════════════════════ 

  ── حالة Throttle — static لمنع stack allocation في كل استدعاء ── */
static uint32_t s_snr_last_update_ms = 0u;
static float    s_snr_last_rendered  = -99.f;
static bool     s_snr_last_stable    = false;

#define SNR_UPDATE_INTERVAL_MS   150u    /* حد أقصى ~6.6 تحديث/ثانية       */
#define SNR_DIRTY_THRESHOLD      0.15f   /* أدنى تغيير يستدعي إعادة الرسم   */

/* ── Peak Hold — متغيرات الحالة ── */
static float    s_peak_snr        = 0.0f;   /* قيمة الذروة الحالية          */
static uint32_t s_peak_decay_ms   = 0u;     /* آخر وقت حُسب فيه التلاشي     */
static int      s_snr_zone        = 0;      /* منطقة اللون: 0=أخضر..3=أحمر  */
static uint32_t s_last_color      = 0u;     /* cache لون الشريط              */

/*
 * معاملات Peak Hold:
 *
 *   PEAK_DECAY_RATE    — معدل التلاشي الطبيعي: 98% محتفظ كل 100ms
 *                        الذروة تتلاشى تدريجياً خلال ~5 ثوانٍ
 *
 *   PEAK_JERK_THRESH   — عتبة القفزة المفاجئة في SNR:
 *                        إذا تغيّر SNR بأكثر من هذا في خطوة واحدة
 *                        دون استقرار → يُعتبر حركة مفاجئة → تلاشٍ فوري
 *
 *   PEAK_JERK_FACTOR   — معامل الإسقاط الفوري عند الحركة المفاجئة:
 *                        0.5 = خفض الذروة 50% فوراً
 *
 *   PEAK_UNSTABLE_FACTOR — معامل التلاشي أثناء الحركة العادية البطيئة
 */
#define PEAK_DECAY_RATE         0.98f   /* retain 98% per 100ms             */
#define PEAK_JERK_THRESH        1.5f    /* قفزة SNR مفاجئة → تلاشٍ قوي     */
#define PEAK_JERK_FACTOR        0.50f   /* إسقاط فوري 50% عند الجرك         */
#define PEAK_UNSTABLE_FACTOR    0.85f   /* خفض 15% كل دورة أثناء الحركة    */
#define JERK_NOISE_FACTOR       2.0f    /* عامل noise_floor لعتبة الجرك      */
#define JERK_THRESH_MIN         0.5f    /* حد أدنى لعتبة الجرك التكيفية      */
#define JERK_THRESH_MAX         5.0f    /* حد أعلى لعتبة الجرك التكيفية      */

/* ═══════════════════════════════════════════════════════════════════
 * PEAK HOLD — دالة مساعدة مستقلة
 * ═══════════════════════════════════════════════════════════════════
 *
 * ثلاثة سيناريوهات:
 *
 *  1. قيمة جديدة أعلى من الذروة + الجهاز مستقر
 *     → تسجيل الذروة الجديدة، تجميد عداد التلاشي
 *
 *  2. الجهاز مستقر لكن SNR أقل من الذروة
 *     → تلاشٍ طبيعي مبني على الزمن (مستقل عن FPS)
 *     → تحديث s_peak_decay_ms بعد كل حساب لمنع تراكم dt
 *
 *  3. الجهاز غير مستقر (حركة)
 *     3a. قفزة مفاجئة (snr_delta > PEAK_JERK_THRESH):
 *         → إسقاط فوري قوي (50%) — يمنع بقاء ذروة كاذبة
 *     3b. حركة عادية:
 *         → تلاشٍ معتدل (85%) كل دورة تحديث
 *
 * @param snr       القيمة الحالية [0..10]
 * @param stable    هل الجهاز مستقر؟
 * @param snr_delta تغيير SNR عن القراءة السابقة (قيمة مطلقة)
 * @param now_ms    الوقت الحالي بالمللي ثانية
 */
static void peak_hold_update(float snr, bool stable,
                             float snr_delta, uint32_t now_ms)
{
    if (stable && snr >= s_peak_snr) {
        /* ── سيناريو 1: ذروة جديدة ── */
        s_peak_snr     = snr;
        s_peak_decay_ms = now_ms;

    } else if (stable) {
        /* ── سيناريو 2: تلاشٍ طبيعي مبني على الزمن ── */
        float dt_s = (float)(now_ms - s_peak_decay_ms) / 1000.0f;
        if (dt_s < 0.0f) dt_s = 0.0f;

        /* powf(0.98, dt*10) = 98% محتفظة كل 100ms */
        s_peak_snr    *= powf(PEAK_DECAY_RATE, dt_s * 10.0f);
        s_peak_decay_ms = now_ms;   /* تحديث المرجع بعد كل تلاشٍ */

        if (s_peak_snr < 0.01f) s_peak_snr = 0.0f;

    } else {
        /* ── سيناريو 3: غير مستقر (حركة) ── */
        /* عتبة الجرك التكيفية: JERK_NOISE_FACTOR × noise_floor
         * إذا كان noise_floor غير متاح (قبل المعايرة) نستخدم الثابت 1.5 */
        float jerk_thresh = PEAK_JERK_THRESH;
        {
            const DeviceProfile_t *dp = devcal_get_profile();
            if (dp && dp->noise_floor > 0.01f) {
                jerk_thresh = JERK_NOISE_FACTOR * dp->noise_floor;
                /* حد أدنى معقول */
                if (jerk_thresh < 0.5f) jerk_thresh = 0.5f;
            }
        }
        /* clamp العتبة التكيفية [MIN..MAX] */
        if (jerk_thresh < JERK_THRESH_MIN) jerk_thresh = JERK_THRESH_MIN;
        if (jerk_thresh > JERK_THRESH_MAX) jerk_thresh = JERK_THRESH_MAX;

        if (snr_delta > jerk_thresh) {
            /* 3a: قفزة مفاجئة — إسقاط فوري */
            s_peak_snr *= PEAK_JERK_FACTOR;
        } else {
            /* 3b: حركة عادية — تلاشٍ معتدل */
            s_peak_snr *= PEAK_UNSTABLE_FACTOR;
        }
        s_peak_decay_ms = now_ms;
        if (s_peak_snr < 0.01f) s_peak_snr = 0.0f;
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * UPDATE SNR PANEL — مؤشر احترافي للوقت الحقيقي
 * ═══════════════════════════════════════════════════════════════════
 *
 * المميزات:
 *   1. Clamping:    حماية من NaN / Inf / قيم خارج النطاق
 *   2. Hysteresis:  state machine بهوامش ±0.2 — بدون وميض اللون
 *                   الزون يُحفظ عند عدم الاستقرار (لا يُصفَّر)
 *   3. Peak Hold:   عرض الذروة في الشريط مع تلاشٍ زمني مستقل عن FPS
 *                   + إسقاط فوري عند حركة مفاجئة (jerk detection)
 *   4. Confidence:  خطي متعدد القطع [0..100%] مُقيَّد عند SNR=4.5
 *   5. Color cache: تحديث لون الشريط فقط عند التغيير
 *   6. Throttle:    حد أقصى ~6.6 تحديث/ثانية
 *   7. Dirty check: تجاهل التحديثات < SNR_DIRTY_THRESHOLD
 *
 * نظام الألوان:
 *   SNR < 1.0  → أخضر    #2ECC71  (تربة نظيفة / لا هدف)
 *   SNR 1–2    → أصفر    #F1C40F  (إشارة ضعيفة)
 *   SNR 2–4    → برتقالي #E67E22  (هدف محتمل)
 *   SNR > 4    → أحمر    #E74C3C  (هدف واضح)
 *   غير مستقر → رمادي   #7F8C8D  (جهاز يتحرك)
 *
 * الشريط:
 *   يعرض max(snr, s_peak_snr) — يحتفظ بالذروة بصرياً حتى تتلاشى
 * ═══════════════════════════════════════════════════════════════════ */

static void update_snr_panel(float snr, bool stable)
{
    /* ── guard: التأكد من وجود عناصر الواجهة ── */
    if (!objects.snr_progress_bar ||
        !objects.snr_val          ||
        !objects.confidence_value ||
        !objects.info_for_user)   return;

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    /* ── Throttle + dirty check ── */
    float delta = snr - s_snr_last_rendered;
    if (delta < 0.f) delta = -delta;

    bool need_update = (now_ms - s_snr_last_update_ms) >= SNR_UPDATE_INTERVAL_MS
                    || (delta >= SNR_DIRTY_THRESHOLD)
                    || (stable != s_snr_last_stable);

    if (!need_update) return;

    s_snr_last_update_ms = now_ms;
    s_snr_last_stable    = stable;

    /* ── Clamping: حماية من قيم غير صالحة ── */
    if (!isfinite(snr) || snr < 0.0f) snr = 0.0f;
    if (snr > 10.0f) snr = 10.0f;

    /* ── Peak Hold: حساب الذروة + jerk detection ── */
    float snr_abs_delta = snr - s_snr_last_rendered;
    if (snr_abs_delta < 0.f) snr_abs_delta = -snr_abs_delta;
    s_snr_last_rendered = snr;

    peak_hold_update(snr, stable, snr_abs_delta, now_ms);

    /* ── Hysteresis: state machine للمنطقة اللونية ──
     *
     * عند عدم الاستقرار: نحتفظ بالزون الأخير مجمَّداً.
     * يضمن العودة التدريجية للون الصحيح عند الاستقرار مجدداً.
     */
    if (stable) {
        switch (s_snr_zone) {
            case 0:   /* أخضر */
                if (snr > 1.2f) { s_snr_zone = 1; }
                break;
            case 1:   /* أصفر */
                if      (snr > 2.2f) { s_snr_zone = 2; }
                else if (snr < 0.8f) { s_snr_zone = 0; }
                break;
            case 2:   /* برتقالي */
                if      (snr > 4.2f) { s_snr_zone = 3; }
                else if (snr < 1.8f) { s_snr_zone = 1; }
                break;
            case 3:   /* أحمر */
                if (snr < 3.8f) { s_snr_zone = 2; }
                break;
            default:
                s_snr_zone = 0;
                break;
        }
    }
    /* غير مستقر: الزون يبقى محفوظاً — يُعرض رمادي في الواجهة 

     * ── تحديد اللون والنص ── */
    uint32_t    hex_color;
    const char *info_text;
    char        snr_buf[10];

    if (!stable) {
        hex_color = SNR_COLOR_GREY;
        info_text = "#7F8C8D Moving...#";
        snr_buf[0] = '-'; snr_buf[1] = '-'; snr_buf[2] = '\0';
    } else {
        switch (s_snr_zone) {
            case 1:
                hex_color = SNR_COLOR_YELLOW;
                info_text = "#F1C40F Weak signal#";
                break;
            case 2:
                hex_color = SNR_COLOR_ORANGE;
                info_text = "#E67E22 Possible target#";
                break;
            case 3:
                hex_color = SNR_COLOR_RED;
                info_text = "#E74C3C Strong target!#";
                break;
            default:   /* case 0 */
                hex_color = SNR_COLOR_GREEN;
                info_text = "#2ECC71 Clear soil#";
                break;
        }
        snprintf(snr_buf, sizeof(snr_buf), "%.1f", (double)snr);
    }

    /* ── Confidence: خطي متعدد القطع مُقيَّد عند SNR=4.5 ── */
    uint8_t conf = 0u;
    if (stable) {
        float sc = (snr < 4.5f) ? snr : 4.5f;
        if      (sc < 1.0f) conf = (uint8_t)(sc * 20.0f);
        else if (sc < 2.0f) conf = (uint8_t)((sc - 1.0f) * 20.0f + 20.0f);
        else if (sc < 4.0f) conf = (uint8_t)((sc - 2.0f) * 20.0f + 40.0f);
        else                conf = (uint8_t)((sc - 4.0f) * 40.0f + 80.0f);
        if (conf > 100u) conf = 100u;
    }

    lv_color_t color = lv_color_hex(hex_color);

    /* ── Progress bar: يعرض max(snr, peak) لبقاء الذروة مرئية ──
     *  SNR=5 → 100%
     */
    float display_snr = (s_peak_snr > snr) ? s_peak_snr : snr;
    int32_t bar_val   = stable ? (int32_t)(display_snr * 20.0f) : 0;
    if (bar_val > 100) bar_val = 100;
    lv_bar_set_value(objects.snr_progress_bar, bar_val, LV_ANIM_OFF);

    /* تحديث لون الشريط فقط عند التغيير (توفير CPU) */
    if (hex_color != s_last_color) {
        lv_obj_set_style_bg_color(objects.snr_progress_bar, color,
                                  LV_PART_INDICATOR | LV_STATE_DEFAULT);
        s_last_color = hex_color;
    }

    /* ── قيمة SNR النصية ── */
    lv_label_set_text(objects.snr_val, snr_buf);
    lv_obj_set_style_text_color(objects.snr_val, color,
                                LV_PART_MAIN | LV_STATE_DEFAULT);

    /* ── نسبة الثقة ── */
    char conf_buf[8];
    snprintf(conf_buf, sizeof(conf_buf), "%u%%", (unsigned)conf);
    lv_label_set_text(objects.confidence_value, conf_buf);
    lv_obj_set_style_text_color(objects.confidence_value, color,
                                LV_PART_MAIN | LV_STATE_DEFAULT);

    /* ── النص الوصفي ── */
    lv_label_set_text(objects.info_for_user, info_text);
}

/* ═══════════════════════════════════════════════════════════════════
 * SMART DETECTION LAYER — طبقة الكشف الذكي للأهداف والفراغات
 * ═══════════════════════════════════════════════════════════════════
 *
 * تُحلِّل الإشارة في الوقت الحقيقي لتحديد:
 *   - وجود هدف (معدن / فراغ / لا هدف)
 *   - ثقة الكشف [0..100%]
 *
 * المدخلات:
 *   snr            — نسبة الإشارة إلى الضوضاء [0..∞)
 *   deviation      — الانحراف الحالي عن baseline (موجب=معدن، سالب=فراغ)
 *   stability      — درجة استقرار الإشارة [0..100] من signal_processor
 *   stable         — هل تجاوز الاستقرار عتبة SNR_STABLE_WINDOW_MS؟
 *   prev_deviation — الانحراف في العينة السابقة (لحساب زخم الإشارة)
 *
 * خوارزمية الثقة:
 *   base_conf = clamp(snr × 20, 0, 100)
 *
 *   تعديل الزخم (momentum):
 *     إشارة تنمو  (|dev| > |prev| × 1.1 و|dev|>2): conf × 1.10
 *     إشارة تتراجع (|dev| < |prev| × 0.8):          conf × 0.90
 *     إشارة ثابتة:                                   بدون تعديل
 *
 *   تعديل الاستقرار:
 *     stability < 70  → conf × 0.60
 *     stability < 85  → conf × 0.85
 *     stability ≥ 85  → بدون تعديل
 *
 * الألوان — معدن:   أخضر(30%) → أصفر(60%) → أحمر
 * الألوان — فراغ:   أزرق فاتح → أزرق متوسط → أزرق داكن
 *
 * الأداء:
 *   - Throttle: حد أقصى 5 تحديثات/ثانية
 *   - s_detection_last_update_ms يُحدَّث دائماً عند تجاوز Throttle
 *   - Dirty check على النص + الثقة (فرق > 5%)
 *   - Static buffers فقط — لا تخصيص ذاكرة ديناميكي
 * ═══════════════════════════════════════════════════════════════════ */

typedef enum {
    DETECTION_NONE  = 0,
    DETECTION_METAL = 1,
    DETECTION_VOID  = 2,
} DetectionType_t;

/* حالة الكشف الذكي */
static uint32_t     s_detection_last_update_ms = 0u;
static char         s_detection_last_msg[48]   = "";
static float        s_detection_last_conf      = -1.0f;
static DetectionType_t s_detection_last_type   = DETECTION_NONE; /* Confidence Hysteresis */
static uint32_t     s_detection_type_change_ms = 0u;             /* Time Hysteresis       */
static uint32_t     s_detection_hold_until_ms  = 0u;             /* Temporal Hold         */
static float        s_det_momentum             = 0.0f;           /* EMA زخم مُخمَّد       */

#define DETECTION_UPDATE_INTERVAL_MS   200u   /* حد أقصى 5 تحديثات/ثانية            */
#define DETECTION_CONF_THRESHOLD       5.0f   /* تغيير > 5% يستدعي render            */
#define DETECTION_MIN_SNR              1.5f   /* SNR أدنى لاعتبار الهدف              */
#define DETECTION_MIN_DEV              2.0f   /* deviation أدنى (LSB)                */
#define DETECTION_MIN_STABILITY        70.0f  /* استقرار أدنى [0..100]               */
#define DETECTION_TYPE_HYST_CONF       40.0f  /* أدنى ثقة للتبديل بين Metal/Void      */
#define DETECTION_TIME_HYST_MS         300u   /* زمن تجميد النوع بعد التبديل (ms)     */
#define DETECTION_HOLD_BASE_MS         120u   /* أدنى hold time  (ms) — إشارة ضعيفة   */
#define DETECTION_HOLD_CONF_SCALE      3.5f   /* ms إضافية لكل 1% ثقة                 */
#define DETECTION_HOLD_MAX_MS          550u   /* أقصى hold time  (ms) عند conf=100     */
#define DETECTION_HOLD_CONF_DECAY      0.92f  /* تلاشٍ الثقة per update (~200ms)       */
#define DETECTION_MOMENTUM_ALPHA       0.30f  /* معامل EMA للزخم (بطيء ومستقر)        */
#define DETECTION_MOMENTUM_UNSTABLE    0.20f  /* decay سريع بدل reset كامل            */
#define DETECTION_MOMENTUM_MAX         3.0f   /* ceiling للزخم — يمنع saturation       */

/* Void consecutive — تكيّفي حسب التربة:
 *   noise_floor > 5 LSB (تربة صاخبة) → 4 عينات
 *   noise_floor ≤ 5 LSB (تربة هادئة) → 3 عينات */
#define DETECTION_VOID_CONSEC_NOISY    4u     /* تأكيد في التربة الصاخبة             */
#define DETECTION_VOID_CONSEC_QUIET    3u     /* تأكيد في التربة الهادئة             */
#define DETECTION_VOID_NOISE_THRESH    5.0f   /* حد noise_floor للتمييز بين الحالتين */

/**
 * @brief حساب ثقة الكشف — DeepSeek improvement: Momentum + Hold مدمجان
 *
 * الفكرة القديمة: طبقتان منفصلتان (momentum يُعدِّل confidence، ثم
 * temporal hold يُمدد العرض بشكل مستقل).
 * المشكلة: هدف عميق ضعيف يدخل momentum بطيئاً → conf=25% فقط →
 *   time hysteresis تمنع التبديل → الشاشة لا تستجيب لـ 300ms إضافية.
 *   مع بطء الكشف الأصلي → يفوت الهدف كلياً في مسح سريع.
 *
 * الحل: نقاط ثقة موحدة تُعبر عن كل الأبعاد:
 *   SNR_score   [0..50]  — الجزء الفيزيائي الحقيقي
 *   dev_score   [0..30]  — قوة الانحراف نسبةً للـ noise floor
 *   mom_score   [0..15]  — الزخم: يُعجِّل عند النمو، يُبطئ عند التراجع
 *   stab_score  [0..5]   — مكافأة الاستقرار
 *
 * الفوائد:
 *   - هدف عميق (SNR=2, dev=3×noise): score = 20+18+0+5 = 43% (يُظهَر!)
 *   - إشارة زائفة (SNR=2, dev=1×noise): score = 20+6+0+0 = 26% (يُخفى)
 *   - هدف قوي نامٍ: momentum +15% → score يتجاوز عتبة أسرع بـ 40%
 */
static float compute_detection_confidence(float snr,
                                          float deviation,
                                          float prev_deviation,
                                          float stability)
{
    /* ── SNR component [0..50] ─────────────────────────────────────
     * SNR=1.5 → 15, SNR=5 → 50 (capped). Linear feels natural. */
    float snr_score = snr * 10.0f;
    if (snr_score > 50.0f) snr_score = 50.0f;

    /* ── Deviation component [0..30] ───────────────────────────────
     * Normalised by noise floor — measures true signal-to-floor ratio.
     * noise_floor = sqrtf(noise_variance) but we receive only deviation
     * here. Proxy: scale deviation by fixed reference 10 LSB.
     * dev = 5× noise_floor → score = 30 (strong target at moderate depth).
     * dev = 1× noise_floor → score = 6  (deep/weak, still detectable). */
    float abs_dev  = deviation >= 0.0f ? deviation : -deviation;
    float dev_score = abs_dev * (30.0f / 10.0f);  /* ref: 10 LSB = full score */
    if (dev_score > 30.0f) dev_score = 30.0f;

    /* ── Momentum component [-8..+15] ──────────────────────────────
     * Growing signal:  +15% bonus → faster confirmation on rising edge
     * Decaying signal: -8% penalty → faster release to avoid ghost hold
     * Peak/plateau:     0  → hold at current confidence (correct)
     *
     * Unified with hold: momentum now directly feeds hold duration.
     * No separate temporal hold calculation needed. */
    float raw_delta = abs_dev - (prev_deviation >= 0.0f ? prev_deviation : -prev_deviation);
    s_det_momentum  = 0.70f * s_det_momentum
                    + DETECTION_MOMENTUM_ALPHA * raw_delta;

    if (s_det_momentum >  DETECTION_MOMENTUM_MAX) s_det_momentum =  DETECTION_MOMENTUM_MAX;
    if (s_det_momentum < -DETECTION_MOMENTUM_MAX) s_det_momentum = -DETECTION_MOMENTUM_MAX;
    if (fabsf(s_det_momentum) < 0.05f) s_det_momentum = 0.0f;

    float mom_score = 0.0f;
    float mom_thresh = DETECTION_MIN_DEV * 0.10f;
    if      (s_det_momentum >  mom_thresh) mom_score = +15.0f;  /* growing  */
    else if (s_det_momentum < -mom_thresh) mom_score =  -8.0f;  /* decaying */

    /* ── Stability bonus [0..5] ─────────────────────────────────────
     * Small bonus only — stability is already gated upstream.
     * High stability → small reward. Low stability → no penalty
     * (penalising here caused misses during sweep acceleration). */
    float stab_score = 0.0f;
    if      (stability >= 85.0f) stab_score = 5.0f;
    else if (stability >= 70.0f) stab_score = 2.0f;

    /* ── Total confidence ───────────────────────────────────────── */
    float conf = snr_score + dev_score + mom_score + stab_score;
    if (conf > 100.0f) conf = 100.0f;
    if (conf <   0.0f) conf =   0.0f;
    return conf;
}

/**
 * @brief الحصول على اللون بناءً على الثقة ونوع الهدف
 *
 * معدن: أخضر → أصفر → أحمر
 * فراغ: أزرق فاتح → متوسط → داكن  (تمييز بصري واضح)
 */
static uint32_t get_detection_color(DetectionType_t type, float confidence)
{
    if (type == DETECTION_NONE) return 0x7F8C8Du;

    if (type == DETECTION_METAL) {
        if      (confidence < 30.0f) return 0x27AE60u;   /* أخضر          */
        else if (confidence < 60.0f) return 0xF39C12u;   /* أصفر برتقالي  */
        else                         return 0xE74C3Cu;   /* أحمر واضح     */
    } else {
        /* DETECTION_VOID: ألوان زرقاء متدرجة */
        if      (confidence < 30.0f) return 0x85C1E9u;   /* أزرق فاتح     */
        else if (confidence < 60.0f) return 0x2E86C1u;   /* أزرق متوسط    */
        else                         return 0x1A5276u;   /* أزرق داكن     */
    }
}

/**
 * @brief تحديث عنصر smart_detection في الواجهة
 *
 * @param snr            نسبة الإشارة إلى الضوضاء
 * @param deviation      الانحراف الحالي عن baseline
 * @param stability      درجة الاستقرار [0..100]
 * @param stable         هل الجهاز مستقر (SNR window)؟
 * @param prev_deviation الانحراف في العينة السابقة
 * @param noise_floor    ضوضاء التربة من device calibration
 *
 * طبقات الحماية المطبَّقة (بالترتيب):
 *
 *  A. Momentum EMA + rapid decay عند الحركة
 *     × 0.20 per update بدل reset كامل — يمنع ghost targets
 *     ويحتفظ بذاكرة خفيفة تُسرِّع العودة بعد حركة قصيرة
 *
 *  B. Adaptive Void consecutive confirmation
 *     تربة هادئة → 3 عينات، تربة صاخبة → 4 عينات
 *     يحوّل spike عرضية → لا شيء، وpattern حقيقي → هدف
 *
 *  C. Confidence Hysteresis (Metal ↔ Void)
 *     لا تبديل النوع إلا بثقة ≥ DETECTION_TYPE_HYST_CONF (40%)
 *
 *  D. Time Hysteresis
 *     لا تبديل النوع قبل مرور DETECTION_TIME_HYST_MS (300ms)
 *     حتى لو كانت الثقة كافية — يمنع التبديل السريع
 *
 *  E. Adaptive Temporal Hold + dt-based EMA confidence decay
 *     Hold duration = clamp(120 + conf×3.5, 120, 550) ms — يتكيف مع قوة الهدف
 *     أثناء Hold: confidence × 0.92 per update → تلاشٍ سلس بصرياً
 *     يمنع تقطيع الهدف الواحد ويعطي إشارة طبيعية للتلاشي
 */
static void smart_detection_update(float snr,
                                   float deviation,
                                   float stability,
                                   bool  stable,
                                   float prev_deviation,
                                   float noise_floor)
{
    if (!objects.smart_detection) return;

    /* ── Throttle ── */
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if ((now_ms - s_detection_last_update_ms) < DETECTION_UPDATE_INTERVAL_MS) return;
    s_detection_last_update_ms = now_ms;

    /* ── A. Momentum: decay سريع عند الحركة، EMA عند الاستقرار ──
     * × DETECTION_MOMENTUM_UNSTABLE (0.20) بدل reset كامل:
     *   - يُبطئ الزخم فوراً (80% خفض per update) → يمنع ghost targets
     *   - يحتفظ بذاكرة خفيفة → عودة أسرع للكشف بعد حركة قصيرة
     * في حالة اهتزاز شديد (كل دورة) → 0.2^3 = 0.008 ≈ صفر عملياً
     */
    if (!stable) {
        s_det_momentum *= DETECTION_MOMENTUM_UNSTABLE;
    }

    /* ── B. Adaptive Void consecutive confirmation ──
     * عدد التأكيدات يتكيف مع ضوضاء التربة:
     *   noise_floor > DETECTION_VOID_NOISE_THRESH → 4 عينات (تربة صاخبة)
     *   noise_floor ≤ DETECTION_VOID_NOISE_THRESH → 3 عينات (تربة هادئة)
     */
    static uint8_t s_void_consec = 0u;
    uint8_t void_min = (noise_floor > DETECTION_VOID_NOISE_THRESH)
                       ? DETECTION_VOID_CONSEC_NOISY
                       : DETECTION_VOID_CONSEC_QUIET;

    if (stable && deviation < -DETECTION_MIN_DEV) {
        if (s_void_consec < void_min) s_void_consec++;
    } else {
        s_void_consec = 0u;
    }

    /* ── تحديد نوع الهدف الخام ── */
    DetectionType_t raw_type = DETECTION_NONE;

    bool conditions_met = (stable
                           && stability  >= DETECTION_MIN_STABILITY
                           && snr        >= DETECTION_MIN_SNR
                           && fabsf(deviation) >= DETECTION_MIN_DEV);

    if (conditions_met) {
        if (deviation > 0.0f) {
            raw_type = DETECTION_METAL;
        } else if (s_void_consec >= void_min) {
            raw_type = DETECTION_VOID;
        }
    }

    /* ── E. Temporal Hold ──
     * إذا اختفى الهدف → أبقِ آخر نتيجة لمدة تكيفية (HOLD_BASE..HOLD_MAX).
     * يمنع تقطيع هدف واحد إلى جزئين عند مرور فوقه.
     * ملاحظة: لا نُبقي النوع أثناء الحركة (!stable) — فقط عند توقف الإشارة.
     */
    DetectionType_t type = raw_type;

    if (type == DETECTION_NONE && stable
        && s_detection_last_type != DETECTION_NONE
        && now_ms < s_detection_hold_until_ms)
    {
        /* لا يزال في نافذة Hold — أبقِ النوع الأخير بثقة متراجعة */
        type = s_detection_last_type;
    }

    if (raw_type != DETECTION_NONE) {
        /* هدف جديد → احسب Hold التكيفي بناءً على الثقة الحالية
         * hold = clamp(BASE + conf × SCALE, BASE, MAX)
         *   conf=30%  → ~270ms
         *   conf=80%  → ~470ms
         *   conf=100% → 550ms (مقيَّد بـ MAX=600)
         */
        float cur_conf = compute_detection_confidence(snr, deviation,
                                                      prev_deviation, stability);
        uint32_t hold_ms = (uint32_t)(DETECTION_HOLD_BASE_MS
                           + cur_conf * DETECTION_HOLD_CONF_SCALE);
        if (hold_ms < DETECTION_HOLD_BASE_MS) hold_ms = DETECTION_HOLD_BASE_MS;
        if (hold_ms > DETECTION_HOLD_MAX_MS)  hold_ms = DETECTION_HOLD_MAX_MS;
        s_detection_hold_until_ms = now_ms + hold_ms;
    }

    /* ── C+D. Confidence + Time Hysteresis (Metal ↔ Void) ──
     * لا تبديل النوع إلا إذا:
     *   1. مرّت DETECTION_TIME_HYST_MS على آخر تبديل
     *   AND
     *   2. الثقة ≥ DETECTION_TYPE_HYST_CONF
     * يمنع الوميض حتى لو SNR متذبذب عند حدود الهدف.
     */
    float confidence = 0.0f;

    if (type != DETECTION_NONE && type != s_detection_last_type
        && s_detection_last_type != DETECTION_NONE)
    {
        bool time_ok = (now_ms - s_detection_type_change_ms) >= DETECTION_TIME_HYST_MS;
        if (time_ok) {
            confidence = compute_detection_confidence(snr, deviation,
                                                      prev_deviation, stability);
            if (confidence < DETECTION_TYPE_HYST_CONF) {
                type = s_detection_last_type;   /* ثقة غير كافية */
            }
        } else {
            type = s_detection_last_type;       /* زمن غير كافٍ */
        }
    }

    /* حساب الثقة النهائية */
    char new_msg[48];
    new_msg[0] = '\0';

    if (type != DETECTION_NONE) {
        if (confidence < 0.01f) {   /* لم تُحسب بعد */
            confidence = compute_detection_confidence(snr, deviation,
                                                      prev_deviation, stability);
        }
        /* تراجع الثقة أثناء Hold بـ EMA مبني على الزمن الحقيقي (dt-based):
         *   powf(0.92, dt) حيث dt مُعيَّر على 200ms
         * → decay ثابت بغض النظر عن تغيّر الـ update rate
         * → تلاشٍ سلس ومريح بصرياً بدون قفز مفاجئ للصفر
         */
        if (raw_type == DETECTION_NONE && now_ms < s_detection_hold_until_ms) {
            float dt_norm = (float)(now_ms - s_detection_last_update_ms + 1u)
                          / 200.0f;   /* معيَّر على 200ms */
            /* clamp dt_norm: يمنع drop مفاجئ عند lag أو update متأخر */
            if (dt_norm > 3.0f) dt_norm = 3.0f;
            confidence *= powf(DETECTION_HOLD_CONF_DECAY, dt_norm);
            /* Confidence floor: يمنع الاختفاء المفاجئ — ذيل بصري طبيعي */
            if (confidence < 10.0f) confidence = 10.0f;
        }
        snprintf(new_msg, sizeof(new_msg),
                 (type == DETECTION_METAL) ? "Metal: %.0f%%" : "Void: %.0f%%",
                 (double)confidence);
    }

    /* ── Dirty check ── */
    bool text_changed = (strcmp(new_msg, s_detection_last_msg) != 0);
    bool conf_changed = (fabsf(confidence - s_detection_last_conf)
                         >= DETECTION_CONF_THRESHOLD);

    if (!text_changed && !conf_changed) return;

    /* ── تطبيق التغييرات على الواجهة ── */
    lv_label_set_text(objects.smart_detection, new_msg);
    lv_obj_set_style_text_color(
        objects.smart_detection,
        lv_color_hex(get_detection_color(type, confidence)),
        LV_PART_MAIN | LV_STATE_DEFAULT);

    /* تسجيل زمن تبديل النوع (للـ Time Hysteresis) */
    if (type != s_detection_last_type) {
        s_detection_type_change_ms = now_ms;
    }

    strcpy(s_detection_last_msg, new_msg);
    s_detection_last_conf  = confidence;
    s_detection_last_type  = type;
}

static void handle_live_result(const ProcessedSample_t *s)
{
    /* ── أثناء المعايرة: progress bar فقط ── */
    if (s_calibrating) {
        /* FIX: Old sentinel check (stability < -0.5f) REMOVED.
         * Phase 2 is now signaled via SYS_EVT_CALIB_PHASE2 event in handle_event().
         * This function only handles result queue samples — no more magic values. */
        uint8_t pct = (uint8_t)s->output_scaled;
        if (pct > 100u) pct = 100u;
        lv_bar_set_value(objects.calibration_progress_bar,
                         (int32_t)pct, LV_ANIM_ON);
        if (pct < 50u) {
            lv_label_set_text(objects.calibration_label_statut, ">Phase 1: Hold still");
        } else {
            lv_label_set_text(objects.calibration_label_statut, ">Phase 2: Walk slowly");
        }
        return;
    }

    /*
     * تحقق أن Live Scan tab هو المفعَّل حالياً.
     * إذا كان المستخدم في tab آخر — لا chart، لا text update، لا رسم.
     * الـ signal_task يستمر بالمعالجة وحفظ last_result للـ scan،
     * لكن LVGL لا يُثقَّل بتحديثات غير مرئية.
     */
    if (objects.tabview &&
        lv_tabview_get_tab_act(objects.tabview) != 0) {
        buzzer_live_stop();   /* أوقف النغمة عند مغادرة Live Scan */
        return;
    }

    char buf[16];

    /* baseline_text */
    snprintf(buf, sizeof(buf), "%.1f", (double)s->baseline);
    lv_textarea_set_text(objects.baseline_text, buf);

    /* values_text — قيمة ADS1115 بعد الفلترة */
    snprintf(buf, sizeof(buf), "%u", (unsigned)s->output_scaled);
    lv_textarea_set_text(objects.values_text, buf);

    /* لون values_text بحسب شدة الانحراف */
    float dev = s->deviation < 0.f ? -s->deviation : s->deviation;
    lv_color_t col;
    if      (dev < 10.f) col = lv_color_hex(0x0d7547);  /* أخضر — هادئ     */
    else if (dev < 35.f) col = lv_color_hex(0xFFC107);  /* أصفر — تنبيه    */
    else                 col = lv_color_hex(0xcd4755);  /* أحمر — كشف هدف  */
    lv_obj_set_style_text_color(objects.values_text, col,
                                LV_PART_MAIN | LV_STATE_DEFAULT);

    /* ── تحديث الـ Bar Chart ──────────────────────────────────────────
     * نُمرر كلاً من output_scaled (لارتفاع العمود) وdeviation
     * (للون الدقيق بوحدات LSB الحقيقية من الـ DSP pipeline).
     * bar_chart_add_value: يُزيح كل الأعمدة يساراً ويُضيف الجديد.
     * lv_obj_invalidate تُطلق LV_EVENT_DRAW_MAIN مرة واحدة — فعّال.
     * ─────────────────────────────────────────────────────────────── */
    if (s_bar_chart_obj) {
        bar_chart_add_value(s_bar_chart_obj, s->output_scaled, s->deviation);
    }

    /* ── Signal notification widget ────────────────────────────────── */
    notifier_update(s->signal_flags, s->confidence);

    /* ── Uncalibrated / Warmup indicator ──────────────────────────────
     * Three sub-states:
     *  1. Warmup (<100 samples): "جارٍ ضبط Baseline... N%"  — amber, progress
     *  2. Warmup done, uncalibrated: "Baseline تلقائي (512)" — grey, info
     *  3. Calibrated: nothing shown here (signal_flags handles the rest)
     *
     * Wording "Baseline تلقائي" instead of "غير مُعاير":
     *   ✓ No error connotation — the system IS working
     *   ✓ Informs user that auto-baseline is active
     *   ✓ Gives confidence that readings are valid
     *
     * Priority: only shown when signal_flags == NONE (no signal issue).
     * ─────────────────────────────────────────────────────────────── */
    extern bool    signal_task_is_calibrated(void);
    extern uint8_t signal_task_get_warmup_pct(void);

    if (!signal_task_is_calibrated() && s->signal_flags == SIGNAL_FLAG_NONE
        && s_notif_container && s_notif_label) {

        uint8_t wpct = signal_task_get_warmup_pct();

        if (wpct < 100u) {
            /* ── Warmup in progress ── */
            char warmup_buf[36];
            snprintf(warmup_buf, sizeof(warmup_buf),
                     "ثبّت الجهاز... %u%%", (unsigned)wpct);

            if (lv_obj_has_flag(s_notif_container, LV_OBJ_FLAG_HIDDEN) ||
                s_notif_flags_last != 0xFEu) {   /* 0xFE = warmup sentinel */
                lv_label_set_text(s_notif_label, warmup_buf);
                lv_obj_set_style_border_color(s_notif_container,
                                               lv_color_hex(0xF39C12u), /* amber */
                                               LV_STATE_DEFAULT);
                lv_obj_clear_flag(s_notif_container, LV_OBJ_FLAG_HIDDEN);
                s_notif_flags_last = 0xFEu;
            } else {
                /* Already showing warmup — just update the percentage */
                lv_label_set_text(s_notif_label, warmup_buf);
            }

        } else {
            /* ── Warmup done, uncalibrated mode ── */
            if (lv_obj_has_flag(s_notif_container, LV_OBJ_FLAG_HIDDEN) ||
                s_notif_flags_last == 0xFEu) {
                lv_label_set_text(s_notif_label, "Baseline تلقائي (512)");
                lv_obj_set_style_border_color(s_notif_container,
                                               lv_color_hex(0x7F8C8Du), /* grey */
                                               LV_STATE_DEFAULT);
                lv_obj_clear_flag(s_notif_container, LV_OBJ_FLAG_HIDDEN);
                s_notif_flags_last = 0xFFu;   /* 0xFF = uncalibrated-stable sentinel */
            }
        }
    }

    /*
     * ── Live Audio Engine ──
     * تُحدَّث كل LIVE_UPDATE_MS (50Hz).
     * تأخذ deviation + snr + confidence لتوليد صوت ذكي متكيف.
     * confidence: نأخذها من s_detection_last_conf (آخر قيمة من smart_detection).
     */
    buzzer_live_tone(s->deviation, s->snr, s_detection_last_conf);

    /* ── SNR Panel Update ──────────────────────────────────────────
     * نحسب استقرار الإشارة: إذا تغير SNR بأقل من 0.3 لمدة 800ms
     * نعتبر الجهاز ثابتاً ونُظهر الألوان الحقيقية.
     * أثناء الحركة → رمادي + "SNR not stable".
     * ─────────────────────────────────────────────────────────── */
    {
        float snr = s->snr;
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        /* تحقق من الاستقرار — إذا تغير SNR كثيراً أعد العداد */
        float snr_delta = snr - s_snr_last;
        if (snr_delta < 0.f) snr_delta = -snr_delta;

        if (snr_delta > 0.5f) {
            /* تغير مفاجئ — إعادة تشغيل نافذة الاستقرار */
            s_snr_stable_since_ms = now_ms;
            s_snr_is_stable       = false;
        } else if (!s_snr_is_stable &&
                   (now_ms - s_snr_stable_since_ms) >= SNR_STABLE_WINDOW_MS) {
            s_snr_is_stable = true;
        }
        s_snr_last = snr;

        update_snr_panel(snr, s_snr_is_stable);

        /* ── Smart Detection Layer ── */
        static float prev_deviation = 0.0f;
        float det_noise_floor = 0.0f;
        {
            const DeviceProfile_t *dp = devcal_get_profile();
            if (dp) det_noise_floor = dp->noise_floor;
        }
        smart_detection_update(snr, s->deviation, s->stability,
                               s_snr_is_stable, prev_deviation,
                               det_noise_floor);
        prev_deviation = s->deviation;
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * BLUETOOTH ICON
 * ═══════════════════════════════════════════════════════════════════ */

static void update_bt_icon(BtConnectionState_t st)
{
    if (st == s_last_bt) return;
    s_last_bt = st;

    switch (st) {
        case BT_STATE_CONNECTED:
            lv_obj_add_flag  (objects.bluetooth_status_idle,    LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag  (objects.bluetooth_status_failed,  LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(objects.bluetooth_status_success, LV_OBJ_FLAG_HIDDEN);
            log_push("[BT] Connected");
            break;

        case BT_STATE_ERROR:
            lv_obj_add_flag  (objects.bluetooth_status_idle,    LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(objects.bluetooth_status_failed,  LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag  (objects.bluetooth_status_success, LV_OBJ_FLAG_HIDDEN);
            log_push("[BT] ERROR / Disconnected");
            break;

        default:   /* OFF, CONNECTING */
            lv_obj_clear_flag(objects.bluetooth_status_idle,    LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag  (objects.bluetooth_status_failed,  LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag  (objects.bluetooth_status_success, LV_OBJ_FLAG_HIDDEN);
            break;
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * ═══════════════════════════════════════════════════════════════════
 * AUTO SCAN — countdown UI (ديناميكي داخل auto_scan_process)
 *
 * تصميم احترافي بـ 3 طبقات:
 *   1. Arc دائري → يتقلص مع انقضاء الوقت (مثل stopwatch)
 *   2. رقم الثواني → يُحرَّك بـ zoom+fade كل ثانية (slot machine effect)
 *   3. "Step N/M" → يتوهج باللون الأصفر/الأخضر حسب التقدم
 *
 * LVGL v8 animations:
 *   - lv_anim_t على lv_obj_set_style_opa  (fade)
 *   - lv_anim_t على lv_obj_set_style_transform_zoom (zoom)
 *   - lv_arc_set_value() على arc دائري مع LV_ANIM_ON
 * ═══════════════════════════════════════════════════════════════════ 

 * مؤشرات واجهة الـ countdown — تُعرَّف في قسم private state أعلاه 

  ── Animation callbacks (LVGL v8) ──────────────────────────────── */
static void _anim_opa_cb(void *obj, int32_t v)
{
    lv_obj_set_style_opa((lv_obj_t *)obj, (lv_opa_t)v, 0);
}

static void _anim_zoom_cb(void *obj, int32_t v)
{
    lv_obj_set_style_transform_zoom((lv_obj_t *)obj, (uint16_t)v, 0);
}

/* ── animate_cd_number: يُشغَّل عند تغيير الثانية فقط ─────────────
 * Motion: zoom 130%→100% مع fade 0→255 في 220ms
 * اللون يتغير تدريجياً: أخضر (start) → أصفر (middle) → أحمر (urgent)
 * ──────────────────────────────────────────────────────────────── */
static void animate_cd_number(lv_obj_t *lbl, lv_color_t new_col)
{
    /* أولاً: ضع اللون الجديد */
    lv_obj_set_style_text_color(lbl, new_col, 0);

    /* Zoom: من 333 (130%) إلى 256 (100%) */
    lv_anim_t az;
    lv_anim_init(&az);
    lv_anim_set_var(&az, lbl);
    lv_anim_set_exec_cb(&az, _anim_zoom_cb);
    lv_anim_set_values(&az, 333, 256);
    lv_anim_set_time(&az, 220);
    lv_anim_set_path_cb(&az, lv_anim_path_ease_out);
    lv_anim_start(&az);

    /* Fade: من 0 إلى 255 */
    lv_anim_t ao;
    lv_anim_init(&ao);
    lv_anim_set_var(&ao, lbl);
    lv_anim_set_exec_cb(&ao, _anim_opa_cb);
    lv_anim_set_values(&ao, 0, 255);
    lv_anim_set_time(&ao, 180);
    lv_anim_set_path_cb(&ao, lv_anim_path_ease_in_out);
    lv_anim_start(&ao);
}

static void autoscan_create_ui(void)
{
    lv_obj_t *panel = objects.auto_scan_process;
    if (!panel) return;

    s_cd_last_sec = -1;   /* إعادة تعيين tracker الثانية */

    lv_coord_t pw = lv_obj_get_width(panel);
    lv_coord_t ph = lv_obj_get_height(panel);

    /* ══ 1. Arc دائري (مثل ساعة stopwatch) ══════════════════════
     * يبدأ ممتلئاً (360°) وينقص مع كل ثانية حتى يصل 0.
     * لون الـ arc يتغير مع اللون النصي (أخضر→أصفر→أحمر).
     * ═══════════════════════════════════════════════════════════ */
    lv_coord_t arc_sz = (pw < ph ? pw : ph) - 12;
    if (arc_sz < 30) arc_sz = 30;

    s_arc_cd = lv_arc_create(panel);
    lv_obj_set_size(s_arc_cd, arc_sz, arc_sz);
    lv_obj_align(s_arc_cd, LV_ALIGN_CENTER, 0, -4);

    lv_arc_set_rotation(s_arc_cd, 270);          /* نقطة البداية: الأعلى (12 o'clock) */
    lv_arc_set_bg_angles(s_arc_cd, 0, 360);      /* خلفية الـ arc: دائرة كاملة */
    lv_arc_set_range(s_arc_cd, 0, 100);
    lv_arc_set_value(s_arc_cd, 100);
    lv_arc_set_mode(s_arc_cd, LV_ARC_MODE_NORMAL);

    /* تعطيل الـ knob (لا تفاعل) */
    lv_obj_remove_style(s_arc_cd, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(s_arc_cd, LV_OBJ_FLAG_CLICKABLE);

    /* تنسيق الـ arc */
    lv_obj_set_style_arc_width(s_arc_cd, 5, LV_PART_MAIN);
    lv_obj_set_style_arc_width(s_arc_cd, 5, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(s_arc_cd, lv_color_hex(0x1C2130u),
                               LV_PART_MAIN);           /* خلفية: رمادي داكن */
    lv_obj_set_style_arc_color(s_arc_cd, lv_color_hex(0x00FF88u),
                               LV_PART_INDICATOR);      /* إرشاد ابتدائي: أخضر */

    /* ══ 2. رقم الثواني (وسط الـ arc) ═══════════════════════════ */
    s_lbl_cd_secs = lv_label_create(panel);
    lv_label_set_text(s_lbl_cd_secs, "--");
    lv_obj_set_style_text_font(s_lbl_cd_secs, &lv_font_montserrat_30,
                               LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(s_lbl_cd_secs, lv_color_hex(0x00FF88u),
                                LV_PART_MAIN | LV_STATE_DEFAULT);
    /* pivot للـ zoom animation — مركز النص */
    lv_obj_set_style_transform_pivot_x(s_lbl_cd_secs, 0, 0);
    lv_obj_set_style_transform_pivot_y(s_lbl_cd_secs, 0, 0);
    lv_obj_align(s_lbl_cd_secs, LV_ALIGN_CENTER, 0, -6);

    /* ══ 3. "Step N/M" (أسفل الرقم) ════════════════════════════ */
    s_lbl_cd_steps = lv_label_create(panel);
    lv_label_set_text(s_lbl_cd_steps, "Step -/-");
    lv_obj_set_style_text_font(s_lbl_cd_steps, &lv_font_montserrat_12,
                               LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(s_lbl_cd_steps, lv_color_hex(0xFFC107u),
                                LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(s_lbl_cd_steps, LV_ALIGN_CENTER, 0, 20);

    /* ══ 4. شريط خطي رفيع (تحت كل شيء) — كـ fallback بصري ════ */
    s_bar_cd = lv_bar_create(panel);
    lv_obj_set_size(s_bar_cd, pw - 14, 3);
    lv_obj_align(s_bar_cd, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_bar_set_range(s_bar_cd, 0, 100);
    lv_bar_set_value(s_bar_cd, 100, LV_ANIM_OFF);
    lv_obj_set_style_radius(s_bar_cd, 2, LV_PART_MAIN);
    lv_obj_set_style_radius(s_bar_cd, 2, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s_bar_cd, lv_color_hex(0x1C2130u),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_bar_cd, lv_color_hex(0x00FF88u),
                              LV_PART_INDICATOR);
}

static void autoscan_refresh_labels(void)
{
    if (!s_lbl_cd_secs || !s_lbl_cd_steps) return;

    /* ثوانٍ متبقية */
    int32_t secs = (s_cd_ms + 999) / 1000;
    if (secs < 0) secs = 0;

    /* نسبة التقدم 0..100 */
    int32_t total_ms = (int32_t)s_secs_total * 1000;
    int32_t pct      = (total_ms > 0)
                       ? (int32_t)((s_cd_ms * 100LL) / total_ms)
                       : 0;
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;

    /* اللون التدرجي الثلاثي حسب الوقت المتبقي */
    lv_color_t col;
    uint32_t col_hex;
    if      (pct > 66) { col_hex = 0x00FF88u; col = lv_color_hex(col_hex); }
    else if (pct > 33) { col_hex = 0xFFC107u; col = lv_color_hex(col_hex); }
    else               { col_hex = 0xFF3366u; col = lv_color_hex(col_hex); }

    /* ── Animation عند تغيير الثانية فقط (لا كل 20ms) ──────────
     * تُشغَّل zoom+fade فقط لحظة انتقال الرقم.
     * يُجنِّب الـ CPU حمل animations متكررة في كل دورة مهمة. */
    if (secs != s_cd_last_sec) {
        s_cd_last_sec = secs;
        char buf[8];
        snprintf(buf, sizeof(buf), "%ld", (long)secs);
        lv_label_set_text(s_lbl_cd_secs, buf);
        animate_cd_number(s_lbl_cd_secs, col);
    }

    /* تحديث الـ Arc الدائري */
    if (s_arc_cd) {
        lv_arc_set_value(s_arc_cd, (int16_t)pct);
        lv_obj_set_style_arc_color(s_arc_cd, col, LV_PART_INDICATOR);
    }

    /* تحديث الشريط الخطي السفلي */
    if (s_bar_cd) {
        lv_bar_set_value(s_bar_cd, pct, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(s_bar_cd, col, LV_PART_INDICATOR);
    }

    /* تحديث Step N/M */
    char buf[20];
    snprintf(buf, sizeof(buf), "Step %u / %u",
             (unsigned)(s_steps_total - s_steps_left + 1),
             (unsigned)s_steps_total);
    lv_label_set_text(s_lbl_cd_steps, buf);
}

/* ─── Dialog Next Line / Finish ──────────────────────────────────── */

static void on_next_line(lv_event_t *e)
{
    (void)e;
    if (s_bt) bt_new_line(s_bt);

    /* إعادة تعيين للخط الجديد */
    s_steps_left = s_steps_total;
    s_cd_ms      = (int32_t)s_secs_total * 1000;
    s_as_state   = AS_COUNTDOWN;

    if (s_dialog) { lv_obj_del(s_dialog); s_dialog = NULL; }

    char msg[40];
    snprintf(msg, sizeof(msg), "[SCAN] Line %u started",
             s_bt ? (unsigned)bt_get_session(s_bt)->current_line : 0u);
    log_push(msg);

    if (UI_LOCK()) {
        lv_textarea_set_text(objects.state_infos, ">Next line...");
        autoscan_refresh_labels();
        UI_UNLOCK();
    }
    ESP_LOGI(TAG, "Auto scan — new line");
}

static void on_finish(lv_event_t *e)
{
    (void)e;
    if (s_bt) bt_session_end(s_bt);
    s_as_state    = AS_IDLE;
    s_cd_last_sec = -1;   /* reset animation tracker */

    if (s_dialog)  { lv_obj_del(s_dialog);  s_dialog  = NULL; }

    /* Nullify countdown widget pointers — panel children will be deleted
     * when the panel is hidden/re-created. Avoids dangling pointer use. */
    s_lbl_cd_secs  = NULL;
    s_lbl_cd_steps = NULL;
    s_bar_cd       = NULL;
    s_arc_cd       = NULL;

    if (UI_LOCK()) {
        lv_obj_add_flag(objects.auto_scan_process,    LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(objects.auto_scan_option_panel, LV_OBJ_FLAG_HIDDEN);
        lv_textarea_set_text(objects.state_infos, ">Scan complete");
        UI_UNLOCK();
    }
    log_push("[SCAN] Session finished");
    ESP_LOGI(TAG, "Auto scan — finished");
}

static void show_line_done_dialog(void)
{
    if (s_dialog) return;

    s_dialog = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_dialog, 210, 95);
    lv_obj_align(s_dialog, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(s_dialog, lv_color_hex(0x151520),
                              LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(s_dialog, lv_color_hex(0x00D4FF),
                                  LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(s_dialog, 1,
                                  LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(s_dialog, 7,
                            LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(s_dialog, LV_OBJ_FLAG_SCROLLABLE);

    /* عنوان */
    lv_obj_t *title = lv_label_create(s_dialog);
    lv_label_set_text(title, "Line complete!");
    lv_obj_set_style_text_color(title, lv_color_hex(0x00D4FF),
                                LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14,
                               LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    /* زر Next Line — أخضر */
    lv_obj_t *btn_n = lv_btn_create(s_dialog);
    lv_obj_set_size(btn_n, 84, 30);
    lv_obj_align(btn_n, LV_ALIGN_BOTTOM_LEFT, 8, -8);
    lv_obj_set_style_bg_color(btn_n, lv_color_hex(0x477f53),
                              LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(btn_n, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_event_cb(btn_n, on_next_line, LV_EVENT_RELEASED, NULL);
    lv_obj_t *lbl_n = lv_label_create(btn_n);
    lv_label_set_text(lbl_n, "Next Line");
    lv_obj_set_style_text_font(lbl_n, &lv_font_montserrat_14,
                               LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(lbl_n, LV_ALIGN_CENTER, 0, 0);

    /* زر Finish — أحمر */
    lv_obj_t *btn_f = lv_btn_create(s_dialog);
    lv_obj_set_size(btn_f, 84, 30);
    lv_obj_align(btn_f, LV_ALIGN_BOTTOM_RIGHT, -8, -8);
    lv_obj_set_style_bg_color(btn_f, lv_color_hex(0xcd4755),
                              LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(btn_f, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_event_cb(btn_f, on_finish, LV_EVENT_RELEASED, NULL);
    lv_obj_t *lbl_f = lv_label_create(btn_f);
    lv_label_set_text(lbl_f, "Finish");
    lv_obj_set_style_text_font(lbl_f, &lv_font_montserrat_14,
                               LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(lbl_f, LV_ALIGN_CENTER, 0, 0);
}

/* ═══════════════════════════════════════════════════════════════════
 * EVENT HANDLER — SysEventMsg_t → UI
 * ═══════════════════════════════════════════════════════════════════ */

static void handle_event(const SysEventMsg_t *evt)
{
    switch (evt->event) {

        /* ── انتقال المعايرة إلى المرحلة الثانية ── */
        case SYS_EVT_CALIB_PHASE2: {
            /* FIX: This event replaces the old stability=-1.0f sentinel.
             * Sent by signal_task when calibration enters the walking phase.
             * Handled here (event queue) not in handle_live_result (result queue)
             * — correct separation of concerns. */
            lv_bar_set_value(objects.calibration_progress_bar, 50, LV_ANIM_ON);
            lv_label_set_text(objects.calibration_label_statut,
                              ">Phase 2: Walk slowly 20s");
            log_push("[CAL] Phase 2 — walk slowly");
            buzzer_beep_nb(1000, 100);   /* beep 1 */
            vTaskDelay(pdMS_TO_TICKS(180));
            buzzer_beep_nb(1000, 100);   /* beep 2 — double beep = phase change */
            break;
        }

        /* ── اكتملت المعايرة ── */
        case SYS_EVT_CALIB_DONE: {
            s_calibrating = false;

            /* اخفِ progress bar وأظهِر الزر */
            lv_bar_set_value(objects.calibration_progress_bar, 100, LV_ANIM_ON);
            /* نترك الـ bar يُكمل الـ animation ثم نُخفيه بعد 500ms */
            lv_obj_add_flag  (objects.calibration_button,       LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(objects.calibration_progress_bar, LV_OBJ_FLAG_HIDDEN);

            bool ok = (bool)(evt->data);

            /* LED */
            lv_color_t led = ok ? lv_color_hex(0x477f53)
                                : lv_color_hex(0xcd4755);
            lv_led_set_color(objects.led_calibration_status, led);
            lv_led_set_brightness(objects.led_calibration_status, 255);

            if (ok) {
                lv_label_set_text(objects.calibration_label_statut, ">Done");
                lv_textarea_set_text(objects.state_infos, ">Calibration OK");
                lv_obj_clear_state(objects.manual_scan_button, LV_STATE_DISABLED);
                lv_obj_clear_state(objects.auto_scan_button,   LV_STATE_DISABLED);
                log_push("[CAL] Success");
                /* نُخفي الـ bar فوراً — الـ ANIM_ON كافٍ للتأثير البصري */
                lv_obj_add_flag  (objects.calibration_progress_bar, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(objects.calibration_button,       LV_OBJ_FLAG_HIDDEN);
                buzzer_beep_nb(1000, 80);
            } else {
                lv_label_set_text(objects.calibration_label_statut, ">Failed");
                lv_textarea_set_text(objects.state_infos, ">Calibration FAILED");
                log_push("[CAL] FAILED — retry");
                lv_obj_add_flag  (objects.calibration_progress_bar, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(objects.calibration_button,       LV_OBJ_FLAG_HIDDEN);
                buzzer_beep_nb(300, 250);
            }
            break;
        }

        /* ── خطأ فادح ── */
        case SYS_EVT_FAULT: {
            char msg[52];
            snprintf(msg, sizeof(msg), "[ERR] Fault 0x%02lX",
                     (unsigned long)evt->data);
            lv_textarea_set_text(objects.state_infos, msg + 6);
            log_push(msg);
            ESP_LOGE(TAG, "%s", msg);
            break;
        }

        default:
            break;
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * AUTO SCAN TICK — كل TASK_PERIOD_MS
 * ═══════════════════════════════════════════════════════════════════ */

static void auto_scan_tick(void)
{
    if (s_as_state != AS_COUNTDOWN) return;

    s_cd_ms -= (int32_t)TASK_PERIOD_MS;

    /* تحديث labels */
    if (UI_LOCK()) {
        autoscan_refresh_labels();
        UI_UNLOCK();
    }

    if (s_cd_ms > 0) return;

    /* ═══════════════════════════════════════════════════════════════
     * STABILITY GATE — Adaptive Sampling
     *
     * إذا كان stability_gate checkbox مُفعَّلاً:
     *   - نراقب تذبذب الإشارة قبل أخذ العينة
     *   - threshold = STAB_K × noise_floor من device calibration
     *   - MIN wait: 1.5s  MAX wait: 5s ثم نُجبر العينة
     *
     * يعمل في Auto Scan فقط.
     * ═══════════════════════════════════════════════════════════════ */
    if (s_stab_gate_on && s_has_sample) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        /* جلب noise_floor من device calibration */
        float noise_fl = 1.0f;
        if (devcal_is_valid()) {
            noise_fl = devcal_get_profile()->noise_floor;
            if (noise_fl < 0.3f) noise_fl = 0.3f;
        }
        float threshold = STAB_K * noise_fl;

        /* بدء نافذة الانتظار عند أول وصول للعداد صفر */
        if (!s_stab_waiting) {
            s_stab_waiting   = true;
            s_stab_start_ms  = now_ms;
            /* أعِد العداد بـ TASK_PERIOD_MS واحد — نبقى في هذا frame */
            s_cd_ms = (int32_t)TASK_PERIOD_MS;
            return;
        }

        uint32_t elapsed = now_ms - s_stab_start_ms;
        float    abs_dev = s_last_sample.deviation < 0.f
                           ? -s_last_sample.deviation
                           : s_last_sample.deviation;

        bool stable   = (abs_dev < threshold) && (elapsed >= STAB_MIN_MS);
        bool timedout = (elapsed >= STAB_MAX_MS);

        if (!stable && !timedout) {
            /* لا يزال يتذبذب — انتظر دورة أخرى */
            s_cd_ms = (int32_t)TASK_PERIOD_MS;
            return;
        }

        /* مستقر أو انتهى الوقت → خذ العينة */
        s_stab_waiting = false;
        if (timedout && !stable) {
            ESP_LOGD("AutoScan", "Stability gate timeout after %lums — forced",
                     (unsigned long)elapsed);
        }
    } else {
        /* gate مُعطَّل أو لا sample — أعد المؤشر */
        s_stab_waiting = false;
    }

    /* ═══ خذ المسحة ═══ */
    uint16_t val = s_has_sample ? s_last_sample.output_scaled : 512u;

    /* إرسال BT — scan message (أولوية عالية) مع retry لحماية البيانات */
    bool line_done = false;
    if (s_bt) {
        for (int retry = 0; retry < 3; retry++) {
            line_done = bt_enqueue_scan(s_bt, val);
            if (line_done) break;
            vTaskDelay(pdMS_TO_TICKS(5)); /* انتظار قصير بين المحاولات */
        }

        if (!line_done) {
            /* فشلت كل المحاولات → بلوتوث عالق! لا تخصم الخطوة وأظهر خطأ */
            log_push("[ERR] BT queue stuck! Point dropped.");
            if (UI_LOCK()) {
                lv_textarea_set_text(objects.state_infos, ">ERR: BT Queue Stuck!");
                UI_UNLOCK();
            }
            /* أعد العداد ليعيد المحاولة في الدورة القادمة */
            s_cd_ms = (int32_t)s_secs_total * 1000;
            return;
        }
    }

    /* تأثير بصري: وميض values_text بلون الـ BT */
    if (UI_LOCK()) {
        lv_obj_set_style_text_color(objects.values_text,
                                    lv_color_hex(0x00D4FF),
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
        UI_UNLOCK();
    }

    /*
     * ── Step-confirm beep (Auto Scan) ──
     * 800Hz / 90ms — واضح ومعتدل مثل أجهزة Garrett/Minelab
     * يُعلم المستخدم باكتمال الخطوة وإمكانية التحرك
     */
    buzzer_beep_nb(800, 90);

    /* استعادة اللون الأصلي */
    if (UI_LOCK()) {
        float dev = s_has_sample
                    ? (s_last_sample.deviation < 0.f
                        ? -s_last_sample.deviation
                        :  s_last_sample.deviation)
                    : 0.f;
        lv_color_t col = (dev < 10.f) ? lv_color_hex(0x0d7547)
                       : (dev < 35.f) ? lv_color_hex(0xFFC107)
                                      : lv_color_hex(0xcd4755);
        lv_obj_set_style_text_color(objects.values_text, col,
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
        UI_UNLOCK();
    }

    /* إنقاص خطوة */
    if (s_steps_left > 0) --s_steps_left;

    char logmsg[40];
    snprintf(logmsg, sizeof(logmsg), "[SCAN] val=%u step=%u/%u",
             (unsigned)val,
             (unsigned)(s_steps_total - s_steps_left),
             (unsigned)s_steps_total);
    log_push(logmsg);

    if (s_steps_left == 0 || line_done) {
        /* ═══ اكتمل السطر ═══ */
        s_as_state = AS_LINE_DONE;

        /* beep مزدوج عبر timer — لا تأخير */
        buzzer_beep_nb(1200, 180);   /* يُشغِّل ويوقف تلقائياً */

        log_push("[SCAN] Line done");

        if (UI_LOCK()) {
            show_line_done_dialog();
            UI_UNLOCK();
        }
    } else {
        /* إعادة العداد للقيمة الأصلية */
        s_cd_ms = (int32_t)s_secs_total * 1000;
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ═══════════════════════════════════════════════════════════════════ */


/* ═══════════════════════════════════════════════════════════════════
 * FILTER CONTROLS
 * ═══════════════════════════════════════════════════════════════════ */

void ui_set_kalman(bool enabled)
{
    signal_task_set_kalman(enabled);

    /* If Kalman is disabled, also uncheck kalman_spatial in UI */
    if (!enabled) {
        signal_task_set_spatial(false);
        if (UI_LOCK()) {
            if (objects.kalman_spatial)
                lv_obj_clear_state(objects.kalman_spatial, LV_STATE_CHECKED);
            UI_UNLOCK();
        }
    }

    char buf[32];
    snprintf(buf, sizeof(buf), ">Kalman: %s", enabled ? "ON" : "OFF");
    log_push(buf);
    ESP_LOGI("UI", "Kalman filter: %s", enabled ? "enabled" : "disabled");
}

void ui_set_kalman_spatial(bool enabled)
{
    if (enabled) {
        /* Enabling Spatial auto-enables Kalman — update UI checkbox too */
        signal_task_set_spatial(true);
        if (UI_LOCK()) {
            if (objects.kalman_filter)
                lv_obj_add_state(objects.kalman_filter, LV_STATE_CHECKED);
            UI_UNLOCK();
        }
        log_push(">Kalman+Spatial: ON [Experimental]");
    } else {
        signal_task_set_spatial(false);
        log_push(">Spatial: OFF");
    }
    ESP_LOGI("UI", "Kalman+Spatial: %s", enabled ? "enabled" : "disabled");
}

/* ═══════════════════════════════════════════════════════════════════
 * BT OUTPUT MODE (scan_mode dropdown)
 * 0 = RAW        — Kalman + Baseline only, no processing
 * 1 = NORMALIZED — Normalize by noise_std + Soft Clip
 * 2 = ENHANCED   — Full Pipeline B with Sensitivity Scale + Drift
 * ═══════════════════════════════════════════════════════════════════ */


/* ═══════════════════════════════════════════════════════════════════
 * HEADING COMPENSATION — 3D SCAN ONLY
 * ═══════════════════════════════════════════════════════════════════
 * Applied only at scan point transmission (not in Live scan).
 * Goal: make Upper/Lower sensors behave as paired twins →
 *       gradient = 0 when no target, regardless of scan direction.
 * heading: 0=N  1=E  2=S  3=W  4=disabled
 * ═══════════════════════════════════════════════════════════════════ */

void ui_set_scan_heading(uint8_t heading)
{
    signal_task_set_heading(heading);
    const char *names[] = {"North","East","South","West","OFF"};
    if (heading > 4) heading = 4;
    char buf[32];
    snprintf(buf, sizeof(buf), ">Heading: %s", names[heading]);
    log_push(buf);
    ESP_LOGI("UI", "Scan heading: %s", names[heading]);
}

void ui_set_heading_comp(bool enable)
{
    signal_task_set_heading_comp(enable);
    log_push(enable ? ">Earth field comp: ON" : ">Earth field comp: OFF");
    ESP_LOGI("UI", "Heading compensation: %s", enable ? "ON" : "OFF");
}


/* ═══════════════════════════════════════════════════════════════════
 * BOOST MODE
 * ═══════════════════════════════════════════════════════════════════
 * OFF: Live=8   Scan=32   Beep=800Hz/90ms
 * ON:  Live=64  Scan=256  Beep=1000Hz/150ms (max depth focus)
 * ═══════════════════════════════════════════════════════════════════ */

void ui_set_boost_mode(bool enable)
{
    signal_task_set_boost(enable);

    /* Update live oversample immediately */
    adc_task_set_oversample(enable ? 64 : 8);

    if (UI_LOCK()) {
        lv_textarea_set_text(objects.state_infos,
            enable ? ">Boost Mode: ON (max depth)" : ">Boost Mode: OFF");
        UI_UNLOCK();
    }
    log_push(enable ? "[BOOST] ON — 256 samples/point" : "[BOOST] OFF");
    ESP_LOGI("UI", "Boost mode: %s", enable ? "ON" : "OFF");
}

void ui_set_stability_gate(bool enabled)
{
    s_stab_gate_on = enabled;
    s_stab_waiting = false;   /* إلغاء أي انتظار جارٍ */

    if (UI_LOCK()) {
        lv_textarea_set_text(objects.state_infos,
            enabled ? ">Adaptive Sampling: ON" : ">Adaptive Sampling: OFF");
        UI_UNLOCK();
    }
    log_push(enabled ? "[STAB] Adaptive Sampling ON" : "[STAB] Adaptive Sampling OFF");
    ESP_LOGI("UI", "Stability gate: %s", enabled ? "ON" : "OFF");
}

void ui_set_bt_mode(uint8_t idx)
{
    signal_task_set_bt_mode(idx);

    const char *mode_names[] = {"RAW", "Normalize", "Enhanced"};
    if (idx > 2) idx = 0;

    char buf[32];
    snprintf(buf, sizeof(buf), ">BT mode: %s", mode_names[idx]);
    log_push(buf);
    ESP_LOGI("UI", "BT output mode: %s", mode_names[idx]);
}

void ui_request_calibration(void)
{
    if (s_calibrating) return;
    s_calibrating = true;

    /* Clear chart history — fresh start after calibration */
    if (s_bar_chart_obj) bar_chart_clear(s_bar_chart_obj);

    if (UI_LOCK()) {
        lv_bar_set_value(objects.calibration_progress_bar, 0, LV_ANIM_OFF);
        lv_obj_clear_flag(objects.calibration_progress_bar, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag  (objects.calibration_button,       LV_OBJ_FLAG_HIDDEN);
        lv_led_set_color(objects.led_calibration_status,
                         lv_color_hex(0xFFC107));
        lv_led_set_brightness(objects.led_calibration_status, 180);
        lv_label_set_text(objects.calibration_label_statut, ">Running");
        lv_textarea_set_text(objects.state_infos, ">Calibrating...");
        UI_UNLOCK();
    }

    log_push("[CAL] Started");
    qm_event_send(SYS_EVT_TOUCH_CALIB, 0);
    ESP_LOGI(TAG, "Calibration requested");
}

void ui_request_manual_scan(void)
{
    if (UI_LOCK()) {
        lv_obj_clear_flag(objects.manual_scan_process, LV_OBJ_FLAG_HIDDEN);
        lv_textarea_set_text(objects.state_infos, ">Manual scan active");
        UI_UNLOCK();
    }
    adc_task_set_oversample(signal_task_boost_enabled() ? 256 : 64);
    signal_task_set_scan_mode(true);
    log_push("[SCAN] Manual started (64 samples/point)");
    qm_event_send(SYS_EVT_TOUCH_SCAN, 0);
    ESP_LOGI(TAG, "Manual scan started — oversample=64");
}

void ui_request_auto_scan(uint8_t steps, uint8_t seconds)
{
    /* تحقق من الاتصال قبل البدء */
    if (!s_bt || !bt_is_ready(s_bt)) {
        if (UI_LOCK()) {
            lv_textarea_set_text(objects.state_infos, ">BT not connected!");
            UI_UNLOCK();
        }
        log_push("[WARN] Auto scan: BT not ready");
        return;
    }

    /* clamp القيم */
    s_steps_total = (steps   < 2)  ? 2  : (steps   > 50) ? 50 : steps;
    s_secs_total  = (seconds < 1)  ? 1  : (seconds > 10) ? 10 : seconds;
    s_steps_left  = s_steps_total;
    s_cd_ms       = (int32_t)s_secs_total * 1000;
    s_as_state    = AS_COUNTDOWN;

    bt_session_start(s_bt, s_steps_total);

    if (UI_LOCK()) {
        lv_obj_add_flag  (objects.auto_scan_option_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(objects.auto_scan_process,      LV_OBJ_FLAG_HIDDEN);
        autoscan_refresh_labels();
        char msg[48];
        snprintf(msg, sizeof(msg), ">Auto: %u steps / %us",
                 (unsigned)s_steps_total, (unsigned)s_secs_total);
        lv_textarea_set_text(objects.state_infos, msg);
        UI_UNLOCK();
    }

    adc_task_set_oversample(signal_task_boost_enabled() ? 256 : 32);
    signal_task_set_scan_mode(true);
    char logmsg[48];
    snprintf(logmsg, sizeof(logmsg), "[SCAN] Auto %u steps %us",
             (unsigned)s_steps_total, (unsigned)s_secs_total);
    log_push(logmsg);
    /* أخبِر signal_task بأن auto scan بدأ → عطِّل الزر الفيزيائي */
    qm_event_send(SYS_EVT_TOUCH_SCAN, 1u);
    ESP_LOGI(TAG, "%s", logmsg);
}

void ui_cancel_scan(void)
{
    s_as_state = AS_IDLE;
    if (s_bt) bt_session_end(s_bt);

    if (UI_LOCK()) {
        lv_obj_add_flag(objects.auto_scan_process,    LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(objects.manual_scan_process,  LV_OBJ_FLAG_HIDDEN);
        if (s_dialog) { lv_obj_del(s_dialog); s_dialog = NULL; }
        lv_textarea_set_text(objects.state_infos, ">Scan cancelled");
        UI_UNLOCK();
    }
    adc_task_set_oversample(signal_task_boost_enabled() ? 64 : 8);
    signal_task_set_scan_mode(false);
    signal_task_set_heading(4);         /* Disable heading comp on cancel */
    /* أخبِر signal_task بإلغاء → أعِد تفعيل الزر الفيزيائي */
    qm_event_send(SYS_EVT_TOUCH_SCAN, 0xFFFFFFFFu);
    log_push("[SCAN] Cancelled by user");
    ESP_LOGI(TAG, "Scan cancelled");
}

void ui_set_sensitivity(uint8_t idx)
{
    /*
     * sensibility_settings dropdown order (EEZ Studio):
     *  "Very Low\nLow\nMedium\nHigh\nVery High"
     *   idx 0 = Very Low  → SENS_MODE_VERY_LOW  (5)
     *   idx 1 = Low       → SENS_MODE_LOW        (4)
     *   idx 2 = Medium    → SENS_MODE_MEDIUM      (3)
     *   idx 3 = High      → SENS_MODE_HIGH        (2)
     *   idx 4 = Very High → SENS_MODE_VERY_HIGH   (1)
     */
    static const SensitivityMode_t map[5] = {
        SENS_MODE_VERY_LOW,
        SENS_MODE_LOW,
        SENS_MODE_MEDIUM,
        SENS_MODE_HIGH,
        SENS_MODE_VERY_HIGH,
    };
    static const char *names[5] = {
        "Very Low", "Low", "Medium", "High", "Very High"
    };

    if (idx >= 5) return;
    if (!s_manual_sens) return;   /* لا تعمل إلا في وضع Manual */

    qm_event_send(SYS_EVT_SENS_CHANGE, (uint32_t)map[idx]);

    char msg[36];
    snprintf(msg, sizeof(msg), "[SENS] %s", names[idx]);
    log_push(msg);

    if (UI_LOCK()) {
        char info[36];
        snprintf(info, sizeof(info), ">Sensitivity: %s", names[idx]);
        lv_textarea_set_text(objects.state_infos, info);
        UI_UNLOCK();
    }
    ESP_LOGI(TAG, "%s", msg);
}

void ui_set_manual_sensitivity_mode(bool enabled)
{
    s_manual_sens = enabled;

    if (UI_LOCK()) {
        if (enabled) {
            lv_obj_clear_state(objects.sensibility_settings, LV_STATE_DISABLED);
            lv_textarea_set_text(objects.state_infos, ">Manual sensitivity");
        } else {
            lv_obj_add_state(objects.sensibility_settings, LV_STATE_DISABLED);
            qm_event_send(SYS_EVT_SENS_CHANGE, (uint32_t)SENS_MODE_AUTO);
            lv_textarea_set_text(objects.state_infos, ">Auto sensitivity");
        }
        UI_UNLOCK();
    }
    log_push(enabled ? "[SENS] Manual mode" : "[SENS] Auto mode");
    ESP_LOGI(TAG, "Sensitivity mode: %s", enabled ? "MANUAL" : "AUTO");
}

/* ═══════════════════════════════════════════════════════════════════
 * MAIN TASK
 * ═══════════════════════════════════════════════════════════════════ */

void ui_event_task(void *arg)
{
    s_bt = (BTSender_t *)arg;
    memset(&s_last_sample, 0, sizeof(s_last_sample));
    s_last_sample.output_scaled = 512;

    ESP_LOGI(TAG, "ui_event_task — Core%d Prio%d",
             (int)xPortGetCoreID(), (int)uxTaskPriorityGet(NULL));

    /* انتظر اكتمال init الشاشات + adc_task يبدأ */
    vTaskDelay(pdMS_TO_TICKS(350));

    /* ═══ BOOT SELF-CHECK ═══
     * يُشغِّل على boot_screen ويمنع الدخول للـ Home عند الفشل.
     * adc_task_get_driver() يُعيد مؤشر ADS1115Driver_t المُهيَّأ.
     */
    if (UI_LOCK()) {
        boot_selfcheck_run(adc_task_get_driver());
        UI_UNLOCK();
    }

    /* ═══ تهيئة UI الأولية ═══ */
    if (UI_LOCK()) {

        /* chart */
        chart_init_series();

        /* countdown UI */
        autoscan_create_ui();

        /* إخفاء panels */
        lv_obj_add_flag(objects.auto_scan_option_panel,    LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(objects.auto_scan_process,         LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(objects.manual_scan_process,       LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(objects.logs_text,                 LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(objects.low_battery_panel,         LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(objects.calibration_progress_bar,  LV_OBJ_FLAG_HIDDEN);

        /* أزرار 3D معطَّلة حتى المعايرة */
        lv_obj_add_state(objects.manual_scan_button, LV_STATE_DISABLED);
        lv_obj_add_state(objects.auto_scan_button,   LV_STATE_DISABLED);

        /* Sensitivity dropdown معطَّل (AUTO افتراضي) */
        lv_obj_add_state(objects.sensibility_settings, LV_STATE_DISABLED);

        /* LED معايرة — محايد */
        lv_led_set_color(objects.led_calibration_status, lv_color_hex(0x0b3759));
        lv_led_set_brightness(objects.led_calibration_status, 255);
        lv_label_set_text(objects.calibration_label_statut, ">Ready");

        /* BT icon — حالة أولية */
        update_bt_icon(BT_STATE_OFF);

        /* رسالة ترحيب */
        lv_textarea_set_text(objects.state_infos, ">Boot OK — Calibrate first");
        lv_textarea_set_text(objects.logs_text,   "");

        /* ── تسجيل callbacks على كل الـ objects ── */
        actions_register_all();

        UI_UNLOCK();
    }

    adc_task_set_oversample(8);    /* Default: live mode normal */
    log_push("[SYS] Device booted OK");

    TickType_t last_bt_tick = xTaskGetTickCount();

    for (;;) {

        /* ── 1. result_queue — drain: خذ آخر قيمة واعرضها ── */
        {
            ProcessedSample_t s;
            while (qm_result_receive(&s, 0)) {
                s_last_sample = s;
                s_has_sample  = true;
                /* Phase 2 transition is now handled via SYS_EVT_CALIB_PHASE2
                 * in the event queue below — no sentinel magic values here. */
            }

            if (s_has_sample) {
                if (UI_LOCK()) {
                    handle_live_result(&s_last_sample);
                    UI_UNLOCK();
                }
            }
        }

        /* ── 2. event_queue ── */
        {
            SysEventMsg_t evt;
            while (qm_event_receive(&evt, 0)) {
                if (UI_LOCK()) {
                    handle_event(&evt);
                    UI_UNLOCK();
                }
            }
        }

        /* ── 3. auto scan countdown ── */
        auto_scan_tick();

        /* ── 4. BT icon كل ثانية ── */
        {
            TickType_t now = xTaskGetTickCount();
            if ((now - last_bt_tick) >= pdMS_TO_TICKS(BT_CHECK_PERIOD_MS)) {
                last_bt_tick = now;
                if (s_bt) {
                    BtConnectionState_t st = bt_get_conn_state(s_bt);
                    if (UI_LOCK()) {
                        update_bt_icon(st);
                        UI_UNLOCK();
                    }
                }
            }
        }

        /* ── 5. logs_text — يُحدَّث فقط عند تغيير البفر ── */
        if (s_log_dirty) {
            s_log_dirty = false;
            if (UI_LOCK()) {
                lv_textarea_set_text(objects.logs_text, s_log);
                lv_textarea_set_cursor_pos(objects.logs_text,
                                           LV_TEXTAREA_CURSOR_LAST);
                UI_UNLOCK();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(TASK_PERIOD_MS));
    }

    vTaskDelete(NULL);
}

/* ═══════════════════════════════════════════════════════════════════
 * RESET DEVICE CALIBRATION — confirmation dialog + NVS erase + restart
 *
 * Called from actions.c when user presses reset_device_calib button.
 *
 * Dialog layout (320×240):
 *   ┌─────────────────────────────┐
 *   │  ⚠  Reset Device Calib?    │
 *   │                             │
 *   │  This will erase the 5-     │
 *   │  phase device profile.      │
 *   │  Touch calibration is       │
 *   │  NOT affected.              │
 *   │                             │
 *   │  [Cancel]      [Confirm]    │
 *   └─────────────────────────────┘
 * ═══════════════════════════════════════════════════════════════════ */

static lv_obj_t *s_reset_dialog = NULL;

static void reset_dlg_confirm_cb(lv_event_t *e)
{
    (void)e;
    /* Close dialog first */
    if (s_reset_dialog) {
        lv_obj_del(s_reset_dialog);
        s_reset_dialog = NULL;
    }

    /* Show brief feedback */
    if (UI_LOCK()) {
        lv_textarea_set_text(objects.state_infos, "Resetting device calib...");
        UI_UNLOCK();
    }
    lv_timer_handler();

    /* Erase ONLY device calibration — touch_cal preserved */
    ESP_LOGI("UI", "User confirmed device calibration reset");
    devcal_nvs_clear();

    /* Small delay so user sees the message */
    vTaskDelay(pdMS_TO_TICKS(800));

    /* Restart — boot logic will detect missing valid flag and run wizard */
    esp_restart();
}

static void reset_dlg_cancel_cb(lv_event_t *e)
{
    (void)e;
    if (s_reset_dialog) {
        lv_obj_del(s_reset_dialog);
        s_reset_dialog = NULL;
    }
}

void ui_request_reset_device_calib(void)
{
    /* Prevent duplicate dialogs */
    if (s_reset_dialog) return;

    if (!UI_LOCK()) return;

    /* ── modal overlay ── */
    lv_obj_t *overlay = lv_obj_create(lv_scr_act());
    s_reset_dialog = overlay;
    lv_obj_set_size(overlay, 320, 240);
    lv_obj_set_pos(overlay, 0, 0);
    lv_obj_set_style_bg_color(overlay, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(overlay, 180, 0);
    lv_obj_set_style_border_width(overlay, 0, 0);
    lv_obj_clear_flag(overlay, LV_OBJ_FLAG_SCROLLABLE);

    /* ── dialog box ── */
    lv_obj_t *box = lv_obj_create(overlay);
    lv_obj_set_size(box, 270, 160);
    lv_obj_align(box, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(box, lv_color_hex(0x1a1a2e), 0);
    lv_obj_set_style_border_color(box, lv_color_hex(0xFF8C00), 0);
    lv_obj_set_style_border_width(box, 2, 0);
    lv_obj_set_style_radius(box, 8, 0);
    lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);

    /* ── title ── */
    lv_obj_t *title = lv_label_create(box);
    lv_label_set_text(title, LV_SYMBOL_WARNING "  Reset Device Calib?");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFF8C00), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

    /* ── message ── */
    lv_obj_t *msg = lv_label_create(box);
    lv_label_set_text(msg,
        "Erases 5-phase device profile.\n"
        "Device will restart and run\n"
        "calibration wizard.\n\n"
        "Touch calibration: NOT affected.");
    lv_obj_set_style_text_color(msg, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(msg, &lv_font_montserrat_12, 0);
    lv_label_set_long_mode(msg, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(msg, 240);
    lv_obj_align(msg, LV_ALIGN_TOP_MID, 0, 34);

    /* ── [Cancel] button ── */
    lv_obj_t *btn_cancel = lv_btn_create(box);
    lv_obj_set_size(btn_cancel, 100, 30);
    lv_obj_align(btn_cancel, LV_ALIGN_BOTTOM_LEFT, 8, -8);
    lv_obj_set_style_bg_color(btn_cancel, lv_color_hex(0x333355), 0);
    lv_obj_set_style_border_color(btn_cancel, lv_color_hex(0x6666AA), 0);
    lv_obj_set_style_border_width(btn_cancel, 1, 0);
    lv_obj_set_style_radius(btn_cancel, 6, 0);
    lv_obj_t *lc = lv_label_create(btn_cancel);
    lv_label_set_text(lc, "Cancel");
    lv_obj_set_style_text_color(lc, lv_color_hex(0xAAAAAA), 0);
    lv_obj_set_style_text_font(lc, &lv_font_montserrat_12, 0);
    lv_obj_align(lc, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(btn_cancel, reset_dlg_cancel_cb, LV_EVENT_RELEASED, NULL);

    /* ── [Confirm] button ── */
    lv_obj_t *btn_confirm = lv_btn_create(box);
    lv_obj_set_size(btn_confirm, 100, 30);
    lv_obj_align(btn_confirm, LV_ALIGN_BOTTOM_RIGHT, -8, -8);
    lv_obj_set_style_bg_color(btn_confirm, lv_color_hex(0x5c1a1a), 0);
    lv_obj_set_style_border_color(btn_confirm, lv_color_hex(0xFF4444), 0);
    lv_obj_set_style_border_width(btn_confirm, 1, 0);
    lv_obj_set_style_radius(btn_confirm, 6, 0);
    lv_obj_t *lf = lv_label_create(btn_confirm);
    lv_label_set_text(lf, LV_SYMBOL_OK "  Confirm");
    lv_obj_set_style_text_color(lf, lv_color_hex(0xFF4444), 0);
    lv_obj_set_style_text_font(lf, &lv_font_montserrat_12, 0);
    lv_obj_align(lf, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(btn_confirm, reset_dlg_confirm_cb, LV_EVENT_RELEASED, NULL);

    UI_UNLOCK();
}