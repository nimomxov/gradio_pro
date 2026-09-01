/**
 * @file ble_data_logger.c
 * @brief ESP32 BLE GATT Server — CSV scan data → mobile phone
 *
 * يعمل بالتوازي مع HC-05:
 *   HC-05 (UART) → OKM Visualizer3D (binary protocol)   [لا يتغير]
 *   BLE داخلي   → هاتف Android/iOS (CSV notifications)  [هذا الملف]
 *
 * المنطق:
 *   1. BLE stack يعمل دائماً في الخلفية (advertising)
 *   2. عند تفعيل data_via_ble switch → ble_logger_set_enabled(true)
 *   3. عند كل نقطة مسح → ui_event_task تستدعي ble_logger_enqueue()
 *   4. ble_logger_task يُرسل CSV عبر NOTIFY إذا كان client متصلاً
 */

#include "ble_data_logger.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* ═══════════════════════════════════════════════════════════════════
 * CONSTANTS
 * ═══════════════════════════════════════════════════════════════════ */

static const char *TAG = "BLE_Logger";

#define BLE_DEVICE_NAME        "GS El-Mersad"
#define BLE_QUEUE_DEPTH        8        /* max buffered scan points — reduced for DRAM */
#define BLE_TASK_STACK         4096
#define BLE_TASK_CORE          1
#define BLE_TASK_PRIO          3
#define BLE_MIN_INTERVAL_MS    50       /* max 20 points/sec           */
#define BLE_MTU_MAX            512
#define BLE_CSV_MAX_LEN        220      /* worst-case CSV line length  */

/* GATT handles */
#define GATTS_APP_ID           0
#define GATTS_NUM_HANDLES      6

/* UUIDs — 128-bit little-endian */
/* Service : 4AFAFADE-1234-11EF-8A1C-2B7E8F6A9C3E */
static uint8_t SERVICE_UUID[16] = {
    0x3E, 0x9C, 0x6A, 0x8F, 0x7E, 0x2B, 0x1C, 0x8A,
    0xEF, 0x11, 0x34, 0x12, 0xDE, 0xFA, 0xAF, 0x4A
};
/* Characteristic : 4AFAFADE-1235-11EF-8A1C-2B7E8F6A9C3E */
static uint8_t CHAR_UUID[16] = {
    0x3E, 0x9C, 0x6A, 0x8F, 0x7E, 0x2B, 0x1C, 0x8A,
    0xEF, 0x11, 0x35, 0x12, 0xDE, 0xFA, 0xAF, 0x4A
};

/* ═══════════════════════════════════════════════════════════════════
 * STATE
 * ═══════════════════════════════════════════════════════════════════ */

static struct {
    /* BLE connection state */
    bool     connected;
    bool     notify_enabled;   /* client wrote 0x0001 to CCCD */
    uint16_t conn_id;
    uint16_t gatts_if;

    /* GATT handles */
    uint16_t service_handle;
    uint16_t char_handle;
    uint16_t cccd_handle;

    /* App state */
    bool     enabled;          /* controlled by data_via_ble switch */
    bool     initialized;

    /* Queue */
    QueueHandle_t queue;
} s = {0};

/* ═══════════════════════════════════════════════════════════════════
 * ADVERTISEMENT CONFIG
 * ═══════════════════════════════════════════════════════════════════ */

static esp_ble_adv_params_t s_adv_params = {
    .adv_int_min       = 0x20,   /* 20ms */
    .adv_int_max       = 0x40,   /* 40ms */
    .adv_type          = ADV_TYPE_IND,
    .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .channel_map       = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_adv_data_t s_adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = false,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0,
    .p_manufacturer_data = NULL,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = 0,
    .p_service_uuid      = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

/* ═══════════════════════════════════════════════════════════════════
 * CSV FORMATTER
 * ═══════════════════════════════════════════════════════════════════ */

static const char *quality_note(float snr)
{
    if (snr < 2.0f) return "noise";
    if (snr < 3.0f) return "weak";
    if (snr < 5.0f) return "moderate";
    return "strong";
}

static int format_csv(const BleLogPoint_t *pt, char *buf, size_t buf_len)
{
    return snprintf(buf, buf_len,
        "%lu,%s,%u,%d,%.1f,%.1f,%.1f,%u,%.1f,%.1f,%u,%u,%u,%u,%u,%.2f,%s\n",
        (unsigned long)pt->timestamp_ms,
        pt->scan_mode == 0 ? "manual" : "auto",
        (unsigned)pt->step,
        (int)pt->gradient_raw,
        pt->gradient_filtered,
        pt->baseline,
        pt->deviation,
        (unsigned)pt->bt_value,
        pt->snr,
        pt->stability,
        (unsigned)pt->soil_type,
        (unsigned)pt->sensitivity,
        (unsigned)pt->boost,
        (unsigned)pt->kalman,
        (unsigned)pt->heading,
        pt->noise_floor,
        quality_note(pt->snr)
    );
}

/* ═══════════════════════════════════════════════════════════════════
 * GAP EVENT HANDLER
 * ═══════════════════════════════════════════════════════════════════ */

static void gap_event_handler(esp_gap_ble_cb_event_t event,
                               esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&s_adv_params);
            break;

        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS)
                ESP_LOGI(TAG, "Advertising started — \"%s\"", BLE_DEVICE_NAME);
            else
                ESP_LOGE(TAG, "Advertising start failed");
            break;

        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            ESP_LOGI(TAG, "Advertising stopped");
            break;

        default:
            break;
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * GATTS EVENT HANDLER
 * ═══════════════════════════════════════════════════════════════════ */

static void gatts_event_handler(esp_gatts_cb_event_t event,
                                 esp_gatt_if_t gatts_if,
                                 esp_ble_gatts_cb_param_t *param)
{
    switch (event) {

        /* ── App registered ── */
        case ESP_GATTS_REG_EVT: {
            if (param->reg.status != ESP_GATT_OK) {
                ESP_LOGE(TAG, "GATTS reg failed: %d", param->reg.app_id);
                break;
            }
            s.gatts_if = gatts_if;
            ESP_LOGI(TAG, "GATTS registered if=%d", gatts_if);

            /* Set device name */
            esp_ble_gap_set_device_name(BLE_DEVICE_NAME);
            esp_ble_gap_config_adv_data(&s_adv_data);

            /* Create service */
            esp_gatt_srvc_id_t service_id = {
                .is_primary = true,
                .id = {
                    .inst_id = 0,
                    .uuid = {
                        .len = ESP_UUID_LEN_128,
                    }
                }
            };
            memcpy(service_id.id.uuid.uuid.uuid128, SERVICE_UUID, 16);
            esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLES);
            break;
        }

        /* ── Service created ── */
        case ESP_GATTS_CREATE_EVT: {
            s.service_handle = param->create.service_handle;
            ESP_LOGI(TAG, "Service created handle=%d", s.service_handle);
            esp_ble_gatts_start_service(s.service_handle);

            /* Add characteristic: READ + NOTIFY, no auto-response */
            esp_bt_uuid_t char_uuid = {
                .len = ESP_UUID_LEN_128,
            };
            memcpy(char_uuid.uuid.uuid128, CHAR_UUID, 16);

            esp_gatt_char_prop_t prop = ESP_GATT_CHAR_PROP_BIT_READ |
                                        ESP_GATT_CHAR_PROP_BIT_NOTIFY;

            static uint8_t char_val[1] = {0};
            esp_attr_value_t attr = {
                .attr_max_len = BLE_CSV_MAX_LEN,
                .attr_len     = sizeof(char_val),
                .attr_value   = char_val,
            };
            esp_ble_gatts_add_char(s.service_handle, &char_uuid,
                                   ESP_GATT_PERM_READ,
                                   prop, &attr, NULL);
            break;
        }

        /* ── Characteristic added — now add CCCD ── */
        case ESP_GATTS_ADD_CHAR_EVT: {
            if (param->add_char.status != ESP_GATT_OK) {
                ESP_LOGE(TAG, "Add char failed");
                break;
            }
            s.char_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "Char added handle=%d", s.char_handle);

            /* CCCD descriptor */
            esp_bt_uuid_t cccd_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = { .uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG }
            };
            static uint8_t cccd_val[2] = {0x00, 0x00};
            esp_attr_value_t cccd_attr = {
                .attr_max_len = 2,
                .attr_len     = 2,
                .attr_value   = cccd_val,
            };
            esp_ble_gatts_add_char_descr(s.service_handle, &cccd_uuid,
                                         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                         &cccd_attr, NULL);
            break;
        }

        /* ── CCCD added ── */
        case ESP_GATTS_ADD_CHAR_DESCR_EVT: {
            s.cccd_handle = param->add_char_descr.attr_handle;
            ESP_LOGI(TAG, "CCCD added handle=%d", s.cccd_handle);
            break;
        }

        /* ── Client connected ── */
        case ESP_GATTS_CONNECT_EVT: {
            s.connected      = true;
            s.notify_enabled = false;
            s.conn_id        = param->connect.conn_id;
            ESP_LOGI(TAG, "BLE client connected conn_id=%d", s.conn_id);
            /* Stop advertising while connected */
            esp_ble_gap_stop_advertising();
            break;
        }

        /* ── Client disconnected ── */
        case ESP_GATTS_DISCONNECT_EVT: {
            s.connected      = false;
            s.notify_enabled = false;
            ESP_LOGI(TAG, "BLE client disconnected — restarting advertising");
            esp_ble_gap_start_advertising(&s_adv_params);
            break;
        }

        /* ── Client wrote to CCCD (enable/disable notifications) ── */
        case ESP_GATTS_WRITE_EVT: {
            if (!param->write.is_prep &&
                param->write.handle == s.cccd_handle &&
                param->write.len == 2) {
                uint16_t cccd_val = param->write.value[0] |
                                    (param->write.value[1] << 8);
                s.notify_enabled = (cccd_val == 0x0001);
                ESP_LOGI(TAG, "Notifications %s",
                         s.notify_enabled ? "ENABLED" : "DISABLED");
            }
            /* Send response */
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                            param->write.trans_id,
                                            ESP_GATT_OK, NULL);
            }
            break;
        }

        /* ── MTU changed ── */
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(TAG, "MTU changed to %d", param->mtu.mtu);
            break;

        default:
            break;
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * BLE LOGGER TASK
 * ═══════════════════════════════════════════════════════════════════ */

static void ble_logger_task(void *arg)
{
    (void)arg;
    BleLogPoint_t pt;
    char csv[BLE_CSV_MAX_LEN];
    TickType_t last_send = 0;

    ESP_LOGI(TAG, "ble_logger_task started Core%d Prio%d",
             xPortGetCoreID(), uxTaskPriorityGet(NULL));

    for (;;) {
        /* انتظر نقطة مسح من queue (بدون timeout — task نائمة حتى تصل بيانات) */
        if (xQueueReceive(s.queue, &pt, portMAX_DELAY) != pdTRUE) continue;

        /* تحقق من شروط الإرسال */
        if (!s.enabled)          continue;   /* switch OFF          */
        if (!s.connected)        continue;   /* لا يوجد client      */
        if (!s.notify_enabled)   continue;   /* client لم يشترك     */

        /* Rate limiting — max 20 points/sec */
        TickType_t now = xTaskGetTickCount();
        TickType_t elapsed = (now - last_send) * portTICK_PERIOD_MS;
        if (elapsed < BLE_MIN_INTERVAL_MS) {
            vTaskDelay(pdMS_TO_TICKS(BLE_MIN_INTERVAL_MS - elapsed));
        }

        /* تنسيق CSV */
        int len = format_csv(&pt, csv, sizeof(csv));
        if (len <= 0 || len >= (int)sizeof(csv)) {
            ESP_LOGW(TAG, "CSV format error len=%d", len);
            continue;
        }

        /* إرسال عبر NOTIFY */
        esp_err_t err = esp_ble_gatts_send_indicate(
            s.gatts_if,
            s.conn_id,
            s.char_handle,
            (uint16_t)len,
            (uint8_t *)csv,
            false   /* false = notify (لا يحتاج ACK) */
        );

        if (err == ESP_OK) {
            last_send = xTaskGetTickCount();
            ESP_LOGD(TAG, "BLE TX [%u bytes]: %.40s...", (unsigned)len, csv);
        } else {
            ESP_LOGW(TAG, "BLE send failed: %s", esp_err_to_name(err));
        }
    }

    vTaskDelete(NULL);
}

/* ═══════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ═══════════════════════════════════════════════════════════════════ */

esp_err_t ble_logger_init(void)
{
    if (s.initialized) return ESP_OK;

    /* إنشاء queue */
    s.queue = xQueueCreate(BLE_QUEUE_DEPTH, sizeof(BleLogPoint_t));
    if (!s.queue) {
        ESP_LOGE(TAG, "Queue creation failed");
        return ESP_ERR_NO_MEM;
    }

    /* تهيئة BT controller للـ BLE فقط */
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_bt_controller_init(&bt_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_bluedroid_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_bluedroid_enable();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(err));
        return err;
    }

    /* تسجيل callbacks */
    err = esp_ble_gatts_register_callback(gatts_event_handler);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GATTS callback register failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_ble_gap_register_callback(gap_event_handler);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GAP callback register failed: %s", esp_err_to_name(err));
        return err;
    }

    /* تسجيل GATT app */
    err = esp_ble_gatts_app_register(GATTS_APP_ID);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GATTS app register failed: %s", esp_err_to_name(err));
        return err;
    }

    s.initialized = true;
    ESP_LOGI(TAG, "BLE Logger initialized — advertising as \"%s\"", BLE_DEVICE_NAME);
    return ESP_OK;
}

void ble_logger_task_start(void)
{
    xTaskCreatePinnedToCore(
        ble_logger_task,
        "ble_logger",
        BLE_TASK_STACK,
        NULL,
        BLE_TASK_PRIO,
        NULL,
        BLE_TASK_CORE
    );
    ESP_LOGI(TAG, "ble_logger_task created Core%d Prio%d",
             BLE_TASK_CORE, BLE_TASK_PRIO);
}

void ble_logger_set_enabled(bool enabled)
{
    s.enabled = enabled;
    ESP_LOGI(TAG, "BLE logger %s", enabled ? "ENABLED" : "DISABLED");
}

void ble_logger_enqueue(const BleLogPoint_t *pt)
{
    if (!s.enabled || !pt) return;
    if (!s.queue) return;

    /* Non-blocking — drop if queue full (لا نحجب ui_event_task) */
    if (xQueueSend(s.queue, pt, 0) != pdTRUE) {
        ESP_LOGD(TAG, "BLE queue full — point dropped");
    }
}

bool ble_logger_is_connected(void)
{
    return s.connected && s.notify_enabled;
}

bool ble_logger_is_enabled(void)
{
    return s.enabled;
}
