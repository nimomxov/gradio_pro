/**
 * @file bluetooth_sender.c
 * @brief HC-05 Classic BT sender ? dedicated task, production-grade.
 *
 * ARCHAEOLOGY MISSION CRITICAL:
 *   Classic BT (HC-05 via UART) is the PRIMARY channel feeding OKM Visualizer3D.
 *   Every scan point MUST reach the PC without loss or corruption.
 *   BLE is a secondary monitoring channel ? can be disabled by user.
 *
 * FIXES APPLIED (v2.1):
 *  1. bt_sender_deinit: safe shutdown ? notifies task via stop_flag,
 *     waits for task to self-delete before destroying queue/mutex.
 *     Prevents use-after-free crash when deinit called during active scan.
 *  2. bt_sender_init: uart_installed flag ? uart_driver_delete only called
 *     if install succeeded, preventing driver corruption on partial init.
 *  3. Timestamp in bt_enqueue_live: uses esp_timer (overflow-safe).
 *  4. UART TX buffer check before write ? prevents HC-05 disconnect freeze.
 *  5. Scan messages use xQueueSendToFront with retry ? scan points are
 *     archaeology-critical and must not be silently dropped.
 *  6. bt_sender_task: explicit vTaskDelete(NULL) on stop instead of while(1).
 */

#include "bluetooth_sender.h"
#include "system_monitor/system_monitor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "BTSender";

/* =========================================================================
 * SAFE TIMESTAMP ? same helper used in queue_manager
 * ========================================================================= */

static inline uint32_t get_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000LL);
}

/* =========================================================================
 * SENDER TASK ? Core 0, dedicated to HC-05 UART transmission
 *
 * DESIGN: task polls send_queue every 100ms.
 *   - SCAN messages: highest priority ? archaeology data, must not block.
 *   - LIVE messages: throttled by producer, best-effort display only.
 *
 * STOP MECHANISM: s->stop_flag set by bt_sender_deinit ? task self-deletes
 * cleanly ? deinit then safely destroys queue and mutex.
 * ========================================================================= */

/* =========================================================================
 * VIS3D SMART PRECISION FORMATTER
 *
 * Sends float if noise_rms < 1.5 LSB (sub-LSB info is real).
 * Falls back to integer if noise is too high (decimals = fake precision).
 *
 * noise_rms < 1.5 LSB:  "512.43\r\n"  ? Kalman sub-LSB is meaningful
 * noise_rms >= 1.5 LSB: "512\r\n"     ? integers only, decimals = noise
 *
 * Uses fixed stack buffer only ? no heap allocation.
 * ========================================================================= */
static int bt_format_value(char *buf, size_t bufsz, const BtMsg_t *msg)
{
    if (msg->use_float) {
        /* Smart precision: only send decimals if noise is low enough */
        if (msg->noise_rms < 1.5f) {
            return snprintf(buf, bufsz, "%.2f\r\n", (double)msg->vis3d_value);
        } else {
            return snprintf(buf, bufsz, "%.0f\r\n", (double)msg->vis3d_value);
        }
    }
    /* Legacy integer modes */
    return snprintf(buf, bufsz, "%u\r\n", (unsigned)msg->value);
}

static void bt_sender_task(void *arg)
{
    BTSender_t *s = (BTSender_t *)arg;
    BtMsg_t     msg;
    char        buf[16];   /* max: "1024.00\r\n" = 10 chars + safety */
    uint32_t    rx_poll_ms = 0;

    ESP_LOGI(TAG, "bt_sender_task started Core%d Prio%d",
             (int)xPortGetCoreID(), (int)uxTaskPriorityGet(NULL));

    while (!s->stop_flag) {
        if (xQueueReceive(s->send_queue, &msg, pdMS_TO_TICKS(50))) {

            int len = bt_format_value(buf, sizeof(buf), &msg);
            if (len < 0 || len >= (int)sizeof(buf)) {
                s->uart_errors++;
                continue;
            }

            /* Check TX buffer space before writing.
             * If HC-05 is physically disconnected, UART TX FIFO fills up.
             * uart_write_bytes would block indefinitely without this guard.
             * This is the primary protection against freeze during field use. */
            size_t free_tx = 0;
            int    written = 0;

            if (uart_get_tx_buffer_free_size(s->uart_num, &free_tx) == ESP_OK
                && free_tx >= (size_t)len) {
                written = uart_write_bytes(s->uart_num, buf, (size_t)len);
            } else {
                /* TX full ? HC-05 likely disconnected or not powered */
                if (msg.type == BT_MSG_SCAN) {
                    s->uart_errors++;
                    ESP_LOGW(TAG, "TX buffer full ? scan point lost (HC-05 disconnected?)");
                }
                /* LIVE drops are silent ? display-only, not archaeology-critical */
                continue;
            }

            if (written == len) {
                if (msg.type == BT_MSG_LIVE) s->sent_live++;
                else                         s->sent_scan++;

                if (s->conn_state == BT_STATE_OFF) {
                    s->conn_state = BT_STATE_CONNECTING;
                }
            } else {
                s->uart_errors++;
                if (msg.type == BT_MSG_SCAN) {
                    ESP_LOGE(TAG, "Scan point partial write %d/%d ? OKM data integrity risk!", written, len);
                } else {
                    ESP_LOGW(TAG, "UART partial write %d/%d", written, len);
                }
            }
        }

        /* Poll RX every ~100ms to detect PC visualizer connection */
        uint32_t now_ms = get_ms();
        if ((now_ms - rx_poll_ms) >= 100u) {
            rx_poll_ms = now_ms;

            uint8_t rx_buf[16];
            int rx_len = uart_read_bytes(s->uart_num, rx_buf, sizeof(rx_buf), 0);
            if (rx_len > 0) {
                s->last_rx_ms = now_ms;
                if (s->conn_state != BT_STATE_CONNECTED) {
                    s->conn_state = BT_STATE_CONNECTED;
                    ESP_LOGI(TAG, "PC CONNECTED ? OKM Visualizer3D responding");
                }
            }

            /* Watchdog: if no RX for BT_RECONNECT_MS, mark as disconnected */
            if (s->conn_state == BT_STATE_CONNECTED &&
                s->last_rx_ms > 0 &&
                (now_ms - s->last_rx_ms) > BT_RECONNECT_MS) {
                s->conn_state = BT_STATE_CONNECTING;
                ESP_LOGW(TAG, "PC disconnected (RX timeout %lums)", (unsigned long)BT_RECONNECT_MS);
            }
        }
    }

    /* Stop flag set ? clean self-delete */
    ESP_LOGI(TAG, "bt_sender_task stopping");
    s->task_handle = NULL;   /* Signal to deinit that we have exited */
    vTaskDelete(NULL);
}

/* =========================================================================
 * INIT
 * ========================================================================= */

esp_err_t bt_sender_init(BTSender_t *s)
{
    memset(s, 0, sizeof(BTSender_t));
    s->uart_num   = BT_UART_NUM;
    s->conn_state = BT_STATE_OFF;
    s->stop_flag  = false;

    bool uart_installed = false;  /* FIX: track install state for safe fail cleanup */

    const uart_config_t cfg = {
        .baud_rate           = BT_BAUD_RATE,
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_DISABLE,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk          = UART_SCLK_APB,
    };

    esp_err_t ret = uart_param_config(s->uart_num, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(ret));
        goto fail;
    }

    ret = uart_set_pin(s->uart_num, BT_UART_TX_PIN, BT_UART_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(ret));
        goto fail;
    }

    ret = uart_driver_install(s->uart_num, BT_UART_RX_BUF, BT_UART_TX_BUF, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(ret));
        goto fail;
    }
    uart_installed = true;  /* mark so fail path knows to delete */

    s->send_queue = xQueueCreate(BT_QUEUE_DEPTH, sizeof(BtMsg_t));
    if (!s->send_queue) {
        ESP_LOGE(TAG, "Queue create failed");
        ret = ESP_ERR_NO_MEM;
        goto fail;
    }

    s->session_mutex = xSemaphoreCreateMutex();
    if (!s->session_mutex) {
        ESP_LOGE(TAG, "Session mutex create failed");
        ret = ESP_ERR_NO_MEM;
        goto fail;
    }

    BaseType_t rc = xTaskCreatePinnedToCore(
        bt_sender_task, "bt_sender", BT_TASK_STACK, s,
        BT_TASK_PRIORITY, &s->task_handle, BT_TASK_CORE
    );
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "Task create failed");
        ret = ESP_FAIL;
        goto fail;
    }

    s->initialized = true;
    s->conn_state  = BT_STATE_CONNECTING;

    ESP_LOGI(TAG, "Ready ? UART%d @ %d baud | TX:GPIO%d RX:GPIO%d | Queue:%u",
             (int)s->uart_num, BT_BAUD_RATE, BT_UART_TX_PIN, BT_UART_RX_PIN, BT_QUEUE_DEPTH);
    return ESP_OK;

fail:
    if (s->send_queue)    { vQueueDelete(s->send_queue);    s->send_queue    = NULL; }
    if (s->session_mutex) { vSemaphoreDelete(s->session_mutex); s->session_mutex = NULL; }
    if (uart_installed)   { uart_driver_delete(s->uart_num); }  /* FIX: only if installed */

    sysmon_report_fault(FAULT_BT_INIT_FAILED, "BT sender init failed");
    s->conn_state = BT_STATE_ERROR;
    return ret;
}

/* =========================================================================
 * DEINIT ? Safe shutdown, waits for task to exit before freeing resources.
 *
 * FIX: Old code called vTaskDelete(handle) then immediately vQueueDelete.
 * If the task was in xQueueReceive or uart_write_bytes, the queue/UART
 * structures got destroyed under its feet ? use-after-free / crash.
 *
 * New approach:
 *   1. Set stop_flag ? task exits its loop and calls vTaskDelete(NULL)
 *   2. Poll task_handle until task sets it NULL (signals self-delete done)
 *   3. Then safely destroy queue and mutex
 * ========================================================================= */

void bt_sender_deinit(BTSender_t *s)
{
    if (!s->initialized) return;

    /* Signal task to stop */
    s->stop_flag = true;

    /* Wait for task to exit cleanly ? poll up to 500ms */
    uint32_t wait_ms = 0;
    while (s->task_handle != NULL && wait_ms < 500u) {
        vTaskDelay(pdMS_TO_TICKS(10));
        wait_ms += 10u;
    }

    if (s->task_handle != NULL) {
        /* Task did not exit in time ? force delete as last resort */
        ESP_LOGW(TAG, "bt_sender_task did not stop cleanly ? force deleting");
        vTaskDelete(s->task_handle);
        s->task_handle = NULL;
        /* Small delay to allow scheduler to process task deletion */
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* Now safe to destroy resources */
    if (s->send_queue)    { vQueueDelete(s->send_queue);        s->send_queue    = NULL; }
    if (s->session_mutex) { vSemaphoreDelete(s->session_mutex); s->session_mutex = NULL; }
    uart_driver_delete(s->uart_num);

    s->initialized = false;
    s->stop_flag   = false;
    s->conn_state  = BT_STATE_OFF;

    ESP_LOGI(TAG, "BT sender deinitialized");
}

/* =========================================================================
 * ENQUEUE API
 * ========================================================================= */

void bt_enqueue_live(BTSender_t *s, uint16_t value)
{
    if (!s->initialized || !s->send_queue) return;

    uint32_t now = get_ms();
    if ((now - s->last_live_ms) < BT_LIVE_MIN_MS) return;
    s->last_live_ms = now;

    BtMsg_t msg = {
        .value       = value,
        .type        = BT_MSG_LIVE,
        .use_float   = false,
        .vis3d_value = 0.0f,
        .noise_rms   = 0.0f,
    };
    if (xQueueSend(s->send_queue, &msg, 0) != pdTRUE) {
        s->dropped_live++;
    }
}

/* =========================================================================
 * VIS3D_FLOAT ENQUEUE API
 * =========================================================================
 * Separate entry points for BT_MODE_VIS3D_FLOAT mode.
 * The float value and noise_rms are carried in BtMsg_t.use_float=true.
 * bt_sender_task calls bt_format_value() which applies smart precision.
 * ========================================================================= */

void bt_enqueue_vis3d_live(BTSender_t *s, float value, float noise_rms)
{
    if (!s->initialized || !s->send_queue) return;

    uint32_t now = get_ms();
    if ((now - s->last_live_ms) < BT_LIVE_MIN_MS) return;
    s->last_live_ms = now;

    /* Clamp to valid Visualizer3D range */
    if (value < 0.0f)    value = 0.0f;
    if (value > 1024.0f) value = 1024.0f;

    BtMsg_t msg = {
        .vis3d_value = value,
        .noise_rms   = noise_rms,
        .value       = (uint16_t)value,  /* integer fallback */
        .type        = BT_MSG_LIVE,
        .use_float   = true,
    };
    if (xQueueSend(s->send_queue, &msg, 0) != pdTRUE) {
        s->dropped_live++;
    }
}

bool bt_enqueue_vis3d_scan(BTSender_t *s, float value, float noise_rms)
{
    if (!s->initialized || !s->send_queue) return false;

    if (xSemaphoreTake(s->session_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGW(TAG, "VIS3D scan enqueue: mutex timeout");
        return false;
    }

    if (!s->session.active || s->session.pause) {
        xSemaphoreGive(s->session_mutex);
        return false;
    }

    /* Clamp */
    if (value < 0.0f)    value = 0.0f;
    if (value > 1024.0f) value = 1024.0f;

    BtMsg_t msg = {
        .vis3d_value = value,
        .noise_rms   = noise_rms,
        .value       = (uint16_t)value,
        .type        = BT_MSG_SCAN,
        .use_float   = true,
    };
    bool sent = false;
    if (xQueueSendToFront(s->send_queue, &msg, pdMS_TO_TICKS(5)) == pdTRUE) {
        sent = true;
    } else if (xQueueSend(s->send_queue, &msg, pdMS_TO_TICKS(15)) == pdTRUE) {
        sent = true;
    }

    if (sent) {
        s->session.current_step++;
        s->session.total_points++;
        bool line_done = (s->session.current_step >= s->session.total_steps);
        if (line_done) s->session.pause = true;
        xSemaphoreGive(s->session_mutex);
        return line_done;
    }

    xSemaphoreGive(s->session_mutex);
    ESP_LOGE(TAG, "VIS3D scan point DROPPED ? queue full!");
    s->uart_errors++;
    return false;
}

bool bt_enqueue_scan(BTSender_t *s, uint16_t value)
{
    if (!s->initialized || !s->send_queue) return false;

    /* Lock mutex for cross-core session state access */
    if (xSemaphoreTake(s->session_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGW(TAG, "Scan enqueue: mutex timeout");
        return false;
    }

    if (!s->session.active || s->session.pause) {
        xSemaphoreGive(s->session_mutex);
        return false;
    }

    BtMsg_t msg = {
        .value       = value,
        .type        = BT_MSG_SCAN,
        .use_float   = false,
        .vis3d_value = 0.0f,
        .noise_rms   = 0.0f,
    };
    bool sent = false;

    /* Scan points are archaeology-critical ? try front of queue first,
     * then retry with back enqueue. Two attempts before declaring failure. */
    if (xQueueSendToFront(s->send_queue, &msg, pdMS_TO_TICKS(5)) == pdTRUE) {
        sent = true;
    } else if (xQueueSend(s->send_queue, &msg, pdMS_TO_TICKS(15)) == pdTRUE) {
        sent = true;
    }

    if (sent) {
        s->session.current_step++;
        s->session.total_points++;

        /* Pause further enqueues when line is complete ? prevents overrun
         * until ui_event_task calls bt_new_line() to start next scan line */
        bool line_done = (s->session.current_step >= s->session.total_steps);
        if (line_done) {
            s->session.pause = true;
        }

        xSemaphoreGive(s->session_mutex);

        ESP_LOGD(TAG, "Scan point %u/%u ? bt_val=%u",
                 (unsigned)s->session.current_step,
                 (unsigned)s->session.total_steps,
                 (unsigned)value);
        return line_done;
    }

    xSemaphoreGive(s->session_mutex);
    ESP_LOGE(TAG, "CRITICAL: scan point DROPPED ? queue full, OKM data may be incomplete!");
    s->uart_errors++;
    return false;
}

/* =========================================================================
 * SESSION MANAGEMENT  (called from UI Task ? Core 1)
 * ========================================================================= */

void bt_session_start(BTSender_t *s, uint8_t steps_per_line)
{
    if (steps_per_line < 1)  steps_per_line = 1;
    if (steps_per_line > 50) steps_per_line = 50;

    xSemaphoreTake(s->session_mutex, portMAX_DELAY);
    memset(&s->session, 0, sizeof(ScanSession_t));
    s->session.active      = true;
    s->session.pause       = false;
    s->session.total_steps = steps_per_line;
    xSemaphoreGive(s->session_mutex);

    ESP_LOGI(TAG, "Session START ? %u steps/line", (unsigned)steps_per_line);
}

void bt_session_end(BTSender_t *s)
{
    xSemaphoreTake(s->session_mutex, portMAX_DELAY);
    ESP_LOGI(TAG, "Session END ? lines:%u  total_points:%lu",
             (unsigned)s->session.current_line,
             (unsigned long)s->session.total_points);
    memset(&s->session, 0, sizeof(ScanSession_t));
    xSemaphoreGive(s->session_mutex);
}

void bt_new_line(BTSender_t *s)
{
    xSemaphoreTake(s->session_mutex, portMAX_DELAY);
    s->session.current_line++;
    s->session.current_step = 0;
    s->session.pause        = false;   /* un-pause: next line can now receive points */
    xSemaphoreGive(s->session_mutex);

    ESP_LOGI(TAG, "New scan line %u", (unsigned)s->session.current_line);
}

const ScanSession_t *bt_get_session(const BTSender_t *s)
{
    /* UI reads this for display only ? torn read of total_points is acceptable */
    return &s->session;
}

BtConnectionState_t bt_get_conn_state(const BTSender_t *s)
{
    return s->conn_state;
}

bool bt_is_ready(const BTSender_t *s)
{
    return s->initialized;
}
