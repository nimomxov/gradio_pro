#include "hardware_config.h"
/**
 * @file bluetooth_sender.h
 *
 * ARCHITECTURE ? ??????? ??????? ?? ????? ??????:
 *   uart_write_bytes() ???? ??????? ?????? ?? ui_event_task (Core 1)
 *   ????? signal_task (Core 0) ????? ?????? ? ?? ???? ????.
 *   ???????: ????? ??? ????? + ?????? race condition.
 *
 * ????: task ????? + queue ?????
 *
 *   [signal_task]  --bt_enqueue_live()--? [queue 32]
 *   [ui_3d_screen] --bt_enqueue_scan()--?    |  (scan ???? ??????)
 *                                             ?
 *                                    [bt_sender_task Core0 Prio4]
 *                                             |  uart_write() DMA
 *                                          HC-05 --? PC
 *
 * PROTOCOL: "<uint16>\r\n" ??? ? ???? "687\r\n"
 */

#pragma once
#include "gradiometer_types.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Pin assignments from hardware_config.h */
#define BT_UART_NUM     HW_BT_UART_NUM
#define BT_UART_TX_PIN  HW_BT_TX_PIN
#define BT_UART_RX_PIN  HW_BT_RX_PIN
#define BT_BAUD_RATE    HW_BT_BAUD_RATE
#define BT_UART_TX_BUF     2048
#define BT_UART_RX_BUF     256
#define BT_QUEUE_DEPTH     32
#define BT_LIVE_MIN_MS     50u      /* 20Hz max ??? live */
#define BT_RECONNECT_MS    3000u
#define BT_TASK_STACK      2048u
#define BT_TASK_PRIORITY   4u
#define BT_TASK_CORE       0

typedef enum {
    BT_MSG_LIVE = 0,  /* throttled, droppable */
    BT_MSG_SCAN = 1,  /* scan point ? not droppable */
} BtMsgType_t;

typedef struct {
    float       vis3d_value;   /* float value for BT_MODE_VIS3D_FLOAT */
    float       noise_rms;     /* for smart precision: float if <1.5, int otherwise */
    uint16_t    value;         /* integer value (legacy modes) */
    BtMsgType_t type;
    bool        use_float;     /* true when VIS3D_FLOAT mode */
} BtMsg_t;

typedef struct {
    bool     active;
	bool     pause;
    uint8_t  total_steps;
    uint8_t  current_step;
    uint8_t  current_line;
    uint32_t total_points;
} ScanSession_t;

typedef struct {
    uart_port_t         uart_num;
    bool                initialized;
    volatile bool       stop_flag;      /* set by deinit ? task exits cleanly */
    QueueHandle_t       send_queue;
    TaskHandle_t        task_handle;
    BtConnectionState_t conn_state;
    SemaphoreHandle_t   session_mutex;
    uint32_t            last_rx_ms;
    uint32_t            last_live_ms;
    ScanSession_t       session;
    uint32_t            sent_live;
    uint32_t            sent_scan;
    uint32_t            dropped_live;
    uint32_t            uart_errors;
} BTSender_t;

esp_err_t bt_sender_init(BTSender_t *s);
void      bt_sender_deinit(BTSender_t *s);

void bt_enqueue_live(BTSender_t *s, uint16_t value);
bool bt_enqueue_scan(BTSender_t *s, uint16_t value);

/**
 * @brief VIS3D_FLOAT mode enqueue ? sends float-precision value.
 *
 * Smart precision: if noise_rms < 1.5 LSB ? "512.43\r\n"
 *                  else                    ? "512\r\n"
 * Prevents fake precision in noisy soil while preserving sub-LSB
 * Kalman-filtered information in clean conditions.
 */
void bt_enqueue_vis3d_live(BTSender_t *s, float value, float noise_rms);
bool bt_enqueue_vis3d_scan(BTSender_t *s, float value, float noise_rms);

void                 bt_session_start(BTSender_t *s, uint8_t steps_per_line);
void                 bt_session_end(BTSender_t *s);
void                 bt_new_line(BTSender_t *s);
const ScanSession_t *bt_get_session(const BTSender_t *s);
BtConnectionState_t  bt_get_conn_state(const BTSender_t *s);
bool                 bt_is_ready(const BTSender_t *s);

#ifdef __cplusplus
}
#endif