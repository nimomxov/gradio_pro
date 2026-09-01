/**
 * @file xpt2046_driver.c
 * @brief XPT2046 touch controller — polling mode, shared HSPI
 *
 * T_CS=25, T_IRQ=not connected → polling
 * Calibration handled by touch_calibration.c (Affine Transform)
 */

#include "xpt2046_driver.h"
#include "touch_calibration.h"
#include "ili9341_driver.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "XPT2046";

#define PIN_T_CS        25
#define TOUCH_SPI_HZ    (2 * 1000 * 1000)

/* Raw value range — used for "is touched" detection only */
#define RAW_MIN         100
#define RAW_MAX         3950

/* Oversampling for LVGL read (fast, single sample) */
#define FAST_SAMPLES    4

static spi_device_handle_t s_touch_spi = NULL;

/* ── Low-level SPI read ── */
static uint16_t spi_read_cmd(uint8_t cmd)
{
    uint8_t tx[3] = {cmd, 0, 0};
    uint8_t rx[3] = {0};
    spi_transaction_t t = {
        .length    = 24,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_polling_transmit(s_touch_spi, &t);
    return ((uint16_t)(rx[1] << 8) | rx[2]) >> 3;
}

void xpt2046_init(void)
{
    spi_device_interface_config_t dev = {
        .clock_speed_hz = TOUCH_SPI_HZ,
        .mode           = 0,
        .spics_io_num   = PIN_T_CS,
        .queue_size     = 3,
        .flags          = 0,
    };
    spi_bus_add_device(SPI2_HOST, &dev, &s_touch_spi);
    ESP_LOGI(TAG, "XPT2046 init done (T_CS=GPIO%d)", PIN_T_CS);
}

/* ── Raw read for calibration (no transform applied) ── */
void xpt2046_read_raw_xy(int32_t *out_x, int32_t *out_y)
{
    /* XPT2046: 0xD0 = X, 0x90 = Y */
    *out_x = (int32_t)spi_read_cmd(0xD0);
    *out_y = (int32_t)spi_read_cmd(0x90);
}

/* ── Is screen currently touched (raw range check) ── */
bool xpt2046_is_touched(void)
{
    int32_t rx, ry;
    xpt2046_read_raw_xy(&rx, &ry);
    return (rx > RAW_MIN && rx < RAW_MAX &&
            ry > RAW_MIN && ry < RAW_MAX);
}

/* ── LVGL input device callback ── */
void xpt2046_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    (void)drv;

    /* Fast oversampled read */
    int32_t sum_x = 0, sum_y = 0;
    for (int i = 0; i < FAST_SAMPLES; i++) {
        sum_x += spi_read_cmd(0xD0);
        sum_y += spi_read_cmd(0x90);
    }
    int32_t raw_x = sum_x / FAST_SAMPLES;
    int32_t raw_y = sum_y / FAST_SAMPLES;

    bool touched = (raw_x > RAW_MIN && raw_x < RAW_MAX &&
                    raw_y > RAW_MIN && raw_y < RAW_MAX);

    if (touched) {
        int32_t sx, sy;
        touch_cal_apply(raw_x, raw_y, &sx, &sy);
        data->point.x = (lv_coord_t)sx;
        data->point.y = (lv_coord_t)sy;
        data->state   = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}
