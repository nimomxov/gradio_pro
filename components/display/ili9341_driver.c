/**
 * @file ili9341_driver.c
 * @brief ILI9341 2.4" TFT SPI driver for LVGL v8 / ESP-IDF v5
 *
 * HSPI: CLK=18, MOSI=23, MISO=19
 * CS=5, DC=2, RST=4
 * Speed: 40MHz write
 */

#include "ili9341_driver.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "ILI9341";

#define LCD_HOST     SPI2_HOST   /* HSPI */
#define PIN_CLK      18
#define PIN_MOSI     23
#define PIN_MISO     19
#define PIN_CS        5
#define PIN_DC        2
#define PIN_RST       4
#define SPI_CLK_HZ   (40 * 1000 * 1000)

#define LCD_W        320
#define LCD_H        240

static spi_device_handle_t s_spi = NULL;

/* ── Low-level ── */

static void lcd_cmd(uint8_t cmd)
{
    gpio_set_level(PIN_DC, 0);
    spi_transaction_t t = { .length = 8, .tx_buffer = &cmd };
    spi_device_polling_transmit(s_spi, &t);
}

static void lcd_data(const uint8_t *data, size_t len)
{
    if (!len) return;
    gpio_set_level(PIN_DC, 1);
    spi_transaction_t t = { .length = len * 8, .tx_buffer = data };
    spi_device_polling_transmit(s_spi, &t);
}

static void lcd_byte(uint8_t b)  { lcd_data(&b, 1); }
static void lcd_word(uint16_t w) { uint8_t b[2] = {w>>8, w&0xFF}; lcd_data(b, 2); }

/* ── Init sequence ── */

static const uint8_t s_init_cmds[] = {
    0xEF, 3, 0x03, 0x80, 0x02,
    0xCF, 3, 0x00, 0xC1, 0x30,
    0xED, 4, 0x64, 0x03, 0x12, 0x81,
    0xE8, 3, 0x85, 0x00, 0x78,
    0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
    0xF7, 1, 0x20,
    0xEA, 2, 0x00, 0x00,
    0xC0, 1, 0x23,          /* Power control */
    0xC1, 1, 0x10,          /* Power control */
    0xC5, 2, 0x3e, 0x28,   /* VCOM control */
    0xC7, 1, 0x86,          /* VCOM control */
    0x36, 1, 0x48,          /* Memory Access Control: landscape */
    0x37, 1, 0x00,          /* Vertical scroll zero */
    0x3A, 1, 0x55,          /* Pixel format: RGB565 */
    0xB1, 2, 0x00, 0x18,   /* Frame rate */
    0xB6, 3, 0x08, 0x82, 0x27, /* Display function */
    0xF2, 1, 0x00,          /* Gamma disable */
    0x26, 1, 0x01,          /* Gamma curve */
    0xE0, 15, 0x0F,0x31,0x2B,0x0C,0x0E,0x08,0x4E,0xF1,0x37,0x07,0x10,0x03,0x0E,0x09,0x00,
    0xE1, 15, 0x00,0x0E,0x14,0x03,0x11,0x07,0x31,0xC1,0x48,0x08,0x0F,0x0C,0x31,0x36,0x0F,
    0x11, 0,  /* Sleep out */
    0x00,     /* Delay sentinel */
    0x29, 0,  /* Display on */
    0xFF,     /* End */
};

void ili9341_init(void)
{
    /* RST & DC as output */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_DC) | (1ULL << PIN_RST),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);

    /* SPI bus */
    spi_bus_config_t bus = {
        .miso_io_num   = PIN_MISO,
        .mosi_io_num   = PIN_MOSI,
        .sclk_io_num   = PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_W * 40 * 2 + 8,  /* 40 lines max DMA */
    };
    spi_bus_initialize(LCD_HOST, &bus, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t dev = {
        .clock_speed_hz = SPI_CLK_HZ,
        .mode           = 0,
        .spics_io_num   = PIN_CS,
        .queue_size     = 7,
        .flags          = SPI_DEVICE_NO_DUMMY,
    };
    spi_bus_add_device(LCD_HOST, &dev, &s_spi);

    /* Hardware reset */
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    /* Send init sequence */
    const uint8_t *p = s_init_cmds;
    while (*p != 0xFF) {
        if (*p == 0x00) { vTaskDelay(pdMS_TO_TICKS(150)); p++; continue; }
        uint8_t cmd  = *p++;
        uint8_t narg = *p++;
        lcd_cmd(cmd);
        for (int i = 0; i < narg; i++) lcd_byte(*p++);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "ILI9341 init done (%dx%d)", LCD_W, LCD_H);
}

/* ── LVGL flush callback ── */

void ili9341_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    /* Column address */
    lcd_cmd(0x2A);
    lcd_word(area->x1); lcd_word(area->x2);

    /* Row address */
    lcd_cmd(0x2B);
    lcd_word(area->y1); lcd_word(area->y2);

    /* Write pixels */
    lcd_cmd(0x2C);
    gpio_set_level(PIN_DC, 1);

    size_t px = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1);

    /* DMA transfer */
    spi_transaction_t t = {
        .length    = px * 2 * 8,
        .tx_buffer = color_map,
    };
    spi_device_polling_transmit(s_spi, &t);

    lv_disp_flush_ready(drv);
}

spi_device_handle_t ili9341_get_spi(void)
{
    return s_spi;
}
