/**
 * @file hardware_config.h
 * @brief Single source of truth for ALL hardware pin assignments.
 *
 * ═══════════════════════════════════════════════════════════════════
 *  GRADIOMETER PRO v2.0 — PIN MAP
 * ═══════════════════════════════════════════════════════════════════
 *
 *  GPIO 1  → UART0 TX  (HC-05 BT, shares with USB-Serial in flash mode)
 *  GPIO 2  → TFT_DC    (ILI9341 Data/Command)
 *  GPIO 3  → UART0 RX  (HC-05 BT, shares with USB-Serial in flash mode)
 *  GPIO 4  → TFT_RST   (ILI9341 Reset, active LOW)
 *  GPIO 5  → TFT_CS    (ILI9341 Chip Select, active LOW)
 *  GPIO 16 → SCAN_BTN  (Physical scan button, active LOW, internal pull-up)
 *  GPIO 18 → SPI_CLK   (Shared: TFT_CLK + T_CLK)
 *  GPIO 19 → SPI_MISO  (Shared: TFT_MISO + T_DO)
 *  GPIO 22 → I2C_SCL   (ADS1115)
 *  GPIO 23 → SPI_MOSI  (Shared: TFT_MOSI + T_DIN)
 *  GPIO 25 → T_CS      (XPT2046 Touch CS, active LOW)
 *  GPIO 26 → BUZZER    (Passive buzzer, PWM via LEDC)
 *  GPIO 27 → I2C_SDA   (ADS1115)
 *
 *  T_IRQ   → NOT CONNECTED (polling mode used instead)
 *
 * ═══════════════════════════════════════════════════════════════════
 *  SPI BUS NOTE:
 *   ILI9341 and XPT2046 share HSPI (SPI2):
 *    CLK=18, MOSI=23, MISO=19
 *   Each device has its own CS pin (TFT_CS=5, T_CS=25)
 *   Max SPI clock: ILI9341=40MHz write / 6.7MHz read
 *                  XPT2046=2MHz (slower, be careful in sdkconfig)
 *
 *  I2C BUS NOTE:
 *   ADS1115 on I2C_NUM_0, Fast-mode 400kHz
 *   ADDR pin → GND → I2C address = 0x48
 *   AIN0 vs AIN1 differential mode (FLC100 dual sensor)
 *
 *  BLUETOOTH NOTE:
 *   HC-05 on UART_NUM_1
 *   GPIO1 (TX) and GPIO3 (RX) are also UART0 (USB-Serial)
 *   → Disconnect HC-05 during firmware flashing
 *   → After boot, UART0 is re-configured for HC-05
 *
 * ═══════════════════════════════════════════════════════════════════
 */

#pragma once

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"

/* ─── Display: ILI9341 2.4" TFT (HSPI) ──────────────────────────── */
#define HW_TFT_SPI_HOST      HSPI_HOST
#define HW_TFT_CLK_PIN       18
#define HW_TFT_MOSI_PIN      23
#define HW_TFT_MISO_PIN      19
#define HW_TFT_CS_PIN         5
#define HW_TFT_DC_PIN         2
#define HW_TFT_RST_PIN        4
#define HW_TFT_WIDTH        320
#define HW_TFT_HEIGHT       240
#define HW_TFT_SPI_CLK_HZ  (40 * 1000 * 1000)   /* 40 MHz write clock */

/* ─── Touch: XPT2046 (shared SPI bus) ───────────────────────────── */
#define HW_TOUCH_CS_PIN      25
/* T_IRQ not connected → polling mode */
#define HW_TOUCH_IRQ_PIN     (-1)   /* Sentinel: not used */
#define HW_TOUCH_SPI_CLK_HZ  (2 * 1000 * 1000)   /* 2 MHz max for XPT2046 */

/*
 * TOUCH CALIBRATION VALUES (XPT2046 → screen pixel mapping)
 *
 * How to recalibrate:
 *  1. Enable CALIBRATION_MODE in sdkconfig or define below
 *  2. Touch the 3 calibration points shown on screen
 *  3. Read raw X/Y values from serial log
 *  4. Compute:
 *       x_scale = (screen_x2 - screen_x1) / (raw_x2 - raw_x1)
 *       x_offset = screen_x1 - raw_x1 * x_scale
 *  5. Update values below
 *
 * Current values measured for ILI9341 2.4" with XPT2046 at 3.3V:
 */
#define HW_TOUCH_X_MIN       200    /* Raw ADC value at left edge   */
#define HW_TOUCH_X_MAX      3800    /* Raw ADC value at right edge  */
#define HW_TOUCH_Y_MIN       200    /* Raw ADC value at top edge    */
#define HW_TOUCH_Y_MAX      3800    /* Raw ADC value at bottom edge */
#define HW_TOUCH_X_INV         0    /* 1 = invert X axis            */
#define HW_TOUCH_Y_INV         0    /* 1 = invert Y axis            */
#define HW_TOUCH_XY_SWAP       0    /* 1 = swap X and Y axes        */

/* ─── ADS1115 (I2C_NUM_0) ────────────────────────────────────────── */
#define HW_I2C_PORT          I2C_NUM_0
#define HW_I2C_SDA_PIN       27
#define HW_I2C_SCL_PIN       22
#define HW_I2C_CLK_HZ        400000   /* Fast-mode 400kHz */
#define HW_ADS1115_ADDR      0x48     /* ADDR → GND */

/* ─── Scan Button ────────────────────────────────────────────────── */
#define HW_BTN_SCAN_GPIO     GPIO_NUM_16
/* Active LOW, internal pull-up enabled, NEGEDGE interrupt */

/* ─── Buzzer (passive, PWM) ──────────────────────────────────────── */
#define HW_BUZZER_GPIO       GPIO_NUM_26
#define HW_BUZZER_LEDC_CHAN  LEDC_CHANNEL_0
#define HW_BUZZER_LEDC_TIMER LEDC_TIMER_0
#define HW_BUZZER_FREQ_HZ    2000    /* Default beep frequency */
#define HW_BUZZER_DUTY       4096    /* 50% duty on 13-bit timer */

/* ─── Bluetooth HC-05 (UART_NUM_1) ───────────────────────────────── */
#define HW_BT_UART_NUM       UART_NUM_1
#define HW_BT_TX_PIN          1      /* ESP32 TX → HC-05 RX */
#define HW_BT_RX_PIN          3      /* ESP32 RX → HC-05 TX */
#define HW_BT_BAUD_RATE       9600   /* HC-05 default baud   */
