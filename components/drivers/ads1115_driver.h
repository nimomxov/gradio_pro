/**
 * @file ads1115_driver.h
 * @brief ADS1115 16-bit ADC driver — continuous conversion, non-blocking.
 *
 * DESIGN DECISIONS:
 *  - Uses CONTINUOUS conversion mode (not single-shot).
 *    Single-shot required 8ms delay per read → at 4 samples averaged = 32ms.
 *    That exceeds our 20ms cycle time at 50Hz. Continuous mode eliminates
 *    this delay entirely — we just read the last converted value.
 *
 *  - ALERT/RDY pin NOT used (requires extra GPIO + IRQ setup).
 *    Instead: at 860SPS, conversion time = 1.16ms. We use 3ms delay in
 *    trimmed_mean reads — always fresh. In continuous mode (main loop),
 *    no delay needed at all.
 *
 *  - Default data rate: 860SPS (max).
 *    Reason: FLC100 gradiometer reads AIN0 and AIN1 sequentially via MUX.
 *    At 860SPS, the inter-channel time gap = 1.16ms.
 *    Earth's field changes < 1nT/s → 1.16ms gap is negligible.
 *    Higher SPS also improves oversampling ENOB in signal_processor.
 *
 *  - I2C retry on NACK: hardware can glitch. 3 retries before fault report.
 *
 *  - Differential measurement: AIN0(+) vs AIN1(-).
 *    This is the correct mode for FLC100 gradiometer coils.
 *    Common-mode interference cancels out — critical for field use.
 *
 *  - Gain set to PGA ±2.048V (ADS1115 full-scale per channel).
 *    FLC100 output typically ±1V differential. This gives headroom
 *    without clipping and maximizes resolution (62.5µV/LSB).
 */

#pragma once

#include "gradiometer_types.h"
#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * ADS1115 CONFIGURATION
 * ========================================================================= */

/** I2C address with ADDR pin → GND */
#define ADS1115_I2C_ADDR        0x48

/**
 * @brief Data rate options (samples per second).
 * Higher rate = more noise per sample, but enables oversampling for better ENOB.
 * 860SPS chosen as default:
 *   - Min inter-channel gap (1.16ms) for pseudo-synchronous FLC100 reading
 *   - signal_processor oversampling (N=32) gives effective ~27Hz update
 *   - ENOB gain: +2.5 bits over raw 16-bit → effective 18.5-bit resolution
 */
typedef enum {
    ADS1115_DR_8SPS   = 0x00,
    ADS1115_DR_16SPS  = 0x20,
    ADS1115_DR_32SPS  = 0x40,
    ADS1115_DR_64SPS  = 0x60,
    ADS1115_DR_128SPS = 0x80,  ///< DEFAULT — used in this driver
    ADS1115_DR_250SPS = 0xA0,
    ADS1115_DR_475SPS = 0xC0,
    ADS1115_DR_860SPS = 0xE0,
} ADS1115_DataRate_t;

/**
 * @brief PGA (Programmable Gain Amplifier) settings.
 * Sets full-scale range. Use ±2.048V for FLC100.
 */
typedef enum {
    ADS1115_PGA_6144MV = 0x00,  ///< ±6.144V — 187.5µV/LSB
    ADS1115_PGA_4096MV = 0x02,  ///< ±4.096V — 125µV/LSB
    ADS1115_PGA_2048MV = 0x04,  ///< ±2.048V — 62.5µV/LSB  ← DEFAULT
    ADS1115_PGA_1024MV = 0x06,  ///< ±1.024V — 31.25µV/LSB
    ADS1115_PGA_0512MV = 0x08,  ///< ±0.512V — 15.625µV/LSB
    ADS1115_PGA_0256MV = 0x0A,  ///< ±0.256V — 7.8125µV/LSB
} ADS1115_PGA_t;

/* =========================================================================
 * DRIVER HANDLE
 * ========================================================================= */

typedef struct {
    i2c_port_t       port;
    uint8_t          i2c_addr;
    ADS1115_DataRate_t data_rate;
    ADS1115_PGA_t    pga;
    bool             initialized;
    bool             continuous_started;

    /* Diagnostics */
    uint32_t         read_count;
    uint32_t         error_count;
    int16_t          last_ain0;
    int16_t          last_ain1;
} ADS1115Driver_t;

/* =========================================================================
 * LIFECYCLE
 * ========================================================================= */

/**
 * @brief Initialize the driver struct with default settings.
 * Does NOT communicate with hardware yet.
 *
 * @param drv   Driver handle to initialize.
 * @param port  I2C port number (I2C_NUM_0).
 */
void ads1115_driver_init(ADS1115Driver_t *drv, i2c_port_t port);

/**
 * @brief Probe the ADS1115 and start continuous conversion.
 *
 * Sends configuration to the chip to begin converting AIN0-AIN1
 * differential continuously at 860SPS (default).
 *
 * @return ESP_OK if chip found and configured.
 *         ESP_ERR_NOT_FOUND if chip not on I2C bus.
 *         ESP_FAIL on I2C error.
 */
esp_err_t ads1115_driver_start(ADS1115Driver_t *drv);

/* =========================================================================
 * READING
 * ========================================================================= */

/**
 * @brief Read a single-ended sample (AINx vs GND).
 *
 * Used ONLY in Phase 3C sensor matching calibration.
 * Temporarily switches MUX from differential to single-ended.
 * Caller MUST call ads1115_driver_start() afterward to restore normal mode.
 *
 * @param drv      Initialised driver handle.
 * @param channel  0 = AIN0 (upper sensor), 1 = AIN1 (lower sensor).
 * @param out_val  Output: raw ADC count.
 * @return ESP_OK on success.
 */
esp_err_t ads1115_read_single_ended(ADS1115Driver_t *drv,
                                     uint8_t          channel,
                                     int16_t         *out_val);

/**
 * @brief Read one differential sample (AIN0 - AIN1).
 *
 * In continuous mode, the chip is always converting. This call simply
 * reads the last converted value from the conversion register.
 * No delay needed — call from your task loop at any time.
 *
 * @param drv       Driver handle.
 * @param out_diff  Output: differential value (AIN1 - AIN0) in ADC counts.
 * @param out_ain0  Output: AIN0 raw value (optional, can be NULL).
 * @param out_ain1  Output: AIN1 raw value (optional, can be NULL).
 *
 * @return ESP_OK on success.
 *         ESP_ERR_INVALID_STATE if driver not started.
 *         ESP_FAIL on I2C error (after retries).
 */
esp_err_t ads1115_read_differential(ADS1115Driver_t *drv,
                                    int16_t *out_diff,
                                    int16_t *out_ain0,
                                    int16_t *out_ain1);

/**
 * @brief Read N samples and return the trimmed mean.
 *
 * Takes N reads, discards top and bottom outliers (if N >= 5),
 * and returns the mean of the remaining values.
 * Use this during calibration for maximum accuracy.
 * NOT suitable for real-time loop — takes N × I2C_read time.
 *
 * @param drv      Driver handle.
 * @param n        Number of samples (recommended: 8-16).
 * @param out_mean Output: trimmed mean in ADC counts.
 * @param out_std  Output: standard deviation (noise estimate). Can be NULL.
 *
 * @return ESP_OK on success, ESP_FAIL if too many read errors.
 */
esp_err_t ads1115_read_trimmed_mean(ADS1115Driver_t *drv,
                                    uint8_t n,
                                    float *out_mean,
                                    float *out_std);

/* =========================================================================
 * CONFIGURATION (runtime adjustable)
 * ========================================================================= */

/**
 * @brief Change PGA gain setting (restarts continuous conversion).
 * Use to implement sensitivity modes — different gains for different soils.
 */
esp_err_t ads1115_set_pga(ADS1115Driver_t *drv, ADS1115_PGA_t pga);

/**
 * @brief Get current LSB size in microvolts for the active PGA setting.
 * Useful for converting ADC counts to physical voltage.
 */
float ads1115_get_lsb_uv(const ADS1115Driver_t *drv);

/* =========================================================================
 * DIAGNOSTICS
 * ========================================================================= */

/**
 * @brief Log driver statistics via ESP_LOGI.
 */
void ads1115_log_stats(const ADS1115Driver_t *drv);

#ifdef __cplusplus
}
#endif
