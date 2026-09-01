/**
 * @file ads1115_driver.c
 * @brief ADS1115 16-bit ADC driver — implementation (BUGFIXED).
 */

#include "ads1115_driver.h"
#include "system_monitor/system_monitor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

static const char *TAG = "ADS1115";

/* =========================================================================
 * ADS1115 REGISTER MAP
 * ========================================================================= */

#define REG_CONVERSION  0x00
#define REG_CONFIG      0x01
#define REG_LO_THRESH   0x02
#define REG_HI_THRESH   0x03

/*
 * CONFIG REGISTER BIT FIELDS (16-bit word)
 * Bit 15    : OS        — Operational status (write 1 to start single-shot)
 * Bits 14:12: MUX[2:0] — Input mux
 * Bits 11:9 : PGA[2:0] — Gain (see ADS1115_PGA_t)
 * Bit 8     : MODE      — 0=Continuous, 1=Single-shot
 * Bits 7:5  : DR[2:0]  — Data rate
 * Bits 4:0  : COMP      — Comparator settings
 *
 * MSB Byte Layout (Bits 15-8):
 *   [ OS | MUX2 | MUX1 | MUX0 | PGA2 | PGA1 | PGA0 | MODE ]
 *   Bit 7   6      5      4      3     2     1     0
 *
 * Note: PGA[2:0] aligns with bits 3, 2, 1 of the MSB byte!
 */

#define I2C_TIMEOUT_MS  50
#define READ_RETRY      3

/* =========================================================================
 * PRIVATE HELPERS
 * ========================================================================= */

static esp_err_t i2c_write_reg(i2c_port_t port, uint8_t addr,
                                uint8_t reg, uint8_t msb, uint8_t lsb)
{
    uint8_t buf[3] = {reg, msb, lsb};
    return i2c_master_write_to_device(port, addr, buf, sizeof(buf),
                                      pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static esp_err_t i2c_read_reg(i2c_port_t port, uint8_t addr,
                               uint8_t reg, int16_t *out_val)
{
    uint8_t data[2] = {0};
    esp_err_t ret = i2c_master_write_read_device(
        port, addr,
        &reg, 1,
        data, 2,
        pdMS_TO_TICKS(I2C_TIMEOUT_MS)
    );
    if (ret == ESP_OK) {
        *out_val = (int16_t)((data[0] << 8) | data[1]);
    }
    return ret;
}

/**
 * Build config register MSB based on PGA setting.
 * MUX=000 (AIN0-AIN1 diff), MODE=0 (continuous).
 */
static uint8_t build_config_msb(ADS1115_PGA_t pga)
{
    /* BUGFIX: PGA bits (11, 10, 9) land at MSB byte bits (3, 2, 1).
     * If enum ADS1115_PGA_1024MV is 3 (0b011), we must shift left by 1 
     * to place it at 0b0110 (0x06) in the MSB byte. 
     * Without this, writing PGA=3 results in 0x03, setting the wrong voltage! */
    return (uint8_t)((uint8_t)pga << 1);
}

/* =========================================================================
 * DATA RATE HELPER
 * ========================================================================= */

static uint32_t get_conversion_time_ms(ADS1115_DataRate_t dr)
{
    switch (dr) {
        case ADS1115_DR_860SPS: return 3;    
        case ADS1115_DR_475SPS: return 4;
        case ADS1115_DR_250SPS: return 6;
        case ADS1115_DR_128SPS: return 10;   
        case ADS1115_DR_64SPS:  return 18;
        case ADS1115_DR_32SPS:  return 34;
        case ADS1115_DR_16SPS:  return 65;
        case ADS1115_DR_8SPS:   return 130;
        default:                return 10;
    }
}

static esp_err_t start_continuous(ADS1115Driver_t *drv)
{
    uint8_t msb = build_config_msb(drv->pga);
    uint8_t lsb = (uint8_t)drv->data_rate | 0x03;  /* DR | COMP_QUE=11 (disable) */

    for (int attempt = 0; attempt < READ_RETRY; attempt++) {
        esp_err_t ret = i2c_write_reg(drv->port, drv->i2c_addr,
                                      REG_CONFIG, msb, lsb);
        if (ret == ESP_OK) {
            /* Give ADC one conversion cycle to settle with new config */
            vTaskDelay(pdMS_TO_TICKS(10));
            return ESP_OK;
        }
        ESP_LOGW(TAG, "Config write attempt %d failed: %s",
                 attempt + 1, esp_err_to_name(ret));
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return ESP_FAIL;
}

/* =========================================================================
 * LIFECYCLE
 * ========================================================================= */

void ads1115_driver_init(ADS1115Driver_t *drv, i2c_port_t port)
{
    memset(drv, 0, sizeof(ADS1115Driver_t));
    drv->port               = port;
    drv->i2c_addr           = ADS1115_I2C_ADDR;
    drv->data_rate          = ADS1115_DR_860SPS;  
    drv->pga                = ADS1115_PGA_2048MV;  
    drv->initialized        = false;
    drv->continuous_started = false;
}

esp_err_t ads1115_driver_start(ADS1115Driver_t *drv)
{
    /* Probe: attempt to read the config register */
    int16_t config_val = 0;
    esp_err_t probe = i2c_read_reg(drv->port, drv->i2c_addr,
                                   REG_CONFIG, &config_val);
    if (probe != ESP_OK) {
        ESP_LOGE(TAG, "ADS1115 not found at I2C addr 0x%02X — %s",
                 drv->i2c_addr, esp_err_to_name(probe));
        sysmon_report_fault(FAULT_ADC_NOT_FOUND, "ADS1115 not detected on I2C");
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "ADS1115 detected. Config reg: 0x%04X", (unsigned)config_val);

    esp_err_t ret = start_continuous(drv);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start continuous conversion");
        sysmon_report_fault(FAULT_ADC_NOT_FOUND, "ADS1115 config failed");
        return ret;
    }

    drv->initialized        = true;
    drv->continuous_started = true;

    float    lsb_uv = ads1115_get_lsb_uv(drv);
    uint32_t sps    = 0;
    switch (drv->data_rate) {
        case ADS1115_DR_860SPS: sps = 860;  break;
        case ADS1115_DR_475SPS: sps = 475;  break;
        case ADS1115_DR_250SPS: sps = 250;  break;
        case ADS1115_DR_128SPS: sps = 128;  break;
        case ADS1115_DR_64SPS:  sps = 64;   break;
        case ADS1115_DR_32SPS:  sps = 32;   break;
        case ADS1115_DR_16SPS:  sps = 16;   break;
        case ADS1115_DR_8SPS:   sps = 8;    break;
        default:                sps = 0;    break;
    }
    ESP_LOGI(TAG, "ADS1115 ready. Mode: Continuous | DR: %luSPS | "
             "PGA: ±%.3fV | LSB: %.3f µV",
             (unsigned long)sps,
             lsb_uv * 32768.0f / 1e6f,   
             lsb_uv);

    return ESP_OK;
}

/* =========================================================================
 * READING
 * ========================================================================= */

esp_err_t ads1115_read_differential(ADS1115Driver_t *drv,
                                    int16_t *out_diff,
                                    int16_t *out_ain0,
                                    int16_t *out_ain1)
{
    if (!drv->initialized || !drv->continuous_started) {
        return ESP_ERR_INVALID_STATE;
    }

    int16_t diff = 0;
    esp_err_t ret = ESP_FAIL;

    for (int attempt = 0; attempt < READ_RETRY; attempt++) {
        ret = i2c_read_reg(drv->port, drv->i2c_addr, REG_CONVERSION, &diff);
        if (ret == ESP_OK) break;

        /* BUGFIX: Removed drv->error_count++ from inside the loop */
        ESP_LOGD(TAG, "Read attempt %d failed", attempt + 1);
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (ret != ESP_OK) {
        /* Increment error count ONCE per failed read call, not per retry */
        drv->error_count++;
        if ((drv->error_count % 10) == 1) {
            ESP_LOGE(TAG, "Persistent I2C errors: %lu total",
                     (unsigned long)drv->error_count);
            sysmon_report_fault(FAULT_ADC_READ_ERROR, "ADS1115 persistent read failures");
        }
        return ret;
    }

    drv->read_count++;

    /* Direct assignment preserves correct sign based on physical coil wiring */
    *out_diff = diff;
    if (out_ain0) *out_ain0 = 0;  /* Not applicable in differential mode */
    if (out_ain1) *out_ain1 = 0;

    return ESP_OK;
}

/* =========================================================================
 * TRIMMED MEAN (for calibration use only)
 * ========================================================================= */

static int compare_int16(const void *a, const void *b)
{
    return (int)(*(const int16_t *)a) - (int)(*(const int16_t *)b);
}

esp_err_t ads1115_read_trimmed_mean(ADS1115Driver_t *drv,
                                    uint8_t n,
                                    float *out_mean,
                                    float *out_std)
{
    if (n < 3 || n > 64) return ESP_ERR_INVALID_ARG;
    if (!drv->initialized) return ESP_ERR_INVALID_STATE;

    int16_t samples[64];
    uint8_t valid = 0;

    uint32_t conv_ms = get_conversion_time_ms(drv->data_rate);

    for (uint8_t i = 0; i < n; i++) {
        int16_t diff = 0;
        esp_err_t ret = ads1115_read_differential(drv, &diff, NULL, NULL);
        if (ret == ESP_OK) {
            samples[valid++] = diff;
        }
        vTaskDelay(pdMS_TO_TICKS(conv_ms));
    }

    if (valid < 3) {
        ESP_LOGE(TAG, "Too few valid samples for trimmed mean: %d/%d", valid, n);
        return ESP_FAIL;
    }

    qsort(samples, valid, sizeof(int16_t), compare_int16);

    uint8_t trim = (valid >= 5) ? (valid / 10) : 0;
    uint8_t start = trim;
    uint8_t end   = valid - trim;

    int32_t sum = 0;
    for (uint8_t i = start; i < end; i++) {
        sum += samples[i];
    }
    float mean = (float)sum / (float)(end - start);

    if (out_std) {
        float var_sum = 0.0f;
        for (uint8_t i = start; i < end; i++) {
            float d = (float)samples[i] - mean;
            var_sum += d * d;
        }
        *out_std = sqrtf(var_sum / (float)(end - start));
    }

    *out_mean = mean;
    return ESP_OK;
}

/* =========================================================================
 * CONFIGURATION
 * ========================================================================= */

esp_err_t ads1115_set_pga(ADS1115Driver_t *drv, ADS1115_PGA_t pga)
{
    drv->pga = pga;
    ESP_LOGI(TAG, "PGA changed → %.3f µV/LSB", ads1115_get_lsb_uv(drv));
    return start_continuous(drv);
}

float ads1115_get_lsb_uv(const ADS1115Driver_t *drv)
{
    switch (drv->pga) {
        case ADS1115_PGA_6144MV: return 187.5f;
        case ADS1115_PGA_4096MV: return 125.0f;
        case ADS1115_PGA_2048MV: return 62.5f;
        case ADS1115_PGA_1024MV: return 31.25f;
        case ADS1115_PGA_0512MV: return 15.625f;
        case ADS1115_PGA_0256MV: return 7.8125f;
        default:                 return 62.5f;
    }
}

/* =========================================================================
 * DIAGNOSTICS
 * ========================================================================= */

void ads1115_log_stats(const ADS1115Driver_t *drv)
{
    ESP_LOGI(TAG, "=== ADS1115 Stats ===");
    ESP_LOGI(TAG, "  Reads:  %lu", (unsigned long)drv->read_count);
    ESP_LOGI(TAG, "  Errors: %lu (%.1f%%)", (unsigned long)drv->error_count,
             drv->read_count > 0
                 ? (100.0f * drv->error_count / drv->read_count)
                 : 0.0f);
    ESP_LOGI(TAG, "  LSB:    %.3f µV", ads1115_get_lsb_uv(drv));
}

/* =========================================================================
 * SINGLE-ENDED READ — Phase 3C Sensor Matching
 *
 * Temporarily switches ADS1115 MUX from differential (AIN0-AIN1) to
 * single-ended (AINx vs GND) to read each FLC100 sensor independently.
 *
 * ADS1115 CONFIG MSB bits [6:4] = MUX field:
 *   000 → AIN0-AIN1 differential (normal operation)
 *   100 → AIN0-GND  single-ended (channel=0, FLC100 upper sensor)
 *   101 → AIN1-GND  single-ended (channel=1, FLC100 lower sensor)
 *
 * After reading, caller MUST call ads1115_driver_start() to restore
 * continuous differential mode.
 *
 * SAFETY: called only from devcal_phase3c while adc_task is blocked
 * waiting for calibration to finish. No I2C bus contention possible.
 * ========================================================================= */

esp_err_t ads1115_read_single_ended(ADS1115Driver_t *drv,
                                     uint8_t          channel,
                                     int16_t         *out_val)
{
    if (!drv->initialized) return ESP_ERR_INVALID_STATE;
    if (channel > 1u || !out_val) return ESP_ERR_INVALID_ARG;

    /* Build config MSB: PGA bits [3:1] + MUX bits [6:4]
     * AIN0-GND: MUX=100 → bit6=1 → mask 0x40
     * AIN1-GND: MUX=101 → bit6+bit4=1 → mask 0x50 */
    uint8_t pga_bits = (uint8_t)((uint8_t)drv->pga << 1u);
    uint8_t mux_mask = (channel == 0u) ? 0x40u : 0x50u;
    uint8_t msb      = pga_bits | mux_mask;
    uint8_t lsb      = (uint8_t)drv->data_rate | 0x03u;  /* COMP disabled */

    /* Write new config — switches to single-ended immediately */
    esp_err_t ret = i2c_write_reg(drv->port, drv->i2c_addr,
                                   REG_CONFIG, msb, lsb);
    if (ret != ESP_OK) return ret;

    /* Wait for one full conversion: 860SPS = 1.16ms → use 3ms for margin */
    vTaskDelay(pdMS_TO_TICKS(3));

    int16_t val = 0;
    ret = i2c_read_reg(drv->port, drv->i2c_addr, REG_CONVERSION, &val);
    if (ret == ESP_OK) *out_val = val;

    return ret;
}
