#include "bmp280_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bmp280";

#define BMP280_ADDR    0x76
#define REG_CHIP_ID    0xD0
#define REG_RESET      0xE0
#define REG_CALIB_00   0x88
#define REG_CTRL_MEAS  0xF4
#define REG_CONFIG     0xF5
#define REG_PRESS_MSB  0xF7
#define BMP280_CHIP_ID 0x58

static esp_err_t reg_write(bmp280_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev->dev_handle, buf, 2, pdMS_TO_TICKS(10));
}

static esp_err_t reg_read(bmp280_t *dev, uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_transmit_receive(dev->dev_handle, &reg, 1, out, len, pdMS_TO_TICKS(10));
}

static void read_calibration(bmp280_t *dev)
{
    uint8_t raw[24];
    reg_read(dev, REG_CALIB_00, raw, 24);

    dev->dig_T1 = (uint16_t)(raw[1]  << 8 | raw[0]);
    dev->dig_T2 = (int16_t) (raw[3]  << 8 | raw[2]);
    dev->dig_T3 = (int16_t) (raw[5]  << 8 | raw[4]);
    dev->dig_P1 = (uint16_t)(raw[7]  << 8 | raw[6]);
    dev->dig_P2 = (int16_t) (raw[9]  << 8 | raw[8]);
    dev->dig_P3 = (int16_t) (raw[11] << 8 | raw[10]);
    dev->dig_P4 = (int16_t) (raw[13] << 8 | raw[12]);
    dev->dig_P5 = (int16_t) (raw[15] << 8 | raw[14]);
    dev->dig_P6 = (int16_t) (raw[17] << 8 | raw[16]);
    dev->dig_P7 = (int16_t) (raw[19] << 8 | raw[18]);
    dev->dig_P8 = (int16_t) (raw[21] << 8 | raw[20]);
    dev->dig_P9 = (int16_t) (raw[23] << 8 | raw[22]);
}

esp_err_t bmp280_init(bmp280_t *dev, i2c_master_bus_handle_t bus)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BMP280_ADDR,
        .scl_speed_hz    = 400000,
    };
    esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &dev->dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    uint8_t chip_id = 0;
    ret = reg_read(dev, REG_CHIP_ID, &chip_id, 1);
    if (ret != ESP_OK || chip_id != BMP280_CHIP_ID) {
        ESP_LOGE(TAG, "chip ID mismatch: got 0x%02X, expected 0x%02X", chip_id, BMP280_CHIP_ID);
        return ESP_FAIL;
    }

    reg_write(dev, REG_RESET, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(10));

    read_calibration(dev);

    /* osrs_t=1 (×1), osrs_p=1 (×1), mode=normal */
    reg_write(dev, REG_CTRL_MEAS, 0x27);
    /* standby=0.5ms, filter=off */
    reg_write(dev, REG_CONFIG, 0x00);

    ESP_LOGI(TAG, "BMP280 init OK, chip_id=0x%02X", chip_id);
    return ESP_OK;
}

esp_err_t bmp280_read(bmp280_t *dev, bmp280_reading_t *out)
{
    uint8_t raw[6];
    esp_err_t ret = reg_read(dev, REG_PRESS_MSB, raw, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "read failed: %s", esp_err_to_name(ret));
        return ret;
    }

    int32_t adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);

    /* BMP280 datasheet Appendix B — integer compensation */
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dev->dig_T1 << 1))) * (int32_t)dev->dig_T2) >> 11;
    int32_t var2 = (((((adc_T >> 4) - (int32_t)dev->dig_T1) *
                      ((adc_T >> 4) - (int32_t)dev->dig_T1)) >> 12) *
                    (int32_t)dev->dig_T3) >> 14;
    int32_t t_fine = var1 + var2;
    int32_t T = (t_fine * 5 + 128) >> 8;   /* units: 0.01 °C */

    int64_t p_var1 = (int64_t)t_fine - 128000;
    int64_t p_var2 = p_var1 * p_var1 * (int64_t)dev->dig_P6;
    p_var2 += (p_var1 * (int64_t)dev->dig_P5) << 17;
    p_var2 += ((int64_t)dev->dig_P4) << 35;
    p_var1  = ((p_var1 * p_var1 * (int64_t)dev->dig_P3) >> 8) +
              ((p_var1 * (int64_t)dev->dig_P2) << 12);
    p_var1  = (((int64_t)1 << 47) + p_var1) * (int64_t)dev->dig_P1 >> 33;
    int64_t P = 0;
    if (p_var1 != 0) {
        P  = 1048576 - adc_P;
        P  = (((P << 31) - p_var2) * 3125) / p_var1;
        p_var1 = ((int64_t)dev->dig_P9 * (P >> 13) * (P >> 13)) >> 25;
        p_var2 = ((int64_t)dev->dig_P8 * P) >> 19;
        P = ((P + p_var1 + p_var2) >> 8) + ((int64_t)dev->dig_P7 << 4);
        /* P now in Pa×256 */
    }

    /* T in 0.01°C → ×10 representation = T/10 */
    out->temp_x10 = (int16_t)(T / 10);
    /* P in Pa×256 → hPa = P/25600 */
    out->pres_hpa = (uint16_t)(P / 25600);

    ESP_LOGI(TAG, "P=%u hPa T=%d.%d C",
             out->pres_hpa,
             out->temp_x10 / 10, abs(out->temp_x10 % 10));
    return ESP_OK;
}
