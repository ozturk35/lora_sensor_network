#include "mq135_driver.h"
#include "esp_log.h"

static const char *TAG = "mq135";

esp_err_t mq135_init(mq135_t *dev)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t ret = adc_oneshot_new_unit(&unit_cfg, &dev->adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ret = adc_oneshot_config_channel(dev->adc_handle, ADC_CHANNEL_0, &ch_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(dev->adc_handle);
        return ret;
    }

    ESP_LOGI(TAG, "MQ-135 init OK (GPIO1, ADC1_CH0, 12 dB attenuation)");
    return ESP_OK;
}

esp_err_t mq135_read_raw(mq135_t *dev, uint16_t *out_raw)
{
    int raw = 0;
    esp_err_t ret = adc_oneshot_read(dev->adc_handle, ADC_CHANNEL_0, &raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_read failed: %s", esp_err_to_name(ret));
        return ret;
    }
    *out_raw = (uint16_t)raw;
    ESP_LOGI(TAG, "ADC raw=%u", *out_raw);
    return ESP_OK;
}

void mq135_deinit(mq135_t *dev)
{
    if (dev->adc_handle) {
        adc_oneshot_del_unit(dev->adc_handle);
        dev->adc_handle = NULL;
    }
}
