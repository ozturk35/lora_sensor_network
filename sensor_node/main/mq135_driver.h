#pragma once

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include <stdint.h>

typedef struct {
    adc_oneshot_unit_handle_t adc_handle;
} mq135_t;

esp_err_t mq135_init(mq135_t *dev);
esp_err_t mq135_read_raw(mq135_t *dev, uint16_t *out_raw);
void      mq135_deinit(mq135_t *dev);
