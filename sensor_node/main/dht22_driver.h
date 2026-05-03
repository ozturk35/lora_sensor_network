#pragma once

#include "esp_err.h"
#include <stdint.h>

typedef struct {
    int gpio_num;
} dht22_t;

typedef struct {
    int16_t  temp_x10;
    uint16_t humi_x10;
} dht22_reading_t;

esp_err_t dht22_init(dht22_t *dev, int gpio_num);
esp_err_t dht22_read(dht22_t *dev, dht22_reading_t *out);
