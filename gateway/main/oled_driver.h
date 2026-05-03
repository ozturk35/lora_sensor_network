#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>

typedef struct {
    i2c_master_dev_handle_t dev_handle;
} oled_t;

esp_err_t oled_init(oled_t *dev, i2c_master_bus_handle_t bus);
void      oled_clear(oled_t *dev);
void      oled_write_string(oled_t *dev, uint8_t page, uint8_t col, const char *str);
