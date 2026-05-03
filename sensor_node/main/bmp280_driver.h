#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>

typedef struct {
    i2c_master_dev_handle_t dev_handle;
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} bmp280_t;

typedef struct {
    int16_t  temp_x10;
    uint16_t pres_hpa;
} bmp280_reading_t;

esp_err_t bmp280_init(bmp280_t *dev, i2c_master_bus_handle_t bus);
esp_err_t bmp280_read(bmp280_t *dev, bmp280_reading_t *out);
