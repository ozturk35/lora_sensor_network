#pragma once

#include "esp_err.h"
#include <stdint.h>

#define NODE_ID_SENSOR_1  0x01
#define FLAG_MQ135_FAIL   0x01
#define FLAG_DHT22_FAIL   0x02
#define FLAG_BMP280_FAIL  0x04

typedef struct __attribute__((packed)) {
    uint8_t  node_id;
    uint8_t  seq;
    uint16_t aqi_raw;       /* MQ-135 ADC 12-bit, little-endian (native) */
    int16_t  temp_dht_x10;  /* DHT22 temp × 10 */
    uint16_t humi_dht_x10;  /* DHT22 humidity × 10 */
    uint16_t pres_bmp_hpa;  /* BMP280 pressure whole hPa */
    int16_t  temp_bmp_x10;  /* BMP280 temp × 10 */
    uint8_t  flags;
} lora_payload_t;           /* 13 bytes */

typedef struct {
    uint8_t  node_id;
    uint8_t  seq;
    float    aqi_raw;
    float    temp_dht_c;
    float    humi_dht_pct;
    uint16_t pres_bmp_hpa;
    float    temp_bmp_c;
    uint8_t  flags;
} decoded_payload_t;

esp_err_t payload_decode(const uint8_t *buf, uint8_t len, decoded_payload_t *out);
