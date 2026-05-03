#pragma once

#include <stdint.h>
#include <stdbool.h>

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

void payload_encode(lora_payload_t *out,
                    uint8_t node_id, uint8_t seq,
                    uint16_t aqi_raw, bool aqi_ok,
                    int16_t temp_dht_x10, uint16_t humi_dht_x10, bool dht_ok,
                    uint16_t pres_bmp_hpa, int16_t temp_bmp_x10, bool bmp_ok);
