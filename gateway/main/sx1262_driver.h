#pragma once

#include "driver/spi_master.h"
#include "esp_err.h"
#include <stdint.h>

typedef struct {
    spi_device_handle_t spi;
} sx1262_t;

esp_err_t sx1262_init(sx1262_t *dev);
esp_err_t sx1262_transmit(sx1262_t *dev, const uint8_t *buf, uint8_t len);
esp_err_t sx1262_start_rx_continuous(sx1262_t *dev);
esp_err_t sx1262_read_packet(sx1262_t *dev, uint8_t *buf, uint8_t *len);
esp_err_t sx1262_get_packet_status(sx1262_t *dev, int8_t *rssi_dbm, int8_t *snr_x4);
uint16_t  sx1262_get_irq_status(sx1262_t *dev);
void      sx1262_clear_irq(sx1262_t *dev, uint16_t mask);
void      sx1262_send_sleep_cmd(sx1262_t *dev);
