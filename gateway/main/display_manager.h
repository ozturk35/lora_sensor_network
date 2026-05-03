#pragma once

#include "oled_driver.h"
#include "payload_decoder.h"
#include <stdint.h>

void display_manager_update(oled_t *oled, const decoded_payload_t *p,
                            int8_t rssi_dbm, int8_t snr_x4);
void display_manager_no_signal(oled_t *oled);
