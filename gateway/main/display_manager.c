#include "display_manager.h"
#include "esp_log.h"
#include <stdio.h>

static const char *TAG = "disp";

void display_manager_update(oled_t *oled, const decoded_payload_t *p,
                            int8_t rssi_dbm, int8_t snr_x4)
{
    oled_clear(oled);
    char line[22];
    float snr_db = snr_x4 / 4.0f;

    snprintf(line, sizeof(line), "AQI: %4d", (int)p->aqi_raw);
    oled_write_string(oled, 0, 0, line);

    snprintf(line, sizeof(line), "T:%+5.1fC H:%4.1f%%", p->temp_dht_c, p->humi_dht_pct);
    oled_write_string(oled, 1, 0, line);

    snprintf(line, sizeof(line), "P: %4u hPa", p->pres_bmp_hpa);
    oled_write_string(oled, 2, 0, line);

    snprintf(line, sizeof(line), "Tb:%+5.1fC", p->temp_bmp_c);
    oled_write_string(oled, 3, 0, line);

    snprintf(line, sizeof(line), "RSSI:%4d dBm", rssi_dbm);
    oled_write_string(oled, 4, 0, line);

    snprintf(line, sizeof(line), "SNR:%+4.1fdB #%d", snr_db, p->seq);
    oled_write_string(oled, 5, 0, line);

    ESP_LOGI(TAG, "OLED updated");
}

void display_manager_no_signal(oled_t *oled)
{
    oled_write_string(oled, 0, 0, "AQI:  ----      ");
    oled_write_string(oled, 1, 0, "T:  -.-- H: -.- ");
    oled_write_string(oled, 2, 0, "** NO SIGNAL ** ");
    oled_write_string(oled, 3, 0, "                ");
    oled_write_string(oled, 4, 0, "RSSI: --- dBm   ");
    oled_write_string(oled, 5, 0, "SNR:  --- dB    ");
    ESP_LOGW(TAG, "no signal — display updated");
}
