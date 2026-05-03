#include "payload_decoder.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "pydec";

_Static_assert(sizeof(lora_payload_t) == 13, "payload size mismatch");

esp_err_t payload_decode(const uint8_t *buf, uint8_t len, decoded_payload_t *out)
{
    if (len != sizeof(lora_payload_t)) {
        ESP_LOGE(TAG, "unexpected payload length: %d (expected %d)", len, (int)sizeof(lora_payload_t));
        return ESP_ERR_INVALID_SIZE;
    }

    lora_payload_t p;
    memcpy(&p, buf, sizeof(p));

    out->node_id      = p.node_id;
    out->seq          = p.seq;
    out->aqi_raw      = (float)p.aqi_raw;
    out->temp_dht_c   = p.temp_dht_x10 / 10.0f;
    out->humi_dht_pct = p.humi_dht_x10 / 10.0f;
    out->pres_bmp_hpa = p.pres_bmp_hpa;
    out->temp_bmp_c   = p.temp_bmp_x10 / 10.0f;
    out->flags        = p.flags;

    ESP_LOGI(TAG, "seq=%d node=%02X AQI=%.0f T_dht=%.1f H=%.1f P=%u T_bmp=%.1f flags=0x%02X",
             out->seq, out->node_id, out->aqi_raw,
             out->temp_dht_c, out->humi_dht_pct,
             out->pres_bmp_hpa, out->temp_bmp_c, out->flags);
    return ESP_OK;
}
