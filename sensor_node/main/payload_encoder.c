#include "payload_encoder.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "pyenc";

_Static_assert(sizeof(lora_payload_t) == 13, "payload size mismatch");

void payload_encode(lora_payload_t *out,
                    uint8_t node_id, uint8_t seq,
                    uint16_t aqi_raw, bool aqi_ok,
                    int16_t temp_dht_x10, uint16_t humi_dht_x10, bool dht_ok,
                    uint16_t pres_bmp_hpa, int16_t temp_bmp_x10, bool bmp_ok)
{
    memset(out, 0, sizeof(*out));
    out->node_id = node_id;
    out->seq     = seq;
    out->flags   = (!aqi_ok ? FLAG_MQ135_FAIL : 0)
                 | (!dht_ok ? FLAG_DHT22_FAIL  : 0)
                 | (!bmp_ok ? FLAG_BMP280_FAIL  : 0);

    if (aqi_ok) out->aqi_raw      = aqi_raw;
    if (dht_ok) {
        out->temp_dht_x10 = temp_dht_x10;
        out->humi_dht_x10 = humi_dht_x10;
    }
    if (bmp_ok) {
        out->pres_bmp_hpa = pres_bmp_hpa;
        out->temp_bmp_x10 = temp_bmp_x10;
    }

    ESP_LOGI(TAG, "seq=%d node=%02X flags=%02X packed %d bytes",
             seq, node_id, out->flags, (int)sizeof(lora_payload_t));
}
