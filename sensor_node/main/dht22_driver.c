#include "dht22_driver.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

static const char *TAG = "dht22";

esp_err_t dht22_init(dht22_t *dev, int gpio_num)
{
    dev->gpio_num = gpio_num;
    ESP_LOGI(TAG, "DHT22 init OK (GPIO%d)", gpio_num);
    return ESP_OK;
}

esp_err_t dht22_read(dht22_t *dev, dht22_reading_t *out)
{
    int gpio = dev->gpio_num;

    /* ── Start signal ─────────────────────────────────────────────────── */
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(gpio, 0);
    esp_rom_delay_us(2000);                   /* host low ≥ 1 ms */
    gpio_set_direction(gpio, GPIO_MODE_INPUT);
    gpio_set_pull_mode(gpio, GPIO_FLOATING);  /* external 5kΩ pull-up holds line high */

    /* ── Sensor response ─────────────────────────────────────────────── */
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    taskENTER_CRITICAL(&mux);

    /* wait for sensor to pull low (within 40 µs) */
    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level(gpio) == 1) {
        if (esp_timer_get_time() - t0 > 100) {
            taskEXIT_CRITICAL(&mux);
            ESP_LOGE(TAG, "no response from sensor");
            return ESP_ERR_TIMEOUT;
        }
    }
    /* wait for low phase end (~80 µs) */
    while (gpio_get_level(gpio) == 0);
    /* wait for high phase end (~80 µs) */
    while (gpio_get_level(gpio) == 1);

    /* ── Read 40 bits ────────────────────────────────────────────────── */
    uint8_t data[5] = {0};
    for (int i = 0; i < 40; i++) {
        /* wait for low→high transition (start of data bit high pulse) */
        while (gpio_get_level(gpio) == 0);
        int64_t t_high = esp_timer_get_time();
        /* measure high pulse duration */
        while (gpio_get_level(gpio) == 1);
        int64_t dur = esp_timer_get_time() - t_high;
        /* >50 µs → bit 1, else bit 0 */
        if (dur > 50) {
            data[i / 8] |= (1 << (7 - (i % 8)));
        }
    }

    taskEXIT_CRITICAL(&mux);

    /* ── Checksum ────────────────────────────────────────────────────── */
    uint8_t sum = data[0] + data[1] + data[2] + data[3];
    if ((sum & 0xFF) != data[4]) {
        ESP_LOGE(TAG, "checksum error: calc=0x%02X recv=0x%02X", sum & 0xFF, data[4]);
        return ESP_ERR_INVALID_CRC;
    }

    /* ── Decode ──────────────────────────────────────────────────────── */
    out->humi_x10    = (uint16_t)((data[0] << 8) | data[1]);
    uint16_t t_raw   = (uint16_t)(((data[2] & 0x7F) << 8) | data[3]);
    out->temp_x10    = (data[2] & 0x80) ? -(int16_t)t_raw : (int16_t)t_raw;

    ESP_LOGI(TAG, "T=%d.%d C H=%d.%d %%",
             out->temp_x10 / 10, abs(out->temp_x10 % 10),
             out->humi_x10 / 10, out->humi_x10 % 10);
    return ESP_OK;
}
