#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sx1262_driver.h"
#include "mq135_driver.h"
#include "dht22_driver.h"
#include "bmp280_driver.h"
#include "payload_encoder.h"
#include "power_manager.h"

#define SPI2_PIN_MOSI    9
#define SPI2_PIN_MISO    8
#define SPI2_PIN_SCK     7
#define SX_SPI_HZ        8000000

#define I2C_PIN_SDA      5
#define I2C_PIN_SCL      6

#define DHT22_GPIO       44
#define CYCLE_PERIOD_MS  30000

static const char *TAG = "main";

static sx1262_t   s_sx;
static mq135_t    s_mq;
static dht22_t    s_dht;
static bmp280_t   s_bmp;

void app_main(void)
{
    int64_t t_start = esp_timer_get_time();

    /* Log wakeup reason */
    esp_sleep_wakeup_cause_t cause = power_manager_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_TIMER) {
        ESP_LOGI(TAG, "Wake reason: RTC_TIMER");
    } else {
        ESP_LOGI(TAG, "Wake reason: %d (power-on or reset)", cause);
    }

    /* ── SPI2 bus ──────────────────────────────────────────────────── */
    spi_bus_config_t buscfg = {
        .mosi_io_num     = SPI2_PIN_MOSI,
        .miso_io_num     = SPI2_PIN_MISO,
        .sclk_io_num     = SPI2_PIN_SCK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 64,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SX_SPI_HZ,
        .mode           = 0,
        .spics_io_num   = 41,
        .queue_size     = 4,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &s_sx.spi));

    /* ── SX1262 init (fatal — no radio = no point running) ─────────── */
    ESP_ERROR_CHECK(sx1262_init(&s_sx));

    /* ── I2C bus ───────────────────────────────────────────────────── */
    i2c_master_bus_config_t i2c_cfg = {
        .i2c_port              = -1,
        .sda_io_num            = I2C_PIN_SDA,
        .scl_io_num            = I2C_PIN_SCL,
        .clk_source            = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt     = 7,
        .flags.enable_internal_pullup = false,
    };
    i2c_master_bus_handle_t i2c_bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_cfg, &i2c_bus));

    /* ── Sensor init (non-fatal — continue with flags set on failure) ─ */
    bool bmp_ok = (bmp280_init(&s_bmp, i2c_bus) == ESP_OK);
    bool mq_ok  = (mq135_init(&s_mq)            == ESP_OK);
    dht22_init(&s_dht, DHT22_GPIO);

    /* ── Read sensors ──────────────────────────────────────────────── */
    bmp280_reading_t bmp_data = {0};
    if (bmp_ok) bmp_ok = (bmp280_read(&s_bmp, &bmp_data) == ESP_OK);

    uint16_t aqi_raw = 0;
    if (mq_ok) mq_ok = (mq135_read_raw(&s_mq, &aqi_raw) == ESP_OK);

    dht22_reading_t dht_data = {0};
    bool dht_ok = (dht22_read(&s_dht, &dht_data) == ESP_OK);

    /* ── Encode and transmit ───────────────────────────────────────── */
    static uint8_t s_seq = 0;
    lora_payload_t payload;
    payload_encode(&payload,
                   NODE_ID_SENSOR_1, s_seq++,
                   aqi_raw, mq_ok,
                   dht_data.temp_x10, dht_data.humi_x10, dht_ok,
                   bmp_data.pres_hpa, bmp_data.temp_x10, bmp_ok);

    ESP_ERROR_CHECK(sx1262_transmit(&s_sx, (uint8_t *)&payload, sizeof(payload)));

    /* ── Cleanup and sleep ─────────────────────────────────────────── */
    mq135_deinit(&s_mq);
    sx1262_send_sleep_cmd(&s_sx);

    power_manager_sleep(t_start, CYCLE_PERIOD_MS);
    /* does not return */
}
