#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sx1262_driver.h"
#include "payload_decoder.h"
#include "oled_driver.h"
#include "display_manager.h"

#define SPI2_PIN_MOSI    9
#define SPI2_PIN_MISO    8
#define SPI2_PIN_SCK     7
#define SX_SPI_HZ        8000000

#define I2C_PIN_SDA      5
#define I2C_PIN_SCL      6

#define DIO1_GPIO        39
#define NO_SIGNAL_MS     60000
#define RX_POLL_MS       5000

static const char *TAG = "main";

static sx1262_t  s_sx;
static oled_t    s_oled;

static TaskHandle_t s_rx_task_handle;

/* ── DIO1 ISR — fires on rising edge when SX1262 asserts an IRQ ─────────── */
static void IRAM_ATTR dio1_isr(void *arg)
{
    BaseType_t higher_woken = pdFALSE;
    xTaskNotifyFromISR(s_rx_task_handle, 0, eNoAction, &higher_woken);
    portYIELD_FROM_ISR(higher_woken);
}

/* ── RX task — decodes packets and drives the OLED ─────────────────────── */
static void rx_task(void *arg)
{
    TickType_t last_rx_tick = xTaskGetTickCount();

    for (;;) {
        uint32_t notif = 0;
        BaseType_t got = xTaskNotifyWait(0, 0xFFFFFFFF, &notif,
                                          pdMS_TO_TICKS(RX_POLL_MS));

        if (got == pdFALSE) {
            /* Periodic watchdog: check for extended silence */
            if ((xTaskGetTickCount() - last_rx_tick) > pdMS_TO_TICKS(NO_SIGNAL_MS)) {
                display_manager_no_signal(&s_oled);
            }
            /* Restart RX in case a TIMEOUT IRQ fired and left the radio in standby */
            sx1262_start_rx_continuous(&s_sx);
            continue;
        }

        uint16_t irq = sx1262_get_irq_status(&s_sx);
        sx1262_clear_irq(&s_sx, 0xFFFF);

        if (irq & (1u << 6)) { /* CRC_ERR */
            ESP_LOGW(TAG, "CRC error — packet discarded");
            sx1262_start_rx_continuous(&s_sx);
            continue;
        }

        if (!(irq & (1u << 1))) { /* not RX_DONE */
            sx1262_start_rx_continuous(&s_sx);
            continue;
        }

        uint8_t buf[32] = {0};
        uint8_t len = 0;
        sx1262_read_packet(&s_sx, buf, &len);

        int8_t rssi_dbm = 0, snr_x4 = 0;
        sx1262_get_packet_status(&s_sx, &rssi_dbm, &snr_x4);

        ESP_LOGI(TAG, "RX_DONE RSSI=%d dBm SNR=%+.1f dB len=%d",
                 rssi_dbm, snr_x4 / 4.0f, len);

        decoded_payload_t decoded = {0};
        if (payload_decode(buf, len, &decoded) == ESP_OK) {
            display_manager_update(&s_oled, &decoded, rssi_dbm, snr_x4);
            last_rx_tick = xTaskGetTickCount();
        }

        sx1262_start_rx_continuous(&s_sx);
    }
}

/* ── app_main ───────────────────────────────────────────────────────────── */
void app_main(void)
{
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

    /* ── SX1262 init ───────────────────────────────────────────────── */
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

    /* ── OLED init + splash ────────────────────────────────────────── */
    ESP_ERROR_CHECK(oled_init(&s_oled, i2c_bus));
    oled_clear(&s_oled);
    oled_write_string(&s_oled, 2, 0, "  LoRa Gateway  ");
    oled_write_string(&s_oled, 3, 0, "   Waiting...   ");

    /* ── RX task + DIO1 interrupt ─────────────────────────────────── */
    xTaskCreate(rx_task, "rx_task", 4096, NULL, 5, &s_rx_task_handle);

    gpio_install_isr_service(0);

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << DIO1_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    gpio_config(&io);
    gpio_isr_handler_add(DIO1_GPIO, dio1_isr, NULL);

    /* ── Start continuous RX ───────────────────────────────────────── */
    ESP_ERROR_CHECK(sx1262_start_rx_continuous(&s_sx));

    ESP_LOGI(TAG, "gateway ready — listening on 868 MHz");
    /* app_main returns; rx_task runs indefinitely */
}
