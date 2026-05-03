#include "sx1262_driver.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "sx126";

/* Pin assignments */
#define SX_PIN_RST   42
#define SX_PIN_NSS   41
#define SX_PIN_BUSY  40
#define SX_PIN_DIO1  39
/* GPIO38 = DIO2/SF_SW1: chip-driven after SetDio2AsRfSwitchCtrl — MCU must NOT configure */
#define SX_PIN_MOSI   9
#define SX_PIN_MISO   8
#define SX_PIN_SCK    7

/* SX1262 opcodes */
#define CMD_SET_STANDBY           0x80
#define CMD_SET_PACKET_TYPE       0x8A
#define CMD_SET_RF_FREQ           0x86
#define CMD_SET_TX_PARAMS         0x8E
#define CMD_SET_MOD_PARAMS        0x8B
#define CMD_SET_PKT_PARAMS        0x8C
#define CMD_SET_DIO2_RF_SW_CTRL   0x9D
#define CMD_SET_DIO_IRQ_PARAMS    0x08
#define CMD_SET_TX                0x83
#define CMD_SET_RX                0x82
#define CMD_GET_PKT_STATUS        0x14
#define CMD_READ_BUFFER           0x1E
#define CMD_WRITE_BUFFER          0x0E
#define CMD_CLEAR_IRQ_STATUS      0x02
#define CMD_GET_IRQ_STATUS        0x12
#define CMD_SET_BUFFER_BASE_ADDR  0x8F
#define CMD_GET_RX_BUFFER_STATUS  0x13
#define CMD_SET_SLEEP             0x84

/* IRQ bit masks */
#define IRQ_TX_DONE   (1u << 0)
#define IRQ_RX_DONE   (1u << 1)
#define IRQ_TIMEOUT   (1u << 9)
#define IRQ_CRC_ERR   (1u << 6)
#define IRQ_HDR_ERR   (1u << 11)

/* ─── Internal helpers ──────────────────────────────────────────────────── */

static esp_err_t wait_busy(void)
{
    int64_t deadline = esp_timer_get_time() + 10000; /* 10 ms */
    while (gpio_get_level(SX_PIN_BUSY)) {
        if (esp_timer_get_time() > deadline) {
            ESP_LOGE(TAG, "BUSY timeout");
            return ESP_ERR_TIMEOUT;
        }
    }
    return ESP_OK;
}

static void spi_cmd(sx1262_t *dev, const uint8_t *tx, uint8_t *rx, int len)
{
    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_polling_transmit(dev->spi, &t);
}


static void cmd2(sx1262_t *dev, uint8_t opcode, const uint8_t *params, int n)
{
    wait_busy();
    uint8_t tx[16];
    tx[0] = opcode;
    memcpy(&tx[1], params, n);
    spi_cmd(dev, tx, NULL, 1 + n);
}

/* ─── Public API ────────────────────────────────────────────────────────── */

esp_err_t sx1262_init(sx1262_t *dev)
{
    /* RST pulse */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << SX_PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(SX_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level(SX_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* BUSY pin */
    io.pin_bit_mask = (1ULL << SX_PIN_BUSY);
    io.mode = GPIO_MODE_INPUT;
    gpio_config(&io);

    /* DIO1 pin — polling on sensor node; gateway caller configures ISR after init */
    io.pin_bit_mask = (1ULL << SX_PIN_DIO1);
    io.mode = GPIO_MODE_INPUT;
    gpio_config(&io);

    /* SetStandby(STDBY_RC) */
    { uint8_t p[] = {0x00}; cmd2(dev, CMD_SET_STANDBY, p, 1); }

    /* SetPacketType(LoRa=0x01) */
    { uint8_t p[] = {0x01}; cmd2(dev, CMD_SET_PACKET_TYPE, p, 1); }

    /* SetRfFrequency: 868.0 MHz → word = 0x36400000 */
    { uint8_t p[] = {0x36, 0x40, 0x00, 0x00}; cmd2(dev, CMD_SET_RF_FREQ, p, 4); }

    /* SetDio2AsRfSwitchCtrl(enable=0x01) — critical for Wio SX1262 antenna switch */
    { uint8_t p[] = {0x01}; cmd2(dev, CMD_SET_DIO2_RF_SW_CTRL, p, 1); }

    /* SetTxParams(power=+14dBm=0x16, rampTime=200µs=0x04) */
    { uint8_t p[] = {0x16, 0x04}; cmd2(dev, CMD_SET_TX_PARAMS, p, 2); }

    /* SetModulationParams(SF7=0x07, BW125=0x04, CR4/5=0x01, ldro=0x00) */
    { uint8_t p[] = {0x07, 0x04, 0x01, 0x00}; cmd2(dev, CMD_SET_MOD_PARAMS, p, 4); }

    /* SetPacketParams(preamble=8, explicitHeader=0, payloadLen=13, crcOn=1, iqInvert=0) */
    { uint8_t p[] = {0x00, 0x08, 0x00, 13, 0x01, 0x00}; cmd2(dev, CMD_SET_PKT_PARAMS, p, 6); }

    /* SetBufferBaseAddress(TX=0x00, RX=0x00) */
    { uint8_t p[] = {0x00, 0x00}; cmd2(dev, CMD_SET_BUFFER_BASE_ADDR, p, 2); }

    /* SetDioIrqParams: TX_DONE|RX_DONE|TIMEOUT|CRC_ERR|HDR_ERR on DIO1 */
    {
        uint16_t mask = IRQ_TX_DONE | IRQ_RX_DONE | IRQ_TIMEOUT | IRQ_CRC_ERR | IRQ_HDR_ERR;
        uint8_t p[] = {
            (mask >> 8) & 0xFF, mask & 0xFF,  /* irqMask */
            (mask >> 8) & 0xFF, mask & 0xFF,  /* dio1Mask */
            0x00, 0x00,                        /* dio2Mask */
            0x00, 0x00,                        /* dio3Mask */
        };
        cmd2(dev, CMD_SET_DIO_IRQ_PARAMS, p, 8);
    }

    ESP_LOGI(TAG, "SX1262 init OK");
    return ESP_OK;
}

esp_err_t sx1262_transmit(sx1262_t *dev, const uint8_t *buf, uint8_t len)
{
    /* WriteBuffer(offset=0x00, payload) */
    wait_busy();
    {
        uint8_t tx[2 + len];
        tx[0] = CMD_WRITE_BUFFER;
        tx[1] = 0x00;
        memcpy(&tx[2], buf, len);
        spi_transaction_t t = { .length = (2 + len) * 8, .tx_buffer = tx };
        spi_device_polling_transmit(dev->spi, &t);
    }

    /* Update payload length in packet params */
    { uint8_t p[] = {0x00, 0x08, 0x00, len, 0x01, 0x00}; cmd2(dev, CMD_SET_PKT_PARAMS, p, 6); }

    /* ClearIrqStatus */
    { uint8_t p[] = {0xFF, 0xFF}; cmd2(dev, CMD_CLEAR_IRQ_STATUS, p, 2); }

    /* SetTx(timeout=0 — no timeout, rely on IRQ) */
    { uint8_t p[] = {0x00, 0x00, 0x00}; cmd2(dev, CMD_SET_TX, p, 3); }

    /* Wait for TX_DONE (poll BUSY then GetIrqStatus) */
    int64_t deadline = esp_timer_get_time() + 100000; /* 100 ms */
    while (1) {
        if (esp_timer_get_time() > deadline) {
            ESP_LOGE(TAG, "TX timeout");
            return ESP_ERR_TIMEOUT;
        }
        if (gpio_get_level(SX_PIN_BUSY)) continue;
        uint16_t irq = sx1262_get_irq_status(dev);
        if (irq & IRQ_TX_DONE) break;
        if (irq & IRQ_TIMEOUT) {
            ESP_LOGE(TAG, "TX chip timeout");
            return ESP_ERR_TIMEOUT;
        }
    }

    sx1262_clear_irq(dev, 0xFFFF);
    ESP_LOGI(TAG, "TX_DONE");
    return ESP_OK;
}

esp_err_t sx1262_start_rx_continuous(sx1262_t *dev)
{
    sx1262_clear_irq(dev, 0xFFFF);
    /* SetRx(timeout=0xFFFFFF = continuous) */
    uint8_t p[] = {0xFF, 0xFF, 0xFF};
    cmd2(dev, CMD_SET_RX, p, 3);
    return ESP_OK;
}

esp_err_t sx1262_read_packet(sx1262_t *dev, uint8_t *buf, uint8_t *out_len)
{
    /* GetRxBufferStatus → payload_len, rx_buffer_ptr */
    wait_busy();
    uint8_t tx[4] = {CMD_GET_RX_BUFFER_STATUS, 0x00, 0x00, 0x00};
    uint8_t rx[4] = {0};
    spi_transaction_t t = { .length = 32, .tx_buffer = tx, .rx_buffer = rx };
    spi_device_polling_transmit(dev->spi, &t);
    uint8_t payload_len = rx[2];
    uint8_t buf_ptr     = rx[3];
    *out_len = payload_len;

    /* ReadBuffer(offset=buf_ptr) */
    wait_busy();
    uint8_t tx2[2 + payload_len];
    tx2[0] = CMD_READ_BUFFER;
    tx2[1] = buf_ptr;
    memset(&tx2[2], 0x00, payload_len);
    uint8_t rx2[2 + payload_len];
    spi_transaction_t t2 = {
        .length = (2 + payload_len) * 8,
        .tx_buffer = tx2, .rx_buffer = rx2,
    };
    spi_device_polling_transmit(dev->spi, &t2);
    /* rx2[0]=status, rx2[1]=NOP status, rx2[2..] = payload */
    memcpy(buf, &rx2[2], payload_len);
    return ESP_OK;
}

esp_err_t sx1262_get_packet_status(sx1262_t *dev, int8_t *rssi_dbm, int8_t *snr_x4)
{
    wait_busy();
    uint8_t tx[5] = {CMD_GET_PKT_STATUS, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx[5] = {0};
    spi_transaction_t t = { .length = 40, .tx_buffer = tx, .rx_buffer = rx };
    spi_device_polling_transmit(dev->spi, &t);
    /* rx[1]=status, rx[2]=rssiPkt, rx[3]=snrPkt, rx[4]=signalRssiPkt */
    *rssi_dbm = -(int8_t)((uint8_t)rx[2] / 2);
    *snr_x4   = (int8_t)rx[3];
    return ESP_OK;
}

uint16_t sx1262_get_irq_status(sx1262_t *dev)
{
    wait_busy();
    uint8_t tx[4] = {CMD_GET_IRQ_STATUS, 0x00, 0x00, 0x00};
    uint8_t rx[4] = {0};
    spi_transaction_t t = { .length = 32, .tx_buffer = tx, .rx_buffer = rx };
    spi_device_polling_transmit(dev->spi, &t);
    return ((uint16_t)rx[2] << 8) | rx[3];
}

void sx1262_clear_irq(sx1262_t *dev, uint16_t mask)
{
    wait_busy();
    uint8_t p[] = {(mask >> 8) & 0xFF, mask & 0xFF};
    cmd2(dev, CMD_CLEAR_IRQ_STATUS, p, 2);
}

void sx1262_send_sleep_cmd(sx1262_t *dev)
{
    /* SetSleep(sleepConfig=0x00: cold start on wakeup) */
    uint8_t p[] = {0x00};
    cmd2(dev, CMD_SET_SLEEP, p, 1);
}
