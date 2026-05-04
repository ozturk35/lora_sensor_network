# LoRa Sensor Network

ESP-IDF v6.0 firmware for a two-node LoRa sensor network built on the
[Seeed XIAO ESP32-S3](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/) +
[Wio SX1262 LoRa module](https://wiki.seeedstudio.com/wio_sx1262/).

## Overview

| Node | Role | Sensors |
|------|------|---------|
| **sensor_node** | Reads sensors → encodes 13-byte payload → LoRa TX → deep sleep 30 s | MQ-135 (AQI), DHT22 (temp/humidity), BMP280 (pressure/temp) |
| **gateway** | LoRa RX → decode payload → display on OLED | SH1106G 128×64 I2C OLED |

Both nodes run on **868 MHz**, SF7, BW125, CR4/5 — EU868 ISM band.

## Hardware

| Signal | ESP32-S3 GPIO |
|--------|--------------|
| SPI MOSI | 9 |
| SPI MISO | 8 |
| SPI SCK | 7 |
| SX1262 NSS | 41 |
| SX1262 BUSY | 40 |
| SX1262 DIO1 | 39 |
| SX1262 RST | 42 |
| I2C SDA | 5 |
| I2C SCL | 6 |
| DHT22 (sensor node) | 44 |
| MQ-135 ADC (sensor node) | GPIO1 / ADC1_CH0 |

## Payload Format

```c
typedef struct __attribute__((packed)) {
    uint8_t  node_id;        // 0x01 = sensor node 1
    uint8_t  seq;            // rolling sequence counter
    uint16_t aqi_raw;        // MQ-135 ADC 12-bit (little-endian)
    int16_t  temp_dht_x10;   // DHT22 temperature × 10 (e.g. 235 = 23.5 °C)
    uint16_t humi_dht_x10;   // DHT22 humidity × 10 (e.g. 612 = 61.2 %)
    uint16_t pres_bmp_hpa;   // BMP280 pressure in whole hPa
    int16_t  temp_bmp_x10;   // BMP280 temperature × 10
    uint8_t  flags;          // bit 0=MQ135_FAIL, bit 1=DHT22_FAIL, bit 2=BMP280_FAIL
} lora_payload_t;            // 13 bytes
```

## Project Structure

```
lora_sensor_network/
├── sensor_node/
│   ├── CMakeLists.txt
│   ├── sdkconfig.defaults
│   └── main/
│       ├── main.c
│       ├── sx1262_driver.{c,h}
│       ├── mq135_driver.{c,h}
│       ├── dht22_driver.{c,h}
│       ├── bmp280_driver.{c,h}
│       ├── payload_encoder.{c,h}
│       └── power_manager.{c,h}
├── gateway/
│   ├── CMakeLists.txt
│   ├── sdkconfig.defaults
│   └── main/
│       ├── main.c
│       ├── sx1262_driver.{c,h}
│       ├── payload_decoder.{c,h}
│       ├── oled_driver.{c,h}
│       └── display_manager.{c,h}
└── Documents/
    └── lora-sensor-network-fsd.md
```

## Build & Flash

```bash
source ~/esp/esp-idf/export.sh

# Sensor node
cd sensor_node
idf.py build
idf.py -p /dev/ttyACM0 flash monitor

# Gateway
cd ../gateway
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

> **Native USB note (XIAO ESP32-S3):** After flashing, the device does not
> auto-reset. Use `--after no-reset` with esptool and then unplug/replug the
> USB cable to boot the application.

## OLED Display Layout (gateway)

```
Page 0:  AQI:  ###
Page 1:  T:+##.#C H:##.#%
Page 2:  P: #### hPa
Page 3:  Tb:+##.#C
Page 4:  RSSI: ### dBm
Page 5:  SNR:+#.#dB ###
```

## Key Implementation Notes

- **SX1262 DIO2**: after `SetDio2AsRfSwitchCtrl(1)` the chip drives GPIO38 for
  the antenna switch. The MCU must never configure or touch GPIO38.
- **BUSY discipline**: every SPI command waits for BUSY=low (10 ms timeout)
  before issuing the next command.
- **BMP280 calibration**: re-read from OTP registers on every boot — deep sleep
  clears RAM so coefficients cannot be cached across wakes.
- **Deep sleep**: the sensor node calls `mq135_deinit()` (releases ADC unit)
  and `sx1262_send_sleep_cmd()` before entering deep sleep to minimise leakage.
