# LoRa Environmental Sensor Network — Functional Specification Document (FSD)

**Version**: 1.0  
**Date**: 2026-05-03  
**Status**: Draft  

---

## 1. System Overview

### 1.1 Purpose

A two-node wireless environmental monitoring network using LoRa at 868 MHz. One node (sensor node) acquires air quality, temperature, humidity, and atmospheric pressure readings, packs them into a compact binary payload, and transmits every 30 seconds. The other node (gateway) receives the payload, decodes it, and renders all sensor values along with link quality metrics on a 128×64 OLED display.

### 1.2 Problem Statement

Wired sensor networks are impractical in distributed or outdoor environments. LoRa provides long-range, low-power wireless connectivity suitable for battery-operated sensing nodes without requiring WiFi or cellular infrastructure. Both nodes run ESP-IDF on XIAO ESP32-S3R8V + Wio SX1262 hardware.

### 1.3 Users / Stakeholders

- **Developer / operator**: Programs, flashes, and verifies both nodes; conducts range tests.

### 1.4 Goals

- Sensor node reads all three sensor types reliably and transmits binary-packed data.
- Gateway decodes and displays live readings plus RSSI/SNR on every received packet.
- Sensor node enters deep sleep between transmissions to minimise power consumption.
- Both nodes are implemented in ESP-IDF (no Arduino framework).

### 1.5 Non-Goals

- No cloud or MQTT uplink.
- No OTA firmware updates.
- No encryption or authentication of LoRa packets.
- No multi-node routing; point-to-point link only.

### 1.6 High-Level System Flow

```
[Sensor Node]
  RTC Timer wakes ESP32-S3
  → Read MQ-135 (ADC), DHT22 (1-wire), BMP280 (I2C)
  → Pack 12-byte binary payload
  → SX1262 transmit at 868 MHz
  → Enter deep sleep (target ~29 s)

[Gateway]
  SX1262 listening continuously
  → Receive packet
  → Read RSSI, SNR
  → Decode payload
  → Update OLED display
  → Return to RX mode
```

---

## 2. System Architecture

### 2.1 Logical Architecture

```
┌──────────────────────────────────────────┐   LoRa 868 MHz   ┌────────────────────────────────────────────┐
│            SENSOR NODE                   │ ───────────────► │               GATEWAY                      │
│                                          │                   │                                            │
│  [MQ-135 ADC]  [DHT22]  [BMP280 I2C]    │                   │  [SX1262 RX]                               │
│        │          │         │            │                   │      │                                     │
│        └──────────┴─────────┘            │                   │      ▼                                     │
│               [Payload Encoder]          │                   │  [Payload Decoder]   [RSSI / SNR reader]   │
│                     │                    │                   │        │                    │               │
│               [SX1262 TX]                │                   │        └────────────────────┘               │
│                     │                    │                   │                    │                        │
│            [Deep Sleep / RTC]            │                   │           [OLED Display Driver]             │
└──────────────────────────────────────────┘                   └────────────────────────────────────────────┘
```

### 2.2 Hardware / Platform Architecture

#### Sensor Node

| Component | Part | Interface | GPIO |
|-----------|------|-----------|------|
| MCU | XIAO ESP32-S3R8V | — | — |
| LoRa radio | Wio SX1262 | SPI | GPIO7/8/9/41/42/40/39/38 |
| Air quality sensor | MQ-135 | ADC (12-bit) | GPIO1 |
| Temp/humidity sensor | DHT22 | Single-wire | GPIO44 |
| Pressure/temp sensor | BMP280 | I2C | SDA GPIO5, SCL GPIO6 |
| Power | USB or LiPo (assumed) | — | — |

#### Gateway

| Component | Part | Interface | GPIO |
|-----------|------|-----------|------|
| MCU | XIAO ESP32-S3R8V | — | — |
| LoRa radio | Wio SX1262 | SPI | GPIO7/8/9/41/42/40/39/38 |
| OLED display | 1.3" SH1106G 128×64 | I2C @0x3C | SDA, SCL (assumed) |
| Power | USB | — | — |

#### SX1262 Pin Mapping (Identical for Both Nodes)

| XIAO ESP32-S3 GPIO | Wio SX1262 Signal |
|--------------------|-------------------|
| GPIO42 | SX1262_RST |
| GPIO41 | SX1262_SPI_NSS (CS) |
| GPIO40 | SX1262_BUSY |
| GPIO39 | SX1262_DIO1 (IRQ) |
| GPIO38 | SX1262_DIO2 / SX1262_SF_SW1 |
| GPIO9 | SX1262_SPI_MOSI |
| GPIO8 | SX1262_SPI_MISO |
| GPIO7 | SX1262_SPI_SCK |

> **Note**: GPIO38 is shared between DIO2 and SF_SW1 on the Wio SX1262 module. The SX1262 DIO2 RF switch control must be enabled in driver configuration to manage the antenna switch correctly.

### 2.3 Software Architecture

#### Sensor Node Tasks / Modules

| Module | Responsibility |
|--------|----------------|
| `main.c` | Boot, sensor init, read cycle, transmit, deep sleep entry |
| `sx1262_driver` | SPI driver: init, TX, IRQ handling, DIO2 RF switch |
| `mq135_driver` | ADC one-shot read, raw value return |
| `dht22_driver` | Single-wire protocol, temperature + humidity decode |
| `bmp280_driver` | I2C read, pressure + temperature return |
| `payload_encoder` | Pack sensor readings into `lora_payload_t` struct |
| `power_manager` | Configure RTC wakeup, enter `esp_deep_sleep_start()` |

#### Gateway Tasks / Modules

| Module | Responsibility |
|--------|----------------|
| `main.c` | Boot, init, continuous RX loop |
| `sx1262_driver` | SPI driver: init, RX, IRQ handling, DIO2 RF switch |
| `payload_decoder` | Unpack `lora_payload_t`, extract all fields |
| `oled_driver` | SH1106G I2C driver via `esp-idf-lib` or direct I2C HAL |
| `display_manager` | Format and render sensor values + link quality on OLED |

#### Boot Sequence — Sensor Node

```
Power-on / RTC wakeup
  → esp_sleep_get_wakeup_cause() — log wake reason
  → Initialise SPI (SX1262)
  → Initialise I2C (BMP280)
  → Initialise ADC (MQ-135)
  → Initialise DHT22 GPIO
  → Read all sensors
  → Encode payload
  → SX1262 transmit (blocking, await TX_DONE IRQ)
  → esp_deep_sleep_start() with RTC timer = (30s − elapsed_ms)
```

#### Boot Sequence — Gateway

```
Power-on
  → Initialise SPI (SX1262)
  → Initialise I2C (SH1106G)
  → Display splash screen
  → SX1262 enter continuous RX
  → Event loop: await DIO1 IRQ (RX_DONE / RX_TIMEOUT / CRC_ERR)
    → On RX_DONE: read packet + RSSI/SNR → decode → update display
    → On error: log, return to RX
```

---

## 3. Implementation Phases

### 3.1 Phase 1 — Hardware Bring-Up & LoRa Link

**Scope**: Establish basic SX1262 communication on both nodes; verify LoRa link with a test payload.

**Deliverables**:
- SX1262 SPI driver initialises without error on both nodes.
- Sensor node transmits a fixed test packet; gateway receives it and prints to serial.
- RSSI and SNR are read and logged from received packet.
- SH1106G OLED initialises and renders static text on gateway.

**Exit Criteria**:
- Serial log shows `TX_DONE` on sensor node and `RX_DONE` with valid RSSI/SNR on gateway for ≥ 10 consecutive packets at bench distance.
- OLED shows static text without artefacts.

**Dependencies**: None.

---

### 3.2 Phase 2 — Sensors, Payload, Display, and Deep Sleep

**Scope**: All three sensors operational; binary payload encode/decode; OLED shows live data; deep sleep on sensor node.

**Deliverables**:
- MQ-135 raw ADC value acquired per cycle.
- DHT22 temperature and humidity acquired per cycle.
- BMP280 pressure and temperature acquired per cycle.
- 12-byte `lora_payload_t` correctly encodes all readings.
- Gateway decodes payload and displays all values plus RSSI/SNR on OLED.
- Sensor node enters deep sleep between cycles; wakes on RTC timer.

**Exit Criteria**:
- OLED updates on each received packet with plausible sensor values.
- Deep sleep current measurably lower than active current (verified with multimeter or USB power meter).
- 30-minute unattended run shows continuous display updates with no hangs.

**Dependencies**: Phase 1 complete.

---

### 3.3 Phase 3 — Range Test & Link Quality Validation

**Scope**: Outdoor range test; LoRa parameter tuning; documented maximum stable range.

**Deliverables**:
- Documented test log: distance, RSSI, SNR at multiple distances.
- Maximum stable link distance identified (≥ 5 consecutive packets received).
- Optional: SF/BW/power parameter sweep to characterise range vs. airtime trade-off.

**Exit Criteria**:
- Range test log produced with at least 5 measurement points.
- Minimum stable link range documented.

**Dependencies**: Phase 2 complete.

---

## 4. Functional Requirements

### 4.1 Functional Requirements (FR)

#### Sensor Node — Sensing

- **FR-1.1** [Must]: The sensor node shall read the MQ-135 raw ADC value (12-bit, 0–4095) on GPIO1 during each wake cycle.
- **FR-1.2** [Must]: The sensor node shall read temperature (°C) and relative humidity (%) from the DHT22 on GPIO44 using the single-wire protocol.
- **FR-1.3** [Must]: The sensor node shall read barometric pressure (hPa) and temperature (°C) from the BMP280 over I2C (SDA GPIO5, SCL GPIO6).
- **FR-1.4** [Must]: The sensor node shall detect and flag individual sensor read failures using the `flags` byte in the payload without aborting the transmission cycle.

#### Sensor Node — Payload & Transmission

- **FR-1.5** [Must]: The sensor node shall encode all sensor readings into a 12-byte packed binary payload conforming to the `lora_payload_t` structure defined in Section 6.3.
- **FR-1.6** [Must]: The sensor node shall transmit the payload over LoRa at 868 MHz using the SX1262 radio.
- **FR-1.7** [Must]: The transmission cycle shall repeat at a nominal interval of 30 seconds.
- **FR-1.8** [Must]: The sensor node shall await the TX_DONE IRQ from DIO1 before entering deep sleep, ensuring complete transmission.
- **FR-1.9** [Should]: The sensor node shall include a rolling sequence number (uint8, wraps at 255) in each payload to allow the gateway to detect packet loss.

#### Sensor Node — Power Management

- **FR-1.10** [Must]: The sensor node shall enter ESP32-S3 deep sleep between transmissions.
- **FR-1.11** [Must]: The RTC timer shall be configured to wake the sensor node at the target 30-second interval, adjusted for time spent in the active cycle.
- **FR-1.12** [Should]: The sensor node shall power-gate or tri-state the SX1262 CS line before entering deep sleep (assumed: SX1262_NSS driven high).

#### Gateway — Reception

- **FR-2.1** [Must]: The gateway shall place the SX1262 in continuous RX mode and remain in that state between received packets.
- **FR-2.2** [Must]: The gateway shall read RSSI (dBm) and SNR (dB) for every successfully received packet.
- **FR-2.3** [Must]: The gateway shall discard packets with CRC errors and log the event to the serial console.
- **FR-2.4** [Should]: The gateway shall log each received packet to the serial console (sequence number, all decoded fields, RSSI, SNR).

#### Gateway — Decoding & Display

- **FR-2.5** [Must]: The gateway shall decode the received binary payload according to the `lora_payload_t` structure.
- **FR-2.6** [Must]: The gateway shall display the following on the SH1106G OLED after each received packet:
  - Air quality (MQ-135 raw ADC value)
  - Temperature from DHT22 (°C, one decimal place)
  - Relative humidity from DHT22 (%, one decimal place)
  - Barometric pressure from BMP280 (hPa, integer)
  - RSSI (dBm)
  - SNR (dB)
- **FR-2.7** [Should]: The gateway shall indicate on the OLED when no packet has been received for > 60 seconds (e.g., "NO SIGNAL" status line).
- **FR-2.8** [May]: The gateway shall display the packet sequence number to aid in loss detection.

#### System-Wide

- **FR-3.1** [Must]: Both nodes shall be implemented using ESP-IDF (no Arduino framework or Arduino libraries).
- **FR-3.2** [Must]: The LoRa link shall operate at 868 MHz center frequency.
- **FR-3.3** [Must]: The LoRa payload shall be binary-packed (not JSON or ASCII) to minimise airtime.
- **FR-3.4** [Must]: SX1262 DIO2 shall be configured as RF switch control to correctly drive the Wio SX1262 antenna switch via GPIO38.

### 4.2 Non-Functional Requirements (NFR)

- **NFR-1.1** [Must]: The sensor node deep sleep current shall be measurably lower than the active TX current (verified by observation; target: deep sleep ≪ active).
- **NFR-1.2** [Must]: The LoRa payload size shall not exceed 20 bytes.
- **NFR-1.3** [Should]: The active cycle (wake → sensors → TX → sleep entry) shall complete within 5 seconds.
- **NFR-2.1** [Should]: The gateway OLED shall refresh within 500 ms of RX_DONE interrupt assertion.
- **NFR-2.2** [Should]: The link shall maintain ≥ 5 consecutive packets received at ≥ 100 m in open-field conditions.
- **NFR-3.1** [Must]: Both firmware builds shall compile without warnings under ESP-IDF v5.x.

### 4.3 Constraints

- LoRa frequency restricted to 868 MHz band (EU regional plan); TX power must not exceed local regulatory limits (typically +14 dBm ERP for EU 868 MHz).
- GPIO38 is shared between SX1262_DIO2 and SX1262_SF_SW1 on the Wio SX1262 module; the driver must configure DIO2 as RF switch output and not treat it as a generic IRQ line.
- ESP32-S3R8V has 8 MB Octal PSRAM and 8 MB Octal Flash; no special PSRAM configuration required for this project (assumed: PSRAM not used).
- DHT22 requires a minimum 2-second inter-read interval; the driver must enforce this.
- BMP280 requires calibration coefficients to be read from OTP registers at startup; these must be preserved across the boot cycle (re-read on every wake since deep sleep discards RAM).

---

## 5. Risks, Assumptions & Dependencies

| # | Risk / Assumption | Likelihood | Impact | Mitigation |
|---|------------------|------------|--------|------------|
| R-1 | GPIO38 shared DIO2/SF_SW1 mis-configuration causes TX failure or wrong antenna state | Medium | High | Configure SX1262 via `SetDio2AsRfSwitchCtrl` command; verify with logic analyser on first bring-up |
| R-2 | DHT22 read fails during the same SPI transaction window as SX1262 (shared I/O timing) | Low | Medium | Read all sensors before initialising SX1262 TX; use sufficient pull-up on DHT22 data line |
| R-3 | MQ-135 warm-up time not met during short wake cycle | Medium | Medium | Flag stale/cold reading in `flags` byte; note 24-48h burn-in requirement for calibrated readings (assumed: raw ADC only, no calibration in Phase 2) |
| R-4 | Deep sleep RTC timer drift causes progressive 30s interval skew | Low | Low | Subtract active cycle duration before arming RTC timer |
| R-5 | SH1106G I2C address conflicts with BMP280 on sensor node shared I2C bus | N/A | — | SH1106G is only on the gateway; BMP280 is only on the sensor node — no conflict |
| R-6 | ESP-IDF SPI bus driver conflicts between SX1262 and other peripherals | Low | Medium | Dedicate SPI bus 2 (FSPI) to SX1262 on both nodes |
| A-1 | (assumed) LoRa parameters: SF7, BW 125 kHz, CR 4/5, preamble 8 symbols, CRC enabled | — | — | May be adjusted in Phase 3 range test |
| A-2 | (assumed) TX power: +14 dBm (within EU 868 MHz ISM band limit) | — | — | — |
| A-3 | (assumed) OLED I2C uses default ESP32-S3 I2C pins on gateway | — | — | Confirm at wiring stage |
| A-4 | (assumed) MQ-135 output is raw ADC only; no gas concentration calculation in firmware | — | — | Concentration formula can be added in a future phase |
| A-5 | (assumed) No LoRa packet acknowledgement or retransmit; best-effort delivery | — | — | — |

---

## 6. Interface Specifications

### 6.1 External Interfaces

| Interface | Node | Protocol | Physical | Direction |
|-----------|------|----------|----------|-----------|
| LoRa RF link | Both | SX1262 LoRa | 868 MHz antenna | Sensor→Gateway |
| Serial monitor | Both | USB CDC (ESP-IDF `CONFIG_ESP_CONSOLE_USB_CDC=y`) | ttyACM | Bidirectional (debug) |
| OLED display | Gateway only | I2C @0x3C | SDA/SCL | MCU→display |

### 6.2 Internal Interfaces

| Interface | Module A | Module B | Protocol | Notes |
|-----------|----------|----------|----------|-------|
| SX1262 control | `sx1262_driver` | ESP-IDF SPI master | SPI (FSPI, 8 MHz) | Full-duplex, CS=GPIO41 |
| BMP280 control | `bmp280_driver` | ESP-IDF I2C master | I2C (400 kHz) | Addr 0x76 or 0x77 (assumed 0x76) |
| DHT22 control | `dht22_driver` | GPIO + RMT or bit-bang | Single-wire | GPIO44, 5kΩ pull-up required |
| MQ-135 control | `mq135_driver` | ESP-IDF ADC oneshot | ADC1 CH0 (GPIO1) | 12-bit, attenuation 11dB |
| OLED control | `oled_driver` | ESP-IDF I2C master | I2C (400 kHz) | Addr 0x3C |

### 6.3 Data Models / Schemas

#### `lora_payload_t` — 12-byte packed binary payload

```c
typedef struct __attribute__((packed)) {
    uint8_t  node_id;        // Node identifier (0x01 for sensor node 1)
    uint8_t  seq;            // Rolling sequence counter (0–255, wraps)
    uint16_t aqi_raw;        // MQ-135 ADC raw value (0–4095), big-endian
    int16_t  temp_dht_x10;   // DHT22 temperature × 10 (e.g. 235 = 23.5 °C)
    uint16_t humi_dht_x10;   // DHT22 humidity × 10 (e.g. 652 = 65.2 %)
    uint16_t pres_bmp_hpa;   // BMP280 pressure in whole hPa (e.g. 1013)
    int16_t  temp_bmp_x10;   // BMP280 temperature × 10 (e.g. 237 = 23.7 °C)
    uint8_t  flags;          // Sensor status bitmask (see below)
} lora_payload_t;            // sizeof = 12 bytes
```

**`flags` bitmask**:

| Bit | Mask | Meaning |
|-----|------|---------|
| 0 | `0x01` | MQ-135 read failed |
| 1 | `0x02` | DHT22 read failed |
| 2 | `0x04` | BMP280 read failed |
| 3–7 | reserved | 0 |

> All multi-byte fields are little-endian (native ESP32 byte order) unless explicitly stated.

#### OLED Display Layout (128×64, 6 lines of 8px font)

```
┌────────────────────────┐
│ AQI: 1842              │  line 1 — MQ-135 raw
│ T: 23.5C  H: 65.2%     │  line 2 — DHT22
│ P: 1013 hPa            │  line 3 — BMP280 pressure
│ Tb: 23.7C              │  line 4 — BMP280 temp
│ RSSI: -87 dBm          │  line 5
│ SNR:   9.0 dB  #42     │  line 6 — SNR + sequence
└────────────────────────┘
```

### 6.4 Commands / Opcodes

SX1262 is controlled via the Semtech SX1262 SPI command set. Key commands used:

| Command | Opcode | Usage |
|---------|--------|-------|
| `SetStandby` | 0x80 | Reset to standby XOSC/RC |
| `SetPacketType` | 0x8A | Select LoRa mode (0x01) |
| `SetRfFrequency` | 0x86 | Set 868.0 MHz |
| `SetTxParams` | 0x8E | Set +14 dBm, ramp 200 µs |
| `SetModulationParams` | 0x8B | SF7, BW125, CR4/5 |
| `SetPacketParams` | 0x8C | Preamble 8, explicit header, CRC on |
| `SetDio2AsRfSwitchCtrl` | 0x9D | Enable DIO2 as antenna switch driver |
| `SetTx` | 0x83 | Start TX with timeout |
| `SetRx` | 0x82 | Start RX with timeout (0xFFFFFF = continuous) |
| `GetPacketStatus` | 0x14 | Read RSSI, SNR after RX_DONE |
| `ReadBuffer` | 0x1E | Read received payload bytes |
| `WriteBuffer` | 0x0E | Write TX payload bytes |
| `ClearIrqStatus` | 0x02 | Acknowledge IRQ flags |
| `GetIrqStatus` | 0x12 | Read pending IRQ flags |

---

## 7. Operational Procedures

### 7.1 Initial Firmware Build and Flash

Both sensor node and gateway are flashed individually via local USB using ESP-IDF.

```bash
# 1. Set up ESP-IDF environment
source ~/esp/esp-idf/export.sh

# 2. Build sensor node firmware
cd ~/dev/firmware/lora_sensor_network/sensor_node
idf.py build

# 3. Flash sensor node (device on ttyACM0 / SLOT1)
idf.py -p /dev/ttyACM0 flash

# 4. Monitor sensor node
idf.py -p /dev/ttyACM0 monitor

# 5. Build and flash gateway (repeat with gateway project, re-plug device)
cd ~/dev/firmware/lora_sensor_network/gateway
idf.py build
idf.py -p /dev/ttyACM0 flash
```

> Use `CONFIG_ESP_CONSOLE_USB_CDC=y` in `sdkconfig.defaults` for both nodes (native USB, VID `303a:1001`).  
> Do **not** use `hard-reset` or `watchdog-reset` after flashing. Use `no-reset`, then trigger a soft reset via the workbench portal: `curl -X POST localhost:8080/api/serial/reset -H 'Content-Type: application/json' -d '{"slot":"SLOT1"}'`.

### 7.2 Normal Operation

1. Flash gateway first; it begins listening on power-up.
2. Flash or power sensor node; it begins transmitting immediately on first boot, then sleeps.
3. OLED on gateway updates on each received packet (~30 s interval).
4. Monitor gateway serial output to verify decoded values and link quality.

### 7.3 Sensor Node Power Cycle / Reset

- Cycle USB power or press the XIAO reset button.
- Sequence counter resets to 0; gateway detects discontinuity in `seq`.

### 7.4 Recovery Procedures

| Condition | Action |
|-----------|--------|
| Sensor node stuck (no TX) | Power-cycle node; check serial log for sensor init error |
| Gateway OLED blank | Power-cycle gateway; verify I2C address (0x3C) with I2C scanner sketch |
| SX1262 SPI init failure | Confirm GPIO pin mapping; check for solder bridges on Wio SX1262 header |
| DHT22 always fails | Verify 5 kΩ pull-up resistor on GPIO44; minimum 2 s between reads |
| BMP280 not found | Check I2C address (SDO pin: GND→0x76, VCC→0x77); confirm SDA/SCL |
| No packets received at gateway | Confirm 868 MHz antenna attached; verify DIO2 RF switch config |

---

## 8. Verification & Validation

### 8.1 Phase 1 Verification

| Test ID | Feature | Procedure | Success Criteria |
|---------|---------|-----------|-----------------|
| TC-P1-01 | SX1262 SPI init — sensor node | Flash sensor node; check serial log | Log shows `SX1262 init OK`, no SPI errors |
| TC-P1-02 | SX1262 SPI init — gateway | Flash gateway; check serial log | Log shows `SX1262 init OK`, no SPI errors |
| TC-P1-03 | LoRa TX/RX bench test | Sensor node TX fixed packet; gateway in RX | Gateway serial shows `RX_DONE`, prints raw RSSI and SNR |
| TC-P1-04 | 10-packet bench stability | Run 10 consecutive TX/RX cycles | All 10 packets received; no CRC errors |
| TC-P1-05 | OLED init | Power gateway; observe display | OLED shows splash screen text, no pixel artefacts |

### 8.2 Phase 2 Verification

| Test ID | Feature | Procedure | Success Criteria |
|---------|---------|-----------|-----------------|
| TC-P2-01 | MQ-135 read | Serial log after sensor node boot | ADC value 0–4095, consistent with ambient air; `flags` bit 0 = 0 |
| TC-P2-02 | DHT22 read | Serial log after sensor node boot | Temperature in −40 to +80 °C range, humidity 0–100%; `flags` bit 1 = 0 |
| TC-P2-03 | BMP280 read | Serial log after sensor node boot | Pressure 950–1050 hPa, temperature plausible; `flags` bit 2 = 0 |
| TC-P2-04 | Payload size | Inspect encoded payload | `sizeof(lora_payload_t)` = 12 bytes confirmed in log |
| TC-P2-05 | Payload encode/decode round-trip | Transmit and receive one packet | Gateway decodes all fields to match sensor node log values (±1 LSB) |
| TC-P2-06 | OLED live data display | Gateway receives packet | OLED updates all 6 lines with plausible values within 500 ms of RX_DONE |
| TC-P2-07 | RSSI/SNR display | Gateway receives packet at bench | OLED shows RSSI and SNR; values consistent with bench proximity (RSSI > −80 dBm expected) |
| TC-P2-08 | Deep sleep transition | Monitor sensor node current or USB power meter | Current drops to deep sleep level after TX_DONE; rises again ~30 s later |
| TC-P2-09 | 30-minute unattended run | Leave both nodes running | OLED updates continuously; no hang or reset observed; `seq` increments as expected |
| TC-P2-10 | Sensor failure flag | Disconnect BMP280; check payload | `flags` bit 2 set; other fields unaffected; TX still occurs |

### 8.3 Acceptance Tests

| Test ID | Feature | Procedure | Success Criteria |
|---------|---------|-----------|-----------------|
| TC-ACC-01 | End-to-end sensor-to-display | Full system powered, all sensors connected | OLED shows live, plausible AQI, temp, humidity, pressure values updating every ~30 s |
| TC-ACC-02 | Deep sleep power measurable | USB power meter on sensor node | Active peak visible ~30 s; sustained low current in sleep (order-of-magnitude reduction) |
| TC-ACC-03 | Range test — 10 m | Move nodes 10 m apart indoors | ≥ 5 consecutive packets received; RSSI and SNR logged |
| TC-ACC-04 | Range test — 50 m | Move nodes 50 m apart (outdoors or long corridor) | ≥ 5 consecutive packets received; RSSI and SNR logged |
| TC-ACC-05 | Range test — 100 m | Move nodes 100 m apart open field | ≥ 5 consecutive packets received; link declared stable |
| TC-ACC-06 | No-signal indicator | Power off sensor node for 90 s | Gateway OLED shows "NO SIGNAL" or equivalent status message |
| TC-ACC-07 | Packet loss detection | Manually drop some packets (power off/on sensor node mid-run) | Gateway serial log shows `seq` gap; loss noted |

### 8.4 Traceability Matrix

| Requirement | Priority | Test Case(s) | Status |
|-------------|----------|--------------|--------|
| FR-1.1 (MQ-135 read) | Must | TC-P2-01 | Covered |
| FR-1.2 (DHT22 read) | Must | TC-P2-02 | Covered |
| FR-1.3 (BMP280 read) | Must | TC-P2-03 | Covered |
| FR-1.4 (sensor failure flags) | Must | TC-P2-10 | Covered |
| FR-1.5 (12-byte payload encode) | Must | TC-P2-04, TC-P2-05 | Covered |
| FR-1.6 (LoRa TX at 868 MHz) | Must | TC-P1-03, TC-P1-04 | Covered |
| FR-1.7 (30 s repeat interval) | Must | TC-P2-09, TC-ACC-01 | Covered |
| FR-1.8 (await TX_DONE before sleep) | Must | TC-P2-08 | Covered |
| FR-1.9 (sequence number) | Should | TC-ACC-07 | Covered |
| FR-1.10 (deep sleep) | Must | TC-P2-08, TC-ACC-02 | Covered |
| FR-1.11 (RTC wakeup timer) | Must | TC-P2-08, TC-P2-09 | Covered |
| FR-1.12 (SX1262 CS high before sleep) | Should | — | GAP |
| FR-2.1 (continuous RX) | Must | TC-P1-03, TC-P2-09 | Covered |
| FR-2.2 (RSSI + SNR read) | Must | TC-P1-03, TC-P2-07 | Covered |
| FR-2.3 (CRC error discard) | Must | — | GAP |
| FR-2.4 (serial log received packet) | Should | TC-P2-05 | Covered |
| FR-2.5 (payload decode) | Must | TC-P2-05 | Covered |
| FR-2.6 (OLED display all fields) | Must | TC-P2-06, TC-P2-07, TC-ACC-01 | Covered |
| FR-2.7 (no-signal indicator) | Should | TC-ACC-06 | Covered |
| FR-2.8 (sequence on OLED) | May | TC-ACC-07 | Covered |
| FR-3.1 (ESP-IDF only) | Must | TC-P1-01, TC-P1-02 | Covered |
| FR-3.2 (868 MHz) | Must | TC-P1-03 | Covered |
| FR-3.3 (binary payload) | Must | TC-P2-04 | Covered |
| FR-3.4 (DIO2 as RF switch) | Must | TC-P1-03 | Covered |
| NFR-1.1 (deep sleep current lower) | Must | TC-ACC-02 | Covered |
| NFR-1.2 (payload ≤ 20 bytes) | Must | TC-P2-04 | Covered |
| NFR-1.3 (active cycle ≤ 5 s) | Should | TC-P2-09 | Covered |
| NFR-2.1 (OLED refresh ≤ 500 ms) | Should | TC-P2-06 | Covered |
| NFR-2.2 (link stable at ≥ 100 m) | Should | TC-ACC-05 | Covered |
| NFR-3.1 (zero compile warnings) | Must | TC-P1-01, TC-P1-02 | Covered |

---

## 9. Troubleshooting Guide

| Symptom | Likely Cause | Diagnostic Steps | Corrective Action |
|---------|-------------|-----------------|-------------------|
| SX1262 init fails (SPI timeout) | GPIO mis-wiring or NSS not driven | Scope MOSI/SCK during init; verify GPIO41 = NSS | Re-check wiring against pin table; ensure SPI bus speed ≤ 8 MHz |
| TX_DONE IRQ never fires | DIO2 RF switch not configured; DIO1 IRQ mask wrong | Enable `LORA_IDF_TX_DONE` in IRQ mask; verify `SetDio2AsRfSwitchCtrl` | Send `SetDio2AsRfSwitchCtrl(1)` command; confirm DIO1 GPIO39 interrupt fires |
| Gateway receives nothing | Antenna disconnected; wrong frequency | Attach antenna; confirm `SetRfFrequency` = 868000000 Hz | Verify frequency constant in both firmware builds |
| CRC errors on gateway | LoRa parameter mismatch (SF/BW/CR) | Log raw SX1262 `GetIrqStatus` on gateway | Ensure both nodes use identical SF, BW, CR, preamble length |
| DHT22 always returns error | Missing pull-up or 2 s rule violated | Measure GPIO44 idle level (must be ~3.3 V) | Add 5 kΩ pull-up to 3.3 V; enforce 2 s minimum between reads |
| BMP280 not found on I2C | SDO pin state wrong | Run I2C scanner; check address 0x76 and 0x77 | Connect SDO to GND for 0x76; re-scan |
| OLED blank / garbage | I2C address wrong; missing init sequence | Run I2C scanner on gateway; verify 0x3C present | Confirm SH1106G address; re-run init sequence |
| Sensor node never sleeps | TX_DONE await blocks indefinitely | Monitor GPIO39 (DIO1); check IRQ mask | Ensure IRQ mask includes `IRQ_TX_DONE`; add timeout guard |
| Deep sleep current not reduced | Peripheral holding SPI lines active | Scope current vs time; check GPIO states in sleep | Configure SPI bus to release GPIOs before deep sleep entry |
| MQ-135 reads near 0 | ADC attenuation wrong; sensor not powered | Verify ADC1 CH0 config: `ADC_ATTEN_DB_11`; check 5V supply to MQ-135 heater | Set correct attenuation; confirm sensor heater energised for ≥ 2 min |

---

## 10. Appendix

### 10.1 LoRa RF Parameters (Phase 1 Default)

| Parameter | Value |
|-----------|-------|
| Center frequency | 868.0 MHz |
| Spreading factor | SF7 |
| Bandwidth | 125 kHz |
| Coding rate | 4/5 |
| Preamble length | 8 symbols |
| Header mode | Explicit |
| CRC | Enabled |
| TX power | +14 dBm |
| Ramp time | 200 µs |
| Time on air (12-byte payload) | ~27 ms (SF7, BW125, CR4/5) |

### 10.2 Payload Constants

| Constant | Value | Notes |
|----------|-------|-------|
| `PAYLOAD_SIZE` | 12 | bytes |
| `NODE_ID_SENSOR_1` | 0x01 | Sensor node identifier |
| `TEMP_SCALE` | 10 | Divide int field by 10 to get °C |
| `HUMI_SCALE` | 10 | Divide int field by 10 to get % |
| `FLAG_MQ135_FAIL` | 0x01 | MQ-135 sensor fault |
| `FLAG_DHT22_FAIL` | 0x02 | DHT22 sensor fault |
| `FLAG_BMP280_FAIL` | 0x04 | BMP280 sensor fault |

### 10.3 Timing Budget — Sensor Node Active Cycle

| Step | Estimated Duration |
|------|-------------------|
| Boot / ESP32-S3 wakeup overhead | ~200 ms |
| SX1262 init + configure | ~50 ms |
| BMP280 init + read | ~50 ms |
| DHT22 read | ~20 ms |
| MQ-135 ADC read | ~5 ms |
| Payload encode | < 1 ms |
| SX1262 TX (12 bytes, SF7) | ~27 ms |
| Deep sleep entry | ~5 ms |
| **Total active** | **~360 ms** |
| Sleep duration | ~29.6 s |

### 10.4 Example Serial Log — Sensor Node

```
I (350) main: Wake reason: RTC_TIMER
I (355) bmp280: P=1013 hPa, T=23.7 C
I (360) dht22: T=23.5 C, H=65.2 %
I (362) mq135: ADC raw=1842
I (363) payload: seq=42 node=01 flags=00 packed 12 bytes
I (390) sx1262: TX_DONE, entering deep sleep for 29640 ms
```

### 10.5 Example Serial Log — Gateway

```
I (412) sx1262: RX_DONE, RSSI=-87 dBm, SNR=+9.0 dB
I (413) payload: seq=42 node=01 AQI=1842 T_dht=23.5 H=65.2 P=1013 T_bmp=23.7 flags=0x00
I (415) display: OLED updated
```

### 10.6 Project Directory Structure

```
~/dev/firmware/lora_sensor_network/
├── sensor_node/
│   ├── CMakeLists.txt
│   ├── sdkconfig.defaults
│   └── main/
│       ├── main.c
│       ├── sx1262_driver.c / .h
│       ├── mq135_driver.c / .h
│       ├── dht22_driver.c / .h
│       ├── bmp280_driver.c / .h
│       ├── payload_encoder.c / .h
│       └── power_manager.c / .h
├── gateway/
│   ├── CMakeLists.txt
│   ├── sdkconfig.defaults
│   └── main/
│       ├── main.c
│       ├── sx1262_driver.c / .h
│       ├── payload_decoder.c / .h
│       ├── oled_driver.c / .h
│       └── display_manager.c / .h
└── Documents/
    └── lora-sensor-network-fsd.md
```
