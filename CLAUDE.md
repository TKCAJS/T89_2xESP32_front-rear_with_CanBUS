# Honda CBR1000RR Racing Telemetry — Project Context

Two ESP32-S3 nodes + one STM32H562 sensor node communicating over CAN bus (500 Kbps, 29-bit extended frames, SN65HVD230 transceivers).
Shared CAN ID definitions live in `lib/can_ids/can_ids.h` — include this in all nodes, never use raw CAN_ID() calls in firmware.

## Build status
- **Rear node** — wired, verified. Gear positions detecting correctly. Shift relays (up/down/ign cut) working.
- **Main node** — wired, CAN working. Gear position received from rear node. Shift commands sending correctly.
- **CAN bus** — end-to-end working between both nodes.
- **Sensor node** — in design. WeAct STM32H562RGT6. STM32CubeIDE standalone project at `c:/Users/Adrian/Documents/PlatformIO/Projects/Sensor_node/`.
- **Next** — remove PCF8575/old gear sensor code from main node. Tune shift durations. Scaffold sensor node CubeIDE project.

## can_ids.h synchronisation
`lib/can_ids/can_ids.h` in this repo is the **source of truth**. The sensor node (STM32CubeIDE) keeps a copy at `Sensor_node/Core/Inc/can_ids.h`. Whenever the header changes in either project, manually copy it to the other and rebuild both. The two copies must stay identical.

---

## Rear Node (`src/rear_node/`) — ESP32-S3

| GPIO | Function |
|------|----------|
| 4    | CAN TX (SN65HVD230) |
| 5    | CAN RX (SN65HVD230) |
| 6    | Gear 1 |
| 7    | Gear N |
| 8    | Gear 6 |
| 9    | Display BL (backlight) |
| 10   | Display CS |
| 11   | Display DC |
| 12   | Display RST |
| 13   | Display MOSI (SDA) |
| 14   | Display SCLK (SCL) |
| 15   | Gear 2 |
| 16   | Gear 3 |
| 17   | Gear 4 |
| 18   | Gear 5 |
| 40   | Relay upshift (active HIGH) |
| 41   | Relay downshift (active HIGH) |
| 42   | Relay ign cut (active HIGH) |
| 48   | NeoPixel |

Gear sensor: 7-pin rotary switch, active LOW, internal pullup.
Display: ST7789 170x320 TFT (Adafruit_ST7789).
Pins 6-8 and 15-18 form two inline sequential blocks on the connector — intentional for clean wiring.

### ESP32-S3 strapping pins to avoid
GPIO 0, 3, 45, 46 are sampled at reset. Do not use for outputs that may be driven at boot.
GPIO 9 is tied to the onboard BOOT button via pull-up resistor — avoid for any use.
GPIO 45 was originally assigned to the ign cut relay and was moved to GPIO 42 for this reason.

---

## Main Node (`src/main_node/`) — ESP32-S3

| GPIO | Function |
|------|----------|
| 5    | Hall sensor (analog) |
| 6    | Clutch servo |
| 7    | RPM input |
| 21   | WiFi toggle switch |
| 10   | Switch: Manual mode toggle (long press) |
| 11   | Switch: Neutral (auto direction by gear) |
| 12   | Switch: Shift Down |
| 13   | Switch: Shift Up |
| 15   | Clutch position (analog) |
| 16   | NeoMatrix (8x8) |
| 17   | CAN TX (SN65HVD230) |
| 18   | CAN RX (SN65HVD230) |
| 48   | NeoPixel (single) |

Gear position received from rear node via CAN.
Speed received from sensor node via CAN (CAN_SENS_SPEED).
Web server on port 80 (AP SSID: T89_Gearbox).

---

---

## Sensor Node — WeAct STM32H562RGT6 (standalone STM32CubeIDE project)

| Pin | Function |
|-----|----------|
| PB7 | FDCAN1 TX (SN65HVD230) |
| PB8 | FDCAN1 RX (SN65HVD230) |
| TBD | Oil pressure (ADC, analog) |
| TBD | Water temperature (ADC, analog) |
| TBD | Throttle position sensor (ADC, analog) |
| TBD | Speed input (TIMx input capture, square wave via optocoupler) |
| TBD | Fuel level 1 (ADC, analog) |
| TBD | Fuel level 2 (ADC, analog) |
| TBD | PWM water pump (TIMx PWM output) |

### STM32H562 occupied/reserved pins (WeAct board)
| Pin(s) | Used for |
|--------|----------|
| PB2 | Onboard LED |
| PC13 | Onboard button |
| PA9/PA10 | Debug UART (USART1, 115200) |
| PA4-PA7 | SPI1 |
| PA11/PA12 | USB (CDC virtual COM) |
| PC8-PC12, PD2, PD4 | SDMMC1 (microSD card slot) |

CAN node ID: `NODE_SENSOR (0x0A)`. Broadcasts: oil pressure, water temp, TPS, speed, fuel×2, pump duty. Accepts pump duty override command.

---

## CAN frame format
All frames 8 bytes:
- Byte 0: sequence counter
- Byte 1: node status flags
- Bytes 2-7: payload (little-endian multi-byte values)

Key gear values: `GEAR_NEUTRAL=0x00`, `GEAR_1-6=0x01-0x06`, `GEAR_BETWEEN=0xFE`, `GEAR_UNKNOWN=0xFF`
