# Honda CBR1000RR Racing Telemetry — Project Context

Two ESP32-S3 nodes communicating over CAN bus (500 Kbps, 29-bit extended frames, SN65HVD230 transceivers).
Shared CAN ID definitions live in `lib/can_ids/can_ids.h` — include this in all nodes, never use raw CAN_ID() calls in firmware.

## Build status
- **Rear node** — wired, verified. Gear positions detecting correctly. Shift relays (up/down/ign cut) working.
- **Main node** — wired, CAN working. Gear position received from rear node. Shift commands sending correctly.
- **CAN bus** — end-to-end working between both nodes.
- **Next** — remove PCF8575/old gear sensor code from main node. Tune shift durations.

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
GPIO 45 was originally assigned to the ign cut relay and was moved to GPIO 42 for this reason.

---

## Main Node (`src/main_node/`) — ESP32-S3

| GPIO | Function |
|------|----------|
| 3    | PCF8575 interrupt — to be removed |
| 5    | Hall sensor (analog) |
| 6    | Clutch servo |
| 9    | WiFi toggle switch |
| 10   | Switch: Neutral Down |
| 11   | Switch: Neutral Up |
| 12   | Switch: Shift Down |
| 13   | Switch: Shift Up |
| 14   | RPM input |
| 15   | Clutch position (analog) |
| 16   | Speed (MPH) input |
| 17   | CAN TX (SN65HVD230) |
| 18   | CAN RX (SN65HVD230) |
| 21   | I2C SDA — to be removed (PCF8575 matrix display) |
| 45   | I2C SCL — to be removed (PCF8575 matrix display) |
| 48   | NeoPixel |

PCF8575 matrix display and gear sensor control will be removed from the main node.
Gear position will be handled entirely by the rear node via CAN.
Web server on port 80 (AP SSID: T89_Gearbox).

---

## CAN frame format
All frames 8 bytes:
- Byte 0: sequence counter
- Byte 1: node status flags
- Bytes 2-7: payload (little-endian multi-byte values)

Key gear values: `GEAR_NEUTRAL=0x00`, `GEAR_1-6=0x01-0x06`, `GEAR_BETWEEN=0xFE`, `GEAR_UNKNOWN=0xFF`
