# Sensor Node — STM32H562RGT6 (WeAct Studio)

Board: WeAct Studio STM32H562RGT6 (LQFP64, 250 MHz Cortex-M33, 2 MB flash, 640 KB SRAM)

---

## GPIO initialisation rules

In `MX_GPIO_Init()` (or equivalent), the order is critical:

1. **Enable all GPIO port clocks** before touching any pin.
2. **Blanket analog pass** — set every pin to `GPIO_MODE_ANALOG`, `GPIO_NOPULL`.
   This is the lowest-power, best-EMI default for all unused pins (disables Schmitt
   trigger, prevents floating inputs from drawing crossbar current).
3. **Exclude PA13 / PA14 from the blanket pass** — these are SWDIO / SWCLK.
   Masking them out preserves the SWD debug and programming interface.
   Example: `GPIOA.Pin = GPIO_PIN_All & ~(GPIO_PIN_13 | GPIO_PIN_14)`
4. **Configure active pins after the blanket pass** — oil pressure ADC, cooling pump
   PWM, FDCAN, display SPI, Dallas OneWire, etc. These calls override the analog
   setting on their specific pins only.

Ordering is mandatory: the blanket pass **must** precede any peripheral pin config,
or it will wipe pins that were just configured.

---

## Boot-window safety — cooling pump

The cooling pump is a smart (Infineon-controlled) unit; the **PB1** line is a PWM
**speed command**, not a motor / MOSFET-gate drive. Bench-measured behaviour:

- **0% duty / no PWM signal → controller FAILSAFE = 100% (full speed)**
- **~10% duty → pump off**
- **15–90% duty → proportional speed** (15 = min, 90 = max)

The natural failsafe is therefore **loss of signal = full cooling**: a hung, reset,
or dead MCU that stops driving the line converges the pump to 100%, which is
thermally safe (the engine cannot overheat for lack of cooling). This supersedes the
old "hardware pull to pump-off" model — with a smart pump, "off" is an *active* ~10%
command, so off is **not** the failsafe and cannot be reached without live firmware.

Keep the boot/idle line **defined** (not floating) so the pump reads a clean
no-signal / 0% state and goes full. Expect a brief full-speed burst at cold power-on
until firmware starts; once alive, firmware boots the command at ~10% (off) for fast
warm-up, then modulates per `PumpControl.h` (target 82 °C).

---

## CAN bus

- Interface: FDCAN1
- Speed: 500 Kbps, 29-bit extended frames (matching the rest of the T89 network)
- TX: PB7 | RX: PB8 (fixed by PCB)
- CAN ID definitions: `lib/can_ids/can_ids.h` (project-level source of truth)
  — include via `"can_ids.h"` with `lib_extra_dirs = lib` set in platformio.ini

---

## Reserved / occupied pins (WeAct board)

| Pin(s) | Use |
|--------|-----|
| PB2 | Onboard LED |
| PC13 | Onboard button |
| PA9 / PA10 | Debug UART (USART1, 115200) |
| PA11 / PA12 | USB (CDC virtual COM) |
| PC8–PC12, PD2, PD4 | SDMMC1 (microSD) |
| PA4–PA7 | SPI1 (available but shared with onboard SD via SDMMC1 alt) |
| PA13 | SWDIO — never touch |
| PA14 | SWCLK — never touch |

---

## Peripherals planned

| Peripheral | Notes |
|------------|-------|
| ST7789 2.4" 240×320 TFT (SPI2) | DIN=PB15, CLK=PB13, CS=PB12, DC=PC6, RST=PC7, BL→3.3 V (always on). Init: `init(240,320)`, setRotation(2) → 240×320 portrait, flipped 180°. Module rotated on mount for preferred orientation. Layout: 32px header + 9 data rows × 32px, textSize 2. |
| Dallas DS18B20 (OneWire) | Single GPIO, bit-bang. **Engine water temp** — calibrated °C source that drives cooling-pump control (`PumpControl.h`, target 82 °C). |
| Oil pressure sensor | ADC input |
| Throttle position sensor (TPS) | ADC input |
| Fuel level ×2 | ADC inputs |
| Water temperature (analog NTC) | ADC input (separate from Dallas) |
| Cooling pump (PB1) | Smart pump **speed command**: PB1 PWM (TIM3_CH4, 100 Hz) → 6N137 opto → 2N2222 → controller PWM input. 0%/no-signal = full (failsafe), ~10% = off, 15–90% = proportional. Not a motor drive — no flyback needed. |
| FDCAN1 | 500 Kbps, SN65HVD230 transceiver |
