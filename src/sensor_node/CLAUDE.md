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

## Boot-window safety

Pins that drive external power or actuators (cooling pump PWM gate, any MOSFET gate)
require a **hardware pull to the safe state** (pump off) so the pin is defined from
power-on, before software init runs. Do not rely on software config alone — it leaves
the pin floating through the reset/boot window.

The software fault / safety-monitor state must command the same "off" condition that
the hardware pull enforces, so faults, hangs, and watchdog resets all converge on the
same safe state.

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
| Dallas DS18B20 (OneWire) | Single GPIO, bit-bang |
| Oil pressure sensor | ADC input |
| Throttle position sensor (TPS) | ADC input |
| Fuel level ×2 | ADC inputs |
| Water temperature (analog NTC) | ADC input (separate from Dallas) |
| Cooling pump PWM | TIMx PWM output, 12 V via MOSFET board; hardware pull-down on gate |
| FDCAN1 | 500 Kbps, SN65HVD230 transceiver |
