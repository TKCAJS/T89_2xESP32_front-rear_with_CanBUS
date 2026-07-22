# Sensor Node — STM32H523CET6

The sensor node, ported from the `sensor_node_f103` PlatformIO env (Blue Pill
STM32F103C8T6). Shares `lib/can_ids/can_ids.h` with the other nodes.

Board: WeAct BlackPill-style STM32H523CET6, LQFP48, Cortex-M33 @ 250 MHz,
512 KB flash, 272 KB RAM. No official PlatformIO board definition existed for
this chip at the time of this port — see `boards/generic_h523cetx.json` in
the project root (custom board file, PlatformIO's default `boards_dir`). The
Arduino_Core_STM32 variant this board maps to (`H523C(C-E)(T-U)_H533CE(T-U)`)
is itself newly added and had two real packaging gaps this port had to work
around — see "Upstream packaging gaps" below.

Upload: **ST-Link only** (`upload_protocol = stlink`), same as the F103 build.

Build-verified (`pio run -e sensor_node_h523` succeeds, RAM 0.8% / Flash
11.2%) but **not bench-tested** — see "Not yet verified on real hardware".

---

## Upstream packaging gaps (framework-arduinoststm32, this variant only)

Two genuine defects in how `Arduino_Core_STM32` packages this specific,
newly-added variant — both worked around entirely from project files, no
framework package edits:

1. **No `ldscript.ld` at all** in the variant folder (every other H5 variant
   has one). Without it the linker has no FLASH/RAM memory map and fails at
   link time. Fixed by committing a copy of the sibling STM32H503CB variant's
   linker script (same H5xx family, parametrized via `LD_MAX_SIZE` /
   `LD_MAX_DATA_SIZE` / `LD_FLASH_OFFSET` so it needs no chip-specific
   numbers) as `boards/h523cetx_ldscript.ld`, pointed at via
   `board_build.ldscript` in `platformio.ini`.
2. **`stm32h5xx_ll_dlyb.c`'s guard** (`HAL_SD_MODULE_ENABLED` /
   `HAL_OSPI_MODULE_ENABLED` / `HAL_XSPI_MODULE_ENABLED`) is satisfied by
   default on this chip, but the `LL_DLYB_CfgTypeDef` type it needs ends up
   unavailable where the .c file uses it, so the whole build fails on an
   unrelated DelayBlock (SD/QSPI/XSPI signal-timing calibration) module we
   never touch. Fixed with `-DHAL_SD_MODULE_DISABLED -DHAL_OSPI_MODULE_DISABLED
   -DHAL_XSPI_MODULE_DISABLED` in `build_flags` — these must be **global**
   command-line flags, not a project `include/` header override: the
   `SrcWrapper` library files (where the bug lives) aren't compiled with this
   project's `include/` directory on their search path at all, so a
   `hal_conf_extra.h` in `include/` (the framework's normal, documented
   override hook) silently never gets picked up for them. Confirmed by
   forcing a `#error` inside one and watching it never fire.

Also note: `HAL_FDCAN_MODULE_ENABLED` is **not** on by default for H5 either
(same as the H723 board in this workspace needs it explicitly) — it's easy to
assume otherwise since so much else *is* enabled by default on this chip.

## Not yet verified on real hardware

1. **CAN bit timing.** `SystemClock_Config` in `main.cpp` configures PLL1Q =
   50 MHz based on ST's own reference clock config for the sibling STM32H503CB
   generic variant (same H5xx RCC/PWR IP) — not hand-derived. FDCAN prescaler
   is set for 500 kbit/s off that 50 MHz, but confirm the actual bit rate on
   the bus before connecting other nodes.
2. **PA0 onboard button.** The WeAct board ties PA0 to a populated user
   button (active-low, pulled up) — confirmed via two independent sources
   (Zephyr's board devicetree and its board docs), not measured directly on
   a unit. If your specific board revision doesn't have this button
   populated, PA0 would be free to use — the analog inputs were moved off it
   as a precaution either way.

## Pin map (differences from the F103 build)

| Function | F103C8T6 | H523CET6 | Notes |
|----------|----------|----------|-------|
| CAN TX / RX | PB9 / PB8 | **PA12** / **PA11** | FDCAN1 has no PB9 TX option on this chip — PB9 isn't even a usable Arduino pin name on this variant at all. Originally landed on PB7/PB8 (the other valid FDCAN1 pin pair); moved to PA12/PA11 to free PB7/PB8 for the fan relays (see FanControl.h). |
| Display SCK / MOSI | PB13 / PB15 | PB13 / PB15 | unchanged (SPI2) |
| Display CS / DC | PB12 / PB10 | PB12 / **PA15** | PB11 isn't a usable Arduino pin name on this variant (skipped in `variant_generic.h`, same as PB9 above); DC moved to PA15 so all six display lines sit on the same header edge as the SPI2 block. |
| Display RST | PB11 | **PB3** | Moved to the nearest free GPIO on that same header edge, next to DC. |
| Fan relays (FAN1 / FAN2) | n/a | **PB7 / PB8** | New on H523 — no F103 equivalent. Freed up by moving CAN TX/RX off PB7/PB8 to PA12/PA11 above. |
| Oil pressure | PA0 | **PA1** | ADC — shifted off PA0 (see button note above) |
| TPS | PA1 | **PA2** | ADC |
| Fuel 1 / Fuel 2 | PA2 / PA3 | **PA3 / PA4** | ADC |
| Engine temp NTC | PA4 | **PA5** | ADC — renamed from "water temp" upstream (can_ids.h `MSGTYPE_SENS_ENGINE_TEMP`), same divider/table |
| Rad-out temp NTC | PA5 | **PA6** | ADC — new second NTC added upstream (`MSGTYPE_SENS_RAD_OUT_TEMP`); F103 had PA5 free, H523 doesn't (button-shift above), so it lands on PA6 instead |
| Pump PWM | PB1 | PB1 | unchanged — TIM3_CH4 on both parts, 100 Hz |
| Dallas OneWire | PB0 | PB0 | unchanged — any GPIO |
| Debug UART | PA9 / PA10 | PA9 / PA10 | unchanged (USART1, 115200) |
| SWD | PA13 / PA14 | PA13 / PA14 | never touch |
| Onboard LED | PC13 | PC13 | unchanged, active low |

## CAN driver differences (SensorCan.h)

- FDCAN1 (`HAL_FDCAN_*`), not bxCAN — the H5 series has no bxCAN peripheral
  at all, unlike the F103. Needs `-DHAL_FDCAN_MODULE_ENABLED` explicitly (see
  "Upstream packaging gaps" above).
- FDCAN kernel clock is PLL1Q (50 MHz) — H5 has no plain-PCLK option for
  FDCAN like bxCAN's "straight off APB1." Configured in `SystemClock_Config`.
- H5's `FDCAN_InitTypeDef` is much simpler than the H723's — no message-RAM
  element-count/size fields at all (`RxFifo0ElmtsNbr`, `TxFifoQueueElmtsNbr`,
  etc. don't exist here; message RAM layout is fixed, not configurable). The
  RX FIFO0 and TX FIFO/Queue are both hardwired to 3 elements
  (`SRAMCAN_RF0_NBR`/`SRAMCAN_TFQ_NBR` in the HAL source) — coincidentally
  the same depth as the F103's 3 bxCAN mailboxes, so `canSend`'s
  wait-for-free-slot guard needed no logic changes, just a corrected comment.
- A single accept-all extended mask filter (mask=0) plus software ID
  filtering in `canReceivePoll`, mirroring the F103's bxCAN filter approach,
  rather than the per-ID exact-match filters used on the H723 GPS node.
- `HAL_FDCAN_GetProtocolStatus` (BusOff/ErrorPassive/Warning) replaces the
  bxCAN `CAN1->ESR` register read — same escalation, different API.
- FDCAN1 has its own interrupt vector (`FDCAN1_IT0_IRQn`), not a shared
  USB/CAN vector like the F103's `USB_LP_CAN1_RX0`.

## System clock (main.cpp)

The generic H523 Arduino variant ships an **empty** `SystemClock_Config()`
(just a `#warning`) — without an override, the chip runs on its reset-default
clock, not 250 MHz. This port's override is adapted directly from ST's own
working config for the STM32H503CB generic variant (same H5xx family, same
RCC/PWR IP): CSI 4 MHz → PLL1 (M=2, N=250, P=2) = 250 MHz SYSCLK, VOS0, wide
VCO range. SPI2 (display) and FDCAN both draw their kernel clock from PLL1Q
(50 MHz) since H5 has no plain-PCLK source option for either.

## GPIO blanket init

Same rules as the F103 node (all-analog pass first, PA13/PA14 excluded so
SWD survives, active pins configured after) — but Port A and Port B only.
Unlike the F103's LQFP48 (which exposes PC13/14/15 alongside a full A/B),
**this chip's LQFP48 has no general-purpose Port C pins at all** — confirmed
via the chip's `PeripheralPins.c` in `Arduino_Core_STM32`, which lists zero
PC pins across every peripheral (ADC/SPI/TIM/USART/FDCAN). PC13 still exists
as the onboard LED because it's on the separate VBAT/RTC domain, not general
GPIOC — that part isn't swept in the blanket pass, just configured directly
as an output in `setup()`.

## Flash/RAM budget

512 KB flash / 272 KB RAM — both roughly 8x the F103's 64 KB / 20 KB. Actual
build uses 58.6 KB flash / 2.2 KB RAM, so no budget pressure at all; unlike
the F103 node, `_printf_float` isn't worth avoiding here, though the display
code hasn't been changed to use it since the integer fixed-point renderer
already works and there's no reason to touch it.
