# Sensor Node — STM32H523CCT6

The sensor node, ported from the `sensor_node_f103` PlatformIO env (Blue Pill
STM32F103C8T6). Shares `lib/can_ids/can_ids.h` with the other nodes.

Board: WeAct BlackPill-style STM32H523, LQFP48, Cortex-M33 @ 250 MHz,
256 KB flash, 272 KB RAM. No official PlatformIO board definition existed for
this chip at the time of this port — see `boards/generic_h523cctx.json` in
the project root (custom board file, PlatformIO's default `boards_dir`). The
Arduino_Core_STM32 variant this board maps to (`H523C(C-E)(T-U)_H533CE(T-U)`)
is itself newly added and had two real packaging gaps this port had to work
around — see "Upstream packaging gaps" below.

## Was CET6, now CCT6 (2026-08-01)

This node was originally built on the **STM32H523CET6** (512 KB flash). That
part died on the bench, and the replacement stocked is the **CCT6** — same
die, same LQFP48 pinout, same 272 KB RAM, same peripherals, 256 KB flash.

The swap needed **no source, pin map or clock changes whatsoever**: the
Arduino core variant folder already covers both (`H523C(C-E)…` — the `C` is
the 256 KB part), the build is 22.5% of 256 KB, and nothing here stores data in
flash. Verified by building against both board files and diffing the output —
the two `firmware.bin` files were byte-identical. The entire difference was
`maximum_size: 262144` in the board file and the pyocd target name.

`boards/h523_ldscript.ld` takes its FLASH/RAM lengths from the board file
(`LD_MAX_SIZE` / `LD_MAX_DATA_SIZE`), so it needed no edit and would serve any
other H523 flash size the same way.

### Failure symptoms (2026-08-01)

- Node worked normally on the car the previous day.
- Powered up the next day, **car not started** (battery only, no cranking, no
  alternator) — chip totally dead, no power LED.
- The 3.3 V rail (fed by an external buck) measures **~1.5 V**, not 3.3 V.
- **With the MCU removed entirely and the board powered from USB, still no
  power LED.**
- **The removed MCU measures ~0 Ω from its VDD pin to its VSS pin, out of
  circuit** — the die itself is internally shorted supply-to-ground.

The MCU is therefore definitively destroyed, and that short also explains the
1.5 V: with the dead chip still fitted, it was dragging the buck into current
limit. An internal VDD–VSS short is the classic signature of **overvoltage on
VDD** (or a latch-up that took the die with it) — not of a chip that simply
wore out.

- **With the MCU not fitted, the buck reads a correct 3.3 V** — so the buck is
  regulating properly in steady state, and the board's own rail is not shorted
  either (a shorted rail would drag the buck down).

That last result rules out the simple story. The buck is not sitting there
over-volting the rail, so **the cause is not yet found**. Two hypotheses remain
live, and they need different fixes:

**A — the buck over-volts transiently, not in steady state.** A DC reading
into no load is weak evidence: a degraded output cap, a cracked feedback-divider
joint, or an input transient can produce a startup or load-step overshoot that
never shows on a multimeter. The MCU dying on a *power-up* fits this well.

**B — latch-up through an I/O pin from the 12 V side.** A severe latch-up ends
in exactly this signature (VDD–VSS short) while leaving the supply innocent.
The exposed pins are the senders on PA1–PA4 (oil pressure, TPS, fuel 1/2),
which are the only DC-coupled wires into the 12 V system, then CAN on PB7/PB8.

**The cheapest discriminator: check whether the other parts on the 3.3 V rail
survived.** If the buck had over-volted the rail, the display module, CAN
transceiver and DS18B20 should be dead too. If they all still work, the
overvoltage was localised to the MCU and hypothesis B moves to the front.

What is *not* suspect (confirmed with the builder, 2026-08-01):

- **Pump PWM (PB1) is opto-isolated** — PB1 drives a PC817 LED through a 220 Ω
  series resistor (~9.5 mA at Vf≈1.2 V, comfortably inside the pin's rating),
  and the opto's output transistor drives a 2N2222 that drives the pump. No
  galvanic path from the pump side to the MCU at all, and no pin overstress.
- **Fan relays (PA11/PA12) are not connected yet**, so no relay coil has ever
  been switched by this board.

### Diagnostic order (all with the MCU off the board)

Done so far: buck reads a correct 3.3 V unloaded, board rail not shorted.
Remaining, in order:

1. **Do the other 3.3 V parts still work?** Display, CAN transceiver, DS18B20.
   This is the free discriminator between hypotheses A and B above — all alive
   points at an I/O-pin event, not a rail-wide overvoltage.
2. **Load-test the buck.** Dummy load at roughly the node's normal draw, left
   running 15–30 min until warm, watching for drift. Unloaded DC is not enough
   to clear it.
3. **Scope the rail at power-up** if a scope is available. The startup
   overshoot is exactly the moment an MCU dies, and it is invisible to a meter.
   Anything above ~3.6 V is the answer. Also poke the feedback divider and
   freeze-spray the buck looking for the output jumping.
4. **Check the sender lines on PA1–PA4** for continuity or leakage to 12 V, and
   confirm each has a series resistor and a clamp to 3.3 V. This is hypothesis
   B's mechanism, and the wiring is the fix.
5. **Confirm USB actually feeds this rail** before reading anything into the
   earlier "no LED on USB" result — on WeAct-style boards that path can be
   jumper- or solder-bridge-selected, and if it was never connected, that
   observation proves nothing. (With the buck now reading 3.3 V, check whether
   the power LED lights on *buck* power; if not, the LED itself likely went.)

Regardless of which hypothesis wins, fit the protection before the next chip:
a TVS or zener clamp on the 3.3 V rail, a polyfuse upstream, and series
resistors plus clamp diodes on the four sender inputs. A chip died of
overvoltage and the cause is not proven — that is the case for protecting
against both paths rather than picking one.

### Longer-term exposure (not implicated in this failure)

1. **The analog senders on PA1–PA4** (oil pressure, TPS, fuel 1/2) — the only
   DC-coupled wires running into the 12 V system. A sender line touching 12 V,
   or a chassis-ground offset during cranking, puts the pin well past
   VDD+0.3 V, and the internal clamp diodes are only good for a few mA. Each
   wants a series resistor and a clamp to the 3.3 V rail. The two NTC inputs
   (PA5/PA6) are *not* in this group — per `NtcTemp.h` those sensors are 2-wire
   to board GND with no vehicle ground reference.
2. **CAN on PB7/PB8** — a 5 V transceiver (MCP2551/TJA1050) driving RXD at 5 V
   into a pin that may not be 5 V tolerant on this part; check the datasheet's
   FT/TT column for PB8 before assuming it is.

## Upload

**ST-Link only**, but NOT via `upload_protocol = stlink` — PlatformIO's
bundled OpenOCD (checked up to tool-openocd 3.1200.7, the latest available)
has no `target/stm32h5x.cfg` and no `stm32h5x` flash driver at all, so that
protocol fails outright ("Can't find target/stm32h5x.cfg"). The env instead
uses `upload_protocol = custom` with `pyocd flash -t stm32h523cctx` — pyocd
uses the CMSIS-Pack (Keil.STM32H5xx_DFP) for flash algorithms instead of
OpenOCD's target scripts, so it doesn't hit the same gap.

One-time setup on a new machine: `~/.platformio/penv/bin/pip install pyocd`,
then `~/.platformio/penv/bin/pyocd pack install stm32h523cctx` to fetch the
CMSIS pack (pyocd will also fetch it automatically on first flash if missing,
just slower). After that, `pio run -e sensor_node_h523 -t upload` works
exactly like any other env. If that target name doesn't resolve,
`pyocd pack find stm32h523` lists the exact spellings in the DFP — the CCT6
target has not been flashed yet, only built.

Build-verified (`pio run -e sensor_node_h523` succeeds — sizes in "Flash/RAM
budget" below) and bench-flashes cleanly via pyocd — but a clean flash is not
the same as working hardware: the display didn't actually come up until
2026-07-25 (see "Arduino pin-map ALT traps" below), so treat anything not
listed under "Confirmed on real hardware" as unverified.

## Confirmed on real hardware (2026-07-25)

Everything in this section was confirmed on the **CET6** board, before it
died. The CCT6 is the same die and the firmware binary is identical, so these
results carry over — but nothing has been re-confirmed on a CCT6 yet.

- Display (2.4" ST7789 over SPI2) — boots and renders after the ALT-pin fix
  below.
- Debug UART on PA9/PA10 (USART1) — confirmed receiving `DebugSerial` output
  after rebinding it explicitly (see below); the framework's default `Serial`
  does **not** land here, see "Arduino pin-map ALT traps".
- FDCAN1 500 kbit/s init succeeds and reports bus-off correctly with no
  other node/termination connected (expected with nothing else on the bus).

## Arduino pin-map ALT traps (found bringing the display/UART up)

This variant's `PeripheralPins.c` lists some pins twice: once under the
plain name resolving to one peripheral, and again under a `_ALT1` name
resolving to a different one. `SPIClass`/`HardwareSerial` pick a peripheral
per-pin via `pinmap_peripheral()`, so using the wrong (plain vs. `_ALT1`)
name silently selects the wrong instance — no compile error, and the
resulting peripheral-mismatch hangs HAL init forever with no error message.
Confirmed instances so far:

- **PA9/PA10**: plain names resolve to **LPUART1**, not USART1 — need
  `PA9_ALT1`/`PA10_ALT1` for USART1. The framework's default `Serial` object
  uses UART4 on PA0/PA1 for this variant regardless (see
  `variant_generic.h`'s `SERIAL_UART_INSTANCE`), which also collides with
  this design's onboard-button (PA0) and oil-pressure ADC (PA1) pins. Fixed
  by declaring `HardwareSerial DebugSerial(PA10_ALT1, PA9_ALT1)` in
  `main.cpp` and routing all debug prints through it instead of `Serial`.
- **PB15**: plain name resolves to **SPI1**, not SPI2 — need `PB15_ALT1` for
  SPI2. `SensorDisplay.h`'s `SPIClass s_spi2(PB15, PB14, PB13)` mixed MOSI on
  SPI1 with MISO/SCK on SPI2 (PB13/PB14 plain names are correct, no ALT
  needed there); `pinmap_merge_peripheral()` can't reconcile the mismatch and
  `s_tft.init()` hung forever with the LED stuck solid and no error output.
  Fixed by changing MOSI to `PB15_ALT1`.
- FDCAN1 on PB7/PB8 was **not** affected — `SensorCan.h` sets the GPIO
  alternate function directly via raw HAL calls (`GPIO_AF9_FDCAN1`) rather
  than going through `pinmap_peripheral()`, so it never hit this class of bug.
  Worth checking any *other* `SPIClass`/`HardwareSerial`/`TwoWire` pin
  assignment on this variant against `PeripheralPins.c` before assuming the
  plain pin name is correct.

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
   numbers, so the CC and CE envs share one copy) as
   `boards/h523_ldscript.ld`, pointed at via `board_build.ldscript` in
   `platformio.ini`.
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
2. **The DS18B20 driver in `DallasTemp.h`.** Written to fix a boot hang that
   only appeared with the sensor physically attached (see "DS18B20 driver"
   below); build-verified only. If the radiator temp still reads
   −127 °C / `NODE_STATUS_SENSOR_ERR` stays set, the remaining unknowns are
   the bit-slot constants and the cable — put a scope on PB0 and check that a
   read slot samples ~10 µs after the falling edge and that the line reaches
   3.3 V within it.
3. **PA0 onboard button.** The WeAct board ties PA0 to a populated user
   button (active-low, pulled up) — confirmed via two independent sources
   (Zephyr's board devicetree and its board docs), not measured directly on
   a unit. If your specific board revision doesn't have this button
   populated, PA0 would be free to use — the analog inputs were moved off it
   as a precaution either way.
3. **Oil pressure calibration.** `main.cpp` still passes raw ADC counts
   straight through for oil pressure — no sensor is wired up yet. `can_ids.h`
   documents this channel as kPa×10; `sendOilPressure()` clamps so the raw
   counts can't overflow/wrap the CAN payload in the meantime, but the value
   itself isn't real kPa until an actual ADC-to-kPa calibration replaces the
   passthrough. TPS/Fuel1/Fuel2 got a percent-of-full-ADC-range conversion
   instead (`adcToPercent()` in `main.cpp`) since those are potentiometer-style
   senders where that's the standard assumption absent 2-point (end-stop)
   calibration — replace with real end-stop counts if the sender doesn't
   span the full ADC range.

## Pin map (differences from the F103 build)

| Function | F103C8T6 | H523Cx | Notes |
|----------|----------|----------|-------|
| CAN TX / RX | PB9 / PB8 | **PB7** / PB8 | FDCAN1 has no PB9 TX option on this chip — PB9 isn't even a usable Arduino pin name on this variant at all. RX unchanged. (Briefly moved to PA12/PA11 to free PB7/PB8 for the fan relays; moved back and the fans went to PA11/PA12 instead — see FanControl.h.) |
| Display SCK / MOSI | PB13 / PB15 | PB13 / **PB15_ALT1** | plain PB15 resolves to SPI1 on this variant, not SPI2 — see "Arduino pin-map ALT traps" |
| Display CS / DC | PB12 / PB10 | PB12 / **PA15** | PB11 isn't a usable Arduino pin name on this variant (skipped in `variant_generic.h`, same as PB9 above); DC moved to PA15 so all six display lines sit on the same header edge as the SPI2 block. |
| Display RST | PB11 | **PB3** | Moved to the nearest free GPIO on that same header edge, next to DC. |
| Fan relays (FAN1 / FAN2) | PB3 / PB5 | **PA11 / PA12** | No USB on this node (unlike the H723 transmitter, where these are USB-C), so they're free here. Kept off PB3/PB5 (the F103's fan pins) since PB3 is this board's TFT_RST. **Not physically connected yet** (as of 2026-08-01) — the firmware drives them, but nothing is on the other end. |
| Oil pressure | PA0 | **PA1** | ADC — shifted off PA0 (see button note above) |
| TPS | PA1 | **PA2** | ADC |
| Fuel 1 / Fuel 2 | PA2 / PA3 | **PA3 / PA4** | ADC |
| Engine temp NTC | PA4 | **PA5** | ADC — renamed from "water temp" upstream (can_ids.h `MSGTYPE_SENS_ENGINE_TEMP`), same divider/table |
| Rad-out temp NTC | PA5 | **PA6** | ADC — new second NTC added upstream (`MSGTYPE_SENS_RAD_OUT_TEMP`); F103 had PA5 free, H523 doesn't (button-shift above), so it lands on PA6 instead |
| Pump PWM | PB1 | PB1 | unchanged — TIM3_CH4 on both parts, 100 Hz. Drives a PC817 opto LED via 220 Ω (~9.5 mA); the opto's output drives a 2N2222 to the pump — the MCU is galvanically isolated from the pump side. |
| Dallas OneWire | PB0 | PB0 | unchanged — any GPIO |
| Debug UART | PA9 / PA10 | **PA9_ALT1 / PA10_ALT1** | plain names resolve to LPUART1 on this variant, not USART1 — see "Arduino pin-map ALT traps"; routed through a dedicated `DebugSerial`, not the framework's default `Serial` |
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

## DS18B20 driver (DallasTemp.h) — no OneWire/DallasTemperature

This env does **not** depend on `paulstoffregen/OneWire` or
`milesburton/DallasTemperature` (the F103 env still does). `DallasTemp.h` is a
self-contained bit-banger. Two reasons, both specific to running those
libraries on this core:

1. **OneWire's STM32 backend misses the read window.** Its
   `DIRECT_MODE_INPUT` / `DIRECT_MODE_OUTPUT` macros
   (`util/OneWire_direct_gpio.h`, the `ARDUINO_ARCH_STM32` branch) are
   `pin_function()` → `HAL_GPIO_Init()`, and `read_bit()` calls two of them
   *inside* the read slot. The DS18B20 holds read data valid for only 15 µs
   after the falling edge; the two HAL calls on top of the library's 3+10 µs
   waits put the sample at ~17 µs, so zeros come back as ones — but only once a
   device is actually attached, which is why the node behaved with the sensor
   unplugged. Here the pin is a permanently open-drain output and every edge is
   a single-cycle BSRR write, so the sample lands at 10 µs with ~5 µs of margin.
2. **`DallasTemperature` runs a full 64-bit ROM search on every read.**
   `getTempCByIndex()` → `getAddress()` searches the bus each call (~13 ms with
   interrupts masked in ~200 µs chunks), and `begin()` walks the same search —
   which on a bus with corrupted bits wanders a bogus ROM tree. There is
   exactly one device on this bus, so every transaction here uses SKIP ROM and
   no search exists anywhere.

Scratchpad reads are CRC-8 checked and every wait is bounded, so a missing,
shorted or noisy sensor costs ~1 ms once a second and cannot stall the loop. A
failed read publishes `DALLAS_INVALID_C` (−127 °C) on `CAN_SENS_RADIATOR_TEMP`
and raises `NODE_STATUS_SENSOR_ERR` in the status byte — the Dallas is a
calibration sanity check against the NTCs, not part of the cooling control
loop, so it is never allowed to be fatal.

Timing note: `delayMicroseconds()` on this core takes the DWT cycle-counter
path (`wiring_time.h`), which is exact — no `-1 µs` fudge like the SysTick
fallback — and `SystemCoreClock` is updated to 250 MHz by `HAL_RCC_ClockConfig`
inside `SystemClock_Config`. The slot constants assume both.

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

Actual build (2026-08-01): **58,892 bytes flash / 2,540 bytes RAM** — 22.5% of
the CCT6's 256 KB and 0.9% of its 272 KB RAM, leaving ~203 KB of flash free.
Still far beyond the F103's 64 KB / 20 KB, so dropping from the CET6's 512 KB
cost nothing in practice.

Most of that 59 KB is fixed cost that doesn't grow with application code —
`.text` ~53.9 KB (Adafruit GFX/ST7789 plus the STM32 HAL), `.rodata` ~4.7 KB
(mostly the GFX font), vectors 596 B. Nothing here stores anything *in* flash:
the only `FLASH` reference in `main.cpp` is `__HAL_FLASH_SET_PROGRAM_DELAY`
(wait-state config), there is no EEPROM emulation and no code that assumes a
512 KB-based address, which is why the shrink needs no source changes.

Unlike the F103 node, `_printf_float` isn't worth avoiding on either part,
though the display code hasn't been changed to use it since the integer
fixed-point renderer already works and there's no reason to touch it.
