# Sensor Node — STM32F103C8T6 (Blue Pill)

The sensor node, built as the `sensor_node_f103` PlatformIO env. Originally a
port of an STM32H562RGT6 board whose folder (`src/sensor_node/`) has since been
removed — the H562 references below survive as porting history. Shares
`lib/can_ids/can_ids.h` with the other nodes.

Board: Blue Pill STM32F103C8T6 (LQFP48, 72 MHz Cortex-M3, 64 KB flash, 20 KB SRAM)

Upload: **ST-Link only** (`upload_protocol = stlink`). USB is unusable here:
bxCAN and the USB peripheral share their packet SRAM, so CAN and USB cannot run
simultaneously on the F103. No DFU, no CDC — debug output is USART1 (PA9/PA10).

---

## Pin map (differences from the H562 board)

| Function | H562 | F103C8T6 | Notes |
|----------|------|----------|-------|
| CAN TX / RX | PB7 / PB8 | **PB9 / PB8** | bxCAN AFIO remap 2 (`__HAL_AFIO_REMAP_CAN1_2`) |
| Display SCK / MOSI | PB13 / PB15 | PB13 / PB15 | unchanged (SPI2) |
| Display CS | PB12 | PB12 | unchanged |
| Display DC / RST | PC6 / PC7 | **PB10 / PB11** | PC6/PC7 absent on LQFP48; PB14 is claimed as SPI2 MISO so it can't be DC |
| Oil pressure | PC1 | **PA0** | ADC |
| TPS | PC2 | **PA1** | ADC |
| Fuel 1 / Fuel 2 | PC3 / PC4 | **PA2 / PA3** | ADC |
| Water NTC | PC5 | **PA4** | ADC |
| Pump PWM | PB1 | PB1 | unchanged — TIM3_CH4 on both parts, 100 Hz |
| Dallas OneWire | PC0 | **PB0** | any GPIO |
| Debug UART | PA9 / PA10 | PA9 / PA10 | unchanged (USART1, 115200) |
| SWD | PA13 / PA14 | PA13 / PA14 | never touch |
| Onboard LED | PB2 | PC13 | Blue Pill LED, active low |

## CAN driver differences (SensorCan.h)

- bxCAN (`HAL_CAN_*`), not FDCAN. Enabled via `-DHAL_CAN_MODULE_ENABLED`.
- Clocks straight off APB1 (36 MHz) — no kernel-clock mux.
  500 kbps timing: prescaler 4 → 9 MHz, 18 TQ (BS1=14, BS2=3, SJW=3), SP 83%.
- 3 TX mailboxes with `TransmitFifoPriority = ENABLE` (chronological order) in
  place of the FDCAN 3-deep TX FIFO; same wait-for-free-slot guard.
- RX FIFO0 interrupt is the shared `USB_LP_CAN1_RX0` vector (USB unused).
- Health flags read from `CAN1->ESR` (BOFF/EPVF/EWGF) — no GetProtocolStatus.
- Bus-off recovery is manual (`AutoBusOff = DISABLE`, stop → 100 ms → start),
  matching the H562 behaviour.

## GPIO blanket init

Same rules as the H562 node (all-analog pass first, PA13/PA14 excluded, active
pins configured after), but ports A–C only. GPIOD on this package is just
PD0/PD1 = HSE crystal — never reconfigure. The blanket pass also disconnects
JTAG (PB3/PB4/PA15); SWD survives.

## Flash budget

64 KB part. Full build measures ~39 KB (~60%). This env does **not** link
`_printf_float` — the display renders ×10 fixed-point with
integer snprintf (`_dispRow` in SensorDisplay.h); adding a `%f` anywhere will
print blanks until the flag is restored (costs ~13 KB). Most C8T6 dies
physically carry 128 KB if truly needed (`board = bluepill_f103c8_128k`).
