#define SOFTWARE_VERSION  100   // v1.0.0

// Sensor node — STM32H523CET6. Ported from the STM32F103C8T6 (Blue Pill)
// build; pin/driver history is in CLAUDE.md.

#include <Arduino.h>
#include "can_ids.h"
#include "SensorDisplay.h"
#include "SensorCan.h"
#include "DallasTemp.h"
#include "PumpControl.h"
#include "NtcTemp.h"

// ── System clock ──────────────────────────────────────────────────────────────
// The generic H523 Arduino variant ships an empty SystemClock_Config() (runs on
// the reset-default clock otherwise). This override is adapted from ST's own
// working config for the sibling STM32H503CB generic variant (same H5xx RCC/PWR
// IP): CSI 4 MHz -> PLL1 (M=2, N=250, P=2) = 250 MHz SYSCLK, VOS0, wide VCO.
// PLL1Q = 500 MHz / 10 = 50 MHz, used below as the FDCAN kernel clock.
void SystemClock_Config(void) {
    RCC_OscInitTypeDef  osc = {};
    RCC_ClkInitTypeDef  clk = {};
    RCC_PeriphCLKInitTypeDef pclk = {};

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    osc.OscillatorType      = RCC_OSCILLATORTYPE_CSI | RCC_OSCILLATORTYPE_HSI48
                             | RCC_OSCILLATORTYPE_LSI;
    osc.CSIState            = RCC_CSI_ON;
    osc.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
    osc.HSI48State          = RCC_HSI48_ON;
    osc.LSIState            = RCC_LSI_ON;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLL1_SOURCE_CSI;
    osc.PLL.PLLM            = 2;
    osc.PLL.PLLN            = 250;
    osc.PLL.PLLP            = 2;
    osc.PLL.PLLQ            = 10;
    osc.PLL.PLLR            = 2;
    osc.PLL.PLLRGE          = RCC_PLL1_VCIRANGE_1;
    osc.PLL.PLLVCOSEL       = RCC_PLL1_VCORANGE_WIDE;
    osc.PLL.PLLFRACN        = 0;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) { while (1) {} }

    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
                        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                        | RCC_CLOCKTYPE_PCLK3;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    clk.APB3CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) { while (1) {} }

    __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);

    // SPI2 (display) off PLL1Q too, alongside FDCAN — both need an explicit
    // kernel-clock source on H5 (no plain PCLK option for FDCAN).
    pclk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN | RCC_PERIPHCLK_SPI2;
    pclk.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PLL1Q;
    pclk.Spi2ClockSelection   = RCC_SPI2CLKSOURCE_PLL1Q;
    if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK) { while (1) {} }
}

// ── Analogue inputs ───────────────────────────────────────────────────────────
// PA0 is the WeAct BlackPill H523 board's onboard user button (active-low with
// a pull-up) — reading it as analog would fight that pull-up, so the 5 analog
// channels shift one pin right vs the F103 build (PA0-4 -> PA1-5).
#define PIN_OIL_PRESSURE  PA1
#define PIN_TPS           PA2
#define PIN_FUEL_1        PA3
#define PIN_FUEL_2        PA4
#define PIN_WATER_TEMP    PA5   // analog NTC
#define NTC_AVG_SAMPLES   20    // spread evenly across one pump-PWM period so
                                // PWM ripple coupled onto the NTC averages out
                                // (NTC counts drive the pump duty directly)
#define NTC_SAMPLE_GAP_US (1000000 / PUMP_PWM_FREQ_HZ / NTC_AVG_SAMPLES)

// ── Pump PWM speed command ────────────────────────────────────────────────────
// PB1 PWM -> 6N137 opto -> 2N2222 -> smart pump controller's PWM input.
// This is a SPEED COMMAND, not a motor drive. Measured pump band (see
// PumpControl.h): 0% = failsafe full, ~10% = off, 15-90% = proportional.
// NOTE: because 0% = full, a line held low commands FULL cooling, not off.
// PB1 is TIM3_CH4 on the H523 too — unchanged from the F103 board.
#define PIN_PUMP_PWM  PB1   // TIM3_CH4
#define PUMP_PWM_FREQ_HZ  100   // inside the pump controller's expected window

// ── Heartbeat LED ─────────────────────────────────────────────────────────────
// WeAct BlackPill H523 onboard LED, active LOW — same pin as the F103's Blue
// Pill LED. Blink rate tracks the pump duty command: 1 Hz at minimum flow
// (15%) up to 9 Hz at max cooling (90%) — a glance at the LED shows how hard
// the pump works. TIM2 toggles the pin in an interrupt so the blink stays
// steady while the loop blocks on SPI display repaints or CAN TX FIFO waits;
// the loop only updates the timer period when the duty command moves.
// (TIM3 is taken by the pump PWM.)
#define PIN_LED             PC13
#define LED_TOGGLE_SLOW_MS  500   // at PUMP_DUTY_MIN — 1 Hz blink = loop alive
#define LED_TOGGLE_FAST_MS  55    // at PUMP_DUTY_MAX — ~9 Hz blink

static HardwareTimer s_ledTimer(TIM2);
static uint32_t      s_ledToggleMs = LED_TOGGLE_SLOW_MS;

static void ledHeartbeatToggle() {
    digitalToggle(PIN_LED);
}

// ── Shared state (extern'd by SensorCan.h) ────────────────────────────────────
uint8_t            g_nodeStatus = NODE_STATUS_OK;
volatile bool      g_canReady   = false;
volatile CanHealth g_canHealth  = CAN_HEALTH_FAULT;
volatile uint8_t   g_pumpDuty    = 0;
volatile uint8_t   g_pumpTargetC = PUMP_TARGET_DEFAULT;   // set from display via CAN
volatile uint16_t  g_rpm        = 0;
volatile uint8_t   g_gear       = GEAR_UNKNOWN;

// ── Sensor readings ───────────────────────────────────────────────────────────
static float g_oilPressure = 0.0f;
static float g_tps         = 0.0f;
static float g_fuel1       = 0.0f;
static float g_fuel2       = 0.0f;
static float g_waterTempC  = 0.0f;

// ── Timing ────────────────────────────────────────────────────────────────────
static uint32_t g_lastSensorRead  = 0;
static uint32_t g_lastDisplay     = 0;
static uint32_t g_lastSensorTx    = 0;
static uint32_t g_lastCanHealth   = 0;
static uint32_t g_lastStatusTx    = 0;

#define SENSOR_READ_MS   200
#define DISPLAY_MS       200   // 5 fps — plenty for this readout
#define SENSOR_TX_MS     200
#define CAN_HEALTH_MS    200
#define STATUS_TX_MS     200

// ─────────────────────────────────────────────────────────────────────────────
// GPIO init — blanket analog pass FIRST, peripheral configs follow in setup()
// H523CE LQFP48: only ports A and B carry general-purpose pins; Port C exposes
// just PC13 (RTC/VBAT domain — the onboard LED), so it isn't swept here. PA13/
// PA14 lose the blanket pass so SWD (SWDIO/SWCLK) survives.
// ─────────────────────────────────────────────────────────────────────────────
static void gpio_blanket_init() {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef cfg = {};
    cfg.Mode = GPIO_MODE_ANALOG;
    cfg.Pull = GPIO_NOPULL;

    cfg.Pin = GPIO_PIN_ALL & ~(GPIO_PIN_13 | GPIO_PIN_14);
    HAL_GPIO_Init(GPIOA, &cfg);

    cfg.Pin = GPIO_PIN_ALL;
    HAL_GPIO_Init(GPIOB, &cfg);
}

void setup() {
    gpio_blanket_init();

    // STM32duino defaults analogRead to 10-bit; the NTC math (NTC_ADC_MAX)
    // and the display's EPS_RAW deadband both assume 12-bit counts.
    analogReadResolution(12);

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);   // active low — on during boot, blinks once loop runs

    pinMode(PIN_PUMP_PWM, OUTPUT);
    analogWriteFrequency(PUMP_PWM_FREQ_HZ);
    // Boot at minimum flow (15%); the water-temp algorithm takes over at the
    // 200 ms sensor read.
    // Never command 0% — that's the controller's failsafe = full speed.
    g_pumpDuty = PUMP_DUTY_MIN;
    analogWrite(PIN_PUMP_PWM, map(g_pumpDuty, 0, 100, 0, 255));

    Serial.begin(115200);   // USART1 PA9/PA10 — no USB CDC on this build
    delay(500);
    Serial.println("[SENSOR NODE] v" + String(SOFTWARE_VERSION) + " Booting...");

    displayBegin();
    displayUpdate(0, 0, 0, 0, 0, 0, 0, g_rpm, g_gear, CAN_HEALTH_FAULT);

    g_canReady = canInit();
    if (!g_canReady) {
        g_canHealth   = CAN_HEALTH_FAULT;
        g_nodeStatus |= NODE_STATUS_CAN_ERR;
        Serial.println("[SENSOR NODE] WARNING: CAN init failed");
    }

    dallasBegin();

    // Start the heartbeat last — LED is solid through boot, blinks once alive.
    s_ledTimer.setOverflow(s_ledToggleMs * 1000, MICROSEC_FORMAT);
    s_ledTimer.attachInterrupt(ledHeartbeatToggle);
    s_ledTimer.resume();

    Serial.println("[SENSOR NODE] Ready");
}

void loop() {
    uint32_t now = millis();

    // ── Heartbeat — toggled by TIM2; here we only retune the period when the
    //    pump duty command (15..90%) moves, so the blink itself never jitters ──
    uint8_t  duty         = g_pumpDuty;
    uint32_t ledToggleMs  = map(constrain(duty, PUMP_DUTY_MIN, PUMP_DUTY_MAX),
                                PUMP_DUTY_MIN, PUMP_DUTY_MAX,
                                LED_TOGGLE_SLOW_MS, LED_TOGGLE_FAST_MS);
    if (ledToggleMs != s_ledToggleMs) {
        s_ledToggleMs = ledToggleMs;
        s_ledTimer.setOverflow(ledToggleMs * 1000, MICROSEC_FORMAT);
        s_ledTimer.refresh();   // reload now — a shrunk ARR below CNT would
                                // otherwise wait out a full counter wrap
    }

    // ── Read sensors ──────────────────────────────────────────────────────────
    if (now - g_lastSensorRead >= SENSOR_READ_MS) {
        g_lastSensorRead = now;

        g_oilPressure = analogRead(PIN_OIL_PRESSURE);
        g_tps         = analogRead(PIN_TPS);
        g_fuel1       = analogRead(PIN_FUEL_1);
        g_fuel2       = analogRead(PIN_FUEL_2);
        // ~10 ms blocking burst — same order as a display repaint, and CAN RX
        // is interrupt-driven, so nothing time-critical waits on this.
        uint32_t ntcSum = 0;
        for (int i = 0; i < NTC_AVG_SAMPLES; i++) {
            ntcSum += analogRead(PIN_WATER_TEMP);
            if (i < NTC_AVG_SAMPLES - 1) delayMicroseconds(NTC_SAMPLE_GAP_US);
        }
        g_waterTempC  = ntcTempC(ntcSum / NTC_AVG_SAMPLES);

        // Water temp (NTC) -> pump duty command, out as 8-bit PWM (percent
        // mapped to 0..255). Curve centers on the CAN-adjustable target temp.
        g_pumpDuty = pumpDutyForTemp(g_waterTempC, g_pumpTargetC);
        analogWrite(PIN_PUMP_PWM, map(g_pumpDuty, 0, 100, 0, 255));
    }

    // ── Gear fast path — repaint immediately on change (single glyph, cheap),
    //    so gear doesn't wait out the 200ms tick like the slow sensor rows ─────
    displayUpdateGear(g_gear);

    // ── Refresh display (5 fps — decoupled from the faster sensor read) ───────
    if (now - g_lastDisplay >= DISPLAY_MS) {
        g_lastDisplay = now;
        displayUpdate(g_oilPressure, g_tps, g_fuel1, g_fuel2,
                      g_waterTempC, dallasTempC(), g_pumpDuty,
                      g_rpm, g_gear, g_canHealth);
    }

    // ── Dallas radiator temperature (non-blocking, 1 Hz — display + CAN only) ─
    dallasUpdate(now);

    // ── CAN receive is interrupt-driven (FDCAN1_IT0 → canReceivePoll).
    //    The FDCAN RX FIFO0 is configured for 32 elements (see SensorCan.h),
    //    but the loop still blocks tens of ms on the SPI display write, so the
    //    ISR drains every frame the instant it arrives rather than polling. ───

    // ── CAN health (slow cadence — protocol status doesn't change quickly) ───
    if (now - g_lastCanHealth >= CAN_HEALTH_MS) {
        g_lastCanHealth = now;
        canHealthPoll();
    }

    // ── Transmit sensor data ──────────────────────────────────────────────────
    if (g_canReady && now - g_lastSensorTx >= SENSOR_TX_MS) {
        g_lastSensorTx = now;
        sendOilPressure(g_oilPressure);
        sendWaterTemp(g_waterTempC);
        sendRadiatorTemp(dallasTempC());
        sendTPS(g_tps);
        sendFuel1(g_fuel1);
        sendFuel2(g_fuel2);
    }

    // ── Transmit status + heartbeat (5 Hz) ───────────────────────────────────
    if (now - g_lastStatusTx >= STATUS_TX_MS) {
        g_lastStatusTx = now;
        if (g_canReady) {
            sendPumpStatus(g_pumpDuty);
            sendSensStatus();
            sendHeartbeat();
        }
    }
}
