#define SOFTWARE_VERSION  100   // v1.0.0

#include <Arduino.h>
#include "can_ids.h"
#include "SensorDisplay.h"
#include "SensorCan.h"
#include "DallasTemp.h"
#include "PumpControl.h"

// ── Analogue inputs ───────────────────────────────────────────────────────────
// TODO: assign final ADC pins
#define PIN_OIL_PRESSURE  PC1
#define PIN_TPS           PC2
#define PIN_FUEL_1        PC3
#define PIN_FUEL_2        PC4
#define PIN_WATER_TEMP    PC5   // analog NTC

// ── Pump PWM speed command ────────────────────────────────────────────────────
// PB1 PWM -> 6N137 opto -> 2N2222 -> smart pump controller's PWM input.
// This is a SPEED COMMAND, not a motor drive. Measured pump band (see
// PumpControl.h): 0% = failsafe full, ~10% = off, 15-90% = proportional.
// NOTE: because 0% = full, a line held low commands FULL cooling, not off.
#define PIN_PUMP_PWM  PB1   // TIM3_CH4
#define PUMP_PWM_FREQ_HZ  100   // inside the pump controller's expected window

// ── Shared state (extern'd by SensorCan.h) ────────────────────────────────────
uint8_t            g_nodeStatus = NODE_STATUS_OK;
volatile bool      g_canReady   = false;
volatile CanHealth g_canHealth  = CAN_HEALTH_FAULT;
volatile uint8_t   g_pumpDuty   = 0;
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
// ─────────────────────────────────────────────────────────────────────────────
static void gpio_blanket_init() {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    GPIO_InitTypeDef cfg = {};
    cfg.Mode = GPIO_MODE_ANALOG;
    cfg.Pull = GPIO_NOPULL;

    cfg.Pin = GPIO_PIN_ALL & ~(GPIO_PIN_13 | GPIO_PIN_14);
    HAL_GPIO_Init(GPIOA, &cfg);

    cfg.Pin = GPIO_PIN_ALL;
    HAL_GPIO_Init(GPIOB, &cfg);
    HAL_GPIO_Init(GPIOC, &cfg);
    HAL_GPIO_Init(GPIOD, &cfg);
    HAL_GPIO_Init(GPIOE, &cfg);
    HAL_GPIO_Init(GPIOF, &cfg);
    HAL_GPIO_Init(GPIOG, &cfg);
    HAL_GPIO_Init(GPIOH, &cfg);
}

void setup() {
    gpio_blanket_init();

    pinMode(PIN_PUMP_PWM, OUTPUT);
    analogWriteFrequency(PUMP_PWM_FREQ_HZ);
    // Boot at minimum flow (15%); the temp algorithm takes over at 1 Hz.
    // Never command 0% — that's the controller's failsafe = full speed.
    g_pumpDuty = PUMP_DUTY_MIN;
    analogWrite(PIN_PUMP_PWM, map(g_pumpDuty, 0, 100, 0, 255));

    Serial.begin(115200);
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

    Serial.println("[SENSOR NODE] Ready");
}

void loop() {
    uint32_t now = millis();

    // ── Read sensors ──────────────────────────────────────────────────────────
    if (now - g_lastSensorRead >= SENSOR_READ_MS) {
        g_lastSensorRead = now;

        g_oilPressure = analogRead(PIN_OIL_PRESSURE);
        g_tps         = analogRead(PIN_TPS);
        g_fuel1       = analogRead(PIN_FUEL_1);
        g_fuel2       = analogRead(PIN_FUEL_2);
        g_waterTempC  = analogRead(PIN_WATER_TEMP);

        // g_pumpDuty (%) is set by the temperature algorithm at 1 Hz below.
        // analogWrite is 8-bit here, so map the percent command to 0..255.
        analogWrite(PIN_PUMP_PWM, map(g_pumpDuty, 0, 100, 0, 255));
    }

    // ── Refresh display (5 fps — decoupled from the faster sensor read) ───────
    if (now - g_lastDisplay >= DISPLAY_MS) {
        g_lastDisplay = now;
        displayUpdate(g_oilPressure, g_tps, g_fuel1, g_fuel2,
                      g_waterTempC, dallasTempC(), g_pumpDuty,
                      g_rpm, g_gear, g_canHealth);
    }

    // ── Dallas temperature → pump duty (non-blocking, 1 Hz) ───────────────────
    if (dallasUpdate(now)) {
        g_pumpDuty = pumpDutyForTemp(dallasTempC());   // temperature-scheduled pump duty
    }

    // ── CAN receive is interrupt-driven (FDCAN1_IT0 → canReceivePoll). The RX
    //    FIFO only holds 3 frames and the loop blocks tens of ms on the SPI
    //    display write; polling here let that window overrun the FIFO and drop
    //    the low-rate (on-change) gear frame. Draining in the ISR fixes that. ─

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
