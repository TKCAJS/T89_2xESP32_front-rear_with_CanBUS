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

// ── Pump PWM (via MOSFET, 12 V) ───────────────────────────────────────────────
// Hardware pull-down on gate holds it LOW from power-on through the boot window.
#define PIN_PUMP_PWM  PB0   // TIM3_CH3
// PWM carrier frequency. 25 kHz suits the 4-wire PC fan used for bench testing;
// the real pump wants 100 Hz — change back to 100 when driving the pump.
#define PUMP_PWM_FREQ_HZ  25000

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
    analogWrite(PIN_PUMP_PWM, 0);

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

        analogWrite(PIN_PUMP_PWM, g_pumpDuty);
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
