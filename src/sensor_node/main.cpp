#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "can_ids.h"

// ── Display (ST7735 128×160, SPI2) ────────────────────────────────────────────
// TODO: assign final SPI2 pins once PCB layout is finalised (avoid SPI1 PA4-PA7)
#define TFT_CS   PB12
#define TFT_DC   PB13
#define TFT_RST  PB14

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// ── One-wire / Dallas temperature ─────────────────────────────────────────────
// TODO: assign final pin
#define ONEWIRE_PIN  PC0

OneWire oneWire(ONEWIRE_PIN);
DallasTemperature dallas(&oneWire);

// ── Analogue inputs ───────────────────────────────────────────────────────────
// TODO: assign final ADC pins (avoid PA4-PA7 SPI1, reserved board pins)
#define PIN_OIL_PRESSURE  PC1
#define PIN_TPS           PC2
#define PIN_FUEL_1        PC3
#define PIN_FUEL_2        PC4
#define PIN_WATER_TEMP    PC5   // analog NTC

// ── Pump PWM (via MOSFET, 12 V) ───────────────────────────────────────────────
// Hardware pull-down on gate holds it LOW from power-on through the boot window.
// TODO: assign a TIMx-capable pin
#define PIN_PUMP_PWM  PB0

// ── State ─────────────────────────────────────────────────────────────────────
static float   g_oilPressure = 0.0f;
static float   g_tps         = 0.0f;
static float   g_fuel1       = 0.0f;
static float   g_fuel2       = 0.0f;
static float   g_waterTempC  = 0.0f;
static float   g_dallasTemp  = 0.0f;
static uint8_t g_pumpDuty    = 0;

// ─────────────────────────────────────────────────────────────────────────────
// GPIO init — blanket analog pass FIRST, peripheral configs follow in setup()
// ─────────────────────────────────────────────────────────────────────────────
static void gpio_blanket_init() {
    // Step 1: enable all GPIO port clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    // Step 2: set every pin to analog/no-pull (lowest power, best EMI)
    // Step 3: exclude PA13 (SWDIO) and PA14 (SWCLK) to preserve SWD
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

    // Step 4: peripheral pin configuration happens in setup() via library inits,
    // which override the analog blanket on their specific pins only.
}

void setup() {
    // Blanket pass must be first — before any peripheral or library init
    gpio_blanket_init();

    // Pump to safe state immediately (hardware pull-down already holds gate low)
    pinMode(PIN_PUMP_PWM, OUTPUT);
    analogWrite(PIN_PUMP_PWM, 0);

    // Debug UART (USART1, PA9/PA10)
    Serial.begin(115200);

    // Display
    tft.initR(INITR_BLACKTAB);
    tft.setRotation(1);
    tft.fillScreen(ST7735_BLACK);
    tft.setTextColor(ST7735_WHITE);
    tft.setTextSize(1);
    tft.setCursor(0, 0);
    tft.println("Sensor Node");
    tft.println("Init OK");

    // Dallas one-wire temperature
    dallas.begin();

    // TODO: initialise FDCAN1 at 500 Kbps, 29-bit extended frames (PB7 TX, PB8 RX)
}

void loop() {
    // ── Read analogue sensors ─────────────────────────────────────────────────
    g_oilPressure = analogRead(PIN_OIL_PRESSURE);
    g_tps         = analogRead(PIN_TPS);
    g_fuel1       = analogRead(PIN_FUEL_1);
    g_fuel2       = analogRead(PIN_FUEL_2);
    g_waterTempC  = analogRead(PIN_WATER_TEMP);

    dallas.requestTemperatures();
    g_dallasTemp = dallas.getTempCByIndex(0);

    // ── Pump PWM ──────────────────────────────────────────────────────────────
    // TODO: closed-loop control based on temperature
    analogWrite(PIN_PUMP_PWM, g_pumpDuty);

    // ── CAN broadcast ─────────────────────────────────────────────────────────
    // TODO: send CAN frames using IDs from can_ids.h:
    //   CAN_SENS_OIL_PRESSURE, CAN_SENS_WATER_TEMP, CAN_SENS_TPS,
    //   CAN_SENS_SPEED, CAN_SENS_FUEL_1, CAN_SENS_FUEL_2

    // ── Update display ────────────────────────────────────────────────────────
    // TODO: render live sensor readings to ST7735

    delay(50);
}
