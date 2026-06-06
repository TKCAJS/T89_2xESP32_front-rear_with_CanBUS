#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "can_ids.h"

// ── Display (ST7735 128×160, SPI2) ────────────────────────────────────────────
// Module pins: RST=PC7, CS=PB12, DC=PC6, DIN=PB15, CLK=PB13, VCC=3.3V, BL=3.3V, GND
// Confirmed working: INITR_REDTAB, patchOffset(0,1), setRotation(3) → 160×128 landscape
#define TFT_CS   PB12
#define TFT_DC   PC6
#define TFT_RST  PC7

// INITR_REDTAB == INITR_144GREENTAB == 0x01 causes setRotation to set _width=128.
// Override fixes dimensions. patchOffset exposes protected setColRowStart() to
// shift the panel window (rowstart=1 eliminates edge garbage in this module).
class ST7735Patched : public Adafruit_ST7735 {
public:
    ST7735Patched(SPIClass *spi, int8_t cs, int8_t dc, int8_t rst)
        : Adafruit_ST7735(spi, cs, dc, rst) {}
    void patchOffset(int8_t col, int8_t row) { setColRowStart(col, row); }
    void setRotation(uint8_t m) override {
        Adafruit_ST7735::setRotation(m);
        if (m & 1) { _width = 160; _height = 128; }
        else        { _width = 128; _height = 160; }
    }
};

SPIClass SPI_2(PB15, PB14, PB13);   // MOSI=DIN, MISO(NC), SCK=CLK
ST7735Patched tft(&SPI_2, TFT_CS, TFT_DC, TFT_RST);

// ── One-wire / Dallas temperature ─────────────────────────────────────────────
// TODO: assign final pin
#define ONEWIRE_PIN  PC0

OneWire oneWire(ONEWIRE_PIN);
DallasTemperature dallas(&oneWire);

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
    analogWrite(PIN_PUMP_PWM, 0);

    Serial.begin(115200);

    // Display
    tft.initR(INITR_REDTAB);
    tft.patchOffset(0, 1);  // rowstart=1 for this module's panel window
    tft.setRotation(0);     // portrait 128×160 — clean edges, rotate module 90° on mount
    tft.fillScreen(ST7735_BLACK);
    tft.setTextColor(ST7735_WHITE);
    tft.setTextSize(2);
    tft.setCursor(0, 0);
    tft.println("Sensor Node");
    tft.println("Init OK");

    dallas.begin();

    // TODO: initialise FDCAN1 at 500 Kbps, 29-bit extended frames (PB7=TX, PB8=RX)
}

void loop() {
    g_oilPressure = analogRead(PIN_OIL_PRESSURE);
    g_tps         = analogRead(PIN_TPS);
    g_fuel1       = analogRead(PIN_FUEL_1);
    g_fuel2       = analogRead(PIN_FUEL_2);
    g_waterTempC  = analogRead(PIN_WATER_TEMP);

    dallas.requestTemperatures();
    g_dallasTemp = dallas.getTempCByIndex(0);

    analogWrite(PIN_PUMP_PWM, g_pumpDuty);

    // TODO: send CAN frames — CAN_SENS_OIL_PRESSURE, CAN_SENS_WATER_TEMP,
    //       CAN_SENS_TPS, CAN_SENS_SPEED, CAN_SENS_FUEL_1, CAN_SENS_FUEL_2

    // TODO: render live sensor readings to ST7735

    delay(50);
}
