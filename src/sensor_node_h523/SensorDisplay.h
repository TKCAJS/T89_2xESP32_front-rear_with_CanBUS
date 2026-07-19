#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "can_ids.h"

// H523 pins: RST=PB4, CS=PB12, DC=PA15, DIN=PB15, CLK=PB13 — identical to the
// F103 build. DC/RST moved off PB10/PB2 so all six display lines sit on the
// same board edge as the SPI2 block (WeAct header splits PB2/PB10 onto the
// opposite side). PA15/PB4 are otherwise JTAG-only; pinMode() releases them,
// SWD (PA13/PA14) unaffected. PB3/PB5 stay reserved for the planned fan
// relays. PB14 stays claimed as SPI2 MISO — unused but the SPI class owns it.
#define TFT_CS   PB12
#define TFT_DC   PA15
#define TFT_RST  PB4

#define COL_BLACK      0x0000
#define COL_WHITE      0xFFFF
#define COL_DARKGREY   0x4208
#define COL_LIGHTGREY  0x7BEF
#define COL_GREEN      0x07E0
#define COL_ORANGE     0xFD20
#define COL_RED        0xF800

enum CanHealth {
    CAN_HEALTH_OK,
    CAN_HEALTH_NO_BUS,
    CAN_HEALTH_NO_XCVR,
    CAN_HEALTH_BUS_OFF,
    CAN_HEALTH_FAULT
};

static SPIClass        s_spi2(PB15, PB14, PB13);   // MOSI=DIN, MISO(NC), SCK=CLK
static Adafruit_ST7789 s_tft(&s_spi2, TFT_CS, TFT_DC, TFT_RST);

// 2.4" ST7789, 240x320 portrait. Header + 9 data rows × 32px = 320px.
static const int DISP_W     = 240;
static const int HEADER_H   = 32;
static const int DISP_ROW_H = 32;
static const int VAL_X      = 120;   // value column start; labels live left of this
static const int TXT_SZ     = 2;     // 12x16 px glyphs
static const int TXT_Y_OFF  = (DISP_ROW_H - 16) / 2;   // vertical centre within row

// Cached values — initialised out-of-range to force first draw
static float     s_d_oil    = -99.0f;
static float     s_d_tps    = -99.0f;
static float     s_d_fuel1  = -99.0f;
static float     s_d_fuel2  = -99.0f;
static float     s_d_eng    = -99.0f;
static float     s_d_radout = -99.0f;
static uint8_t   s_d_pump   = 0xFF;
static uint16_t  s_d_rpm    = 0xFFFF;
static uint8_t   s_d_gear   = GEAR_UNKNOWN;
static CanHealth s_d_can    = (CanHealth)0xFF;

// ── Internal helpers ──────────────────────────────────────────────────────────

// Deadband changed-check: raw ADC reads jitter by a few LSB every cycle, so a
// plain != would repaint every row on every refresh — and each row repaint is a
// large SPI fillRect that blocks the loop. Only repaint when the value moves
// enough to matter on screen.
static const float EPS_RAW = 8.0f;    // raw 12-bit ADC counts (~0.2% FS)
static const float EPS_C   = 0.1f;    // °C — matches the displayed 0.1 resolution

static bool _dirty(float val, float& cached, float eps) {
    if (fabsf(val - cached) < eps) return false;
    cached = val;
    return true;
}

static void _dispRow(int row, float val, const char* unit = "") {
    int y = HEADER_H + row * DISP_ROW_H;
    s_tft.fillRect(VAL_X, y, DISP_W - VAL_X, DISP_ROW_H, COL_BLACK);
    char buf[16];
    // Integer fixed-point ×10 render — the only %f in this build lived here,
    // and dropping it lets the env omit _printf_float (~8 KB of flash).
    int v10 = (int)(val * 10.0f + (val >= 0 ? 0.5f : -0.5f));
    snprintf(buf, sizeof(buf), "%s%d.%d%s",
             (v10 < 0) ? "-" : "", abs(v10) / 10, abs(v10) % 10, unit);
    s_tft.setTextSize(TXT_SZ);
    s_tft.setTextColor(COL_WHITE, COL_BLACK);
    s_tft.setCursor(VAL_X + 2, y + TXT_Y_OFF);
    s_tft.print(buf);
}

static void _dispRowU16(int row, uint16_t val) {
    int y = HEADER_H + row * DISP_ROW_H;
    s_tft.fillRect(VAL_X, y, DISP_W - VAL_X, DISP_ROW_H, COL_BLACK);
    char buf[8];
    snprintf(buf, sizeof(buf), "%u", val);
    s_tft.setTextSize(TXT_SZ);
    s_tft.setTextColor(COL_WHITE, COL_BLACK);
    s_tft.setCursor(VAL_X + 2, y + TXT_Y_OFF);
    s_tft.print(buf);
}

static void _dispGear(int row, uint8_t gear) {
    int y = HEADER_H + row * DISP_ROW_H;
    s_tft.fillRect(VAL_X, y, DISP_W - VAL_X, DISP_ROW_H, COL_BLACK);
    char c;
    if      (gear == GEAR_NEUTRAL)                      c = 'N';
    else if (gear >= GEAR_1 && gear <= GEAR_6)          c = '0' + gear;
    else if (gear == GEAR_BETWEEN)                      c = '-';
    else                                                c = '?';
    s_tft.setTextSize(TXT_SZ);
    s_tft.setTextColor(COL_WHITE, COL_BLACK);
    s_tft.setCursor(VAL_X + 2, y + TXT_Y_OFF);
    s_tft.print(c);
}

static void _dispCan(CanHealth h) {
    const char* txt;
    uint16_t col;
    switch (h) {
        case CAN_HEALTH_OK:      txt = "OK"; col = COL_GREEN;     break;
        case CAN_HEALTH_NO_BUS:  txt = "NB"; col = COL_ORANGE;    break;
        case CAN_HEALTH_NO_XCVR: txt = "NX"; col = COL_RED;       break;
        case CAN_HEALTH_BUS_OFF: txt = "BO"; col = COL_RED;       break;
        default:                 txt = "--"; col = COL_LIGHTGREY;  break;
    }
    s_tft.fillRect(150, 0, DISP_W - 150, HEADER_H, COL_DARKGREY);
    s_tft.setTextSize(TXT_SZ);
    s_tft.setTextColor(COL_LIGHTGREY, COL_DARKGREY);
    s_tft.setCursor(156, TXT_Y_OFF);
    s_tft.print("CAN:");
    s_tft.setTextColor(col, COL_DARKGREY);
    s_tft.print(txt);
}

// ── Public interface ──────────────────────────────────────────────────────────

void displayBegin() {
    s_tft.init(240, 320);   // ST7789 240x320
    s_tft.setRotation(2);   // portrait, flipped 180° — rotate module on mount for preferred orientation
    s_tft.fillScreen(COL_BLACK);

    s_tft.fillRect(0, 0, DISP_W, HEADER_H, COL_DARKGREY);
    s_tft.setTextSize(TXT_SZ);
    s_tft.setTextColor(COL_WHITE, COL_DARKGREY);
    s_tft.setCursor(4, TXT_Y_OFF);
    s_tft.print("SENSOR NODE");

    s_tft.setTextSize(TXT_SZ);
    s_tft.setTextColor(COL_LIGHTGREY, COL_BLACK);
    const char* labels[] = { "OIL:", "TPS:", "FUEL1:", "FUEL2:", "ENG:", "RADO:", "PUMP:", "RPM:", "GEAR:" };
    for (int i = 0; i < 9; i++) {
        s_tft.setCursor(4, HEADER_H + i * DISP_ROW_H + TXT_Y_OFF);
        s_tft.print(labels[i]);
    }
}

// Gear fast path — called every loop pass so a gear change repaints immediately
// instead of waiting for the next full refresh tick. A single glyph is cheap on
// SPI, unlike the full-row repaints in displayUpdate().
void displayUpdateGear(uint8_t gear) {
    if (gear != s_d_gear) {
        s_d_gear = gear;
        _dispGear(8, gear);
    }
}

void displayUpdate(float oilPressure, float tps, float fuel1, float fuel2,
                   float engineTempC, float radOutTempC, uint8_t pumpDuty,
                   uint16_t rpm, uint8_t gear, CanHealth canHealth) {
    if (canHealth != s_d_can) { s_d_can = canHealth; _dispCan(canHealth); }
    if (_dirty(oilPressure, s_d_oil,    EPS_RAW)) _dispRow(0, oilPressure);
    if (_dirty(tps,         s_d_tps,    EPS_RAW)) _dispRow(1, tps);
    if (_dirty(fuel1,       s_d_fuel1,  EPS_RAW)) _dispRow(2, fuel1);
    if (_dirty(fuel2,       s_d_fuel2,  EPS_RAW)) _dispRow(3, fuel2);
    if (_dirty(engineTempC, s_d_eng,    EPS_C))   _dispRow(4, engineTempC, "C");
    if (_dirty(radOutTempC, s_d_radout, EPS_C))   _dispRow(5, radOutTempC, "C");
    if (rpm != s_d_rpm) { s_d_rpm = rpm; _dispRowU16(7, rpm); }
    displayUpdateGear(gear);
    if (pumpDuty     != s_d_pump) {
        s_d_pump = pumpDuty;
        int y = HEADER_H + 6 * DISP_ROW_H;
        s_tft.fillRect(VAL_X, y, DISP_W - VAL_X, DISP_ROW_H, COL_BLACK);
        char buf[8];
        snprintf(buf, sizeof(buf), "%u%%", pumpDuty);
        s_tft.setTextSize(TXT_SZ);
        s_tft.setTextColor(COL_WHITE, COL_BLACK);
        s_tft.setCursor(VAL_X + 2, y + TXT_Y_OFF);
        s_tft.print(buf);
    }
}
