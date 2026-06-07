#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include "can_ids.h"

// Module pins: RST=PC7, CS=PB12, DC=PC6, DIN=PB15, CLK=PB13
#define TFT_CS   PB12
#define TFT_DC   PC6
#define TFT_RST  PC7

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

// INITR_REDTAB causes setRotation to set _width=128 regardless of orientation.
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

static SPIClass      s_spi2(PB15, PB14, PB13);   // MOSI=DIN, MISO(NC), SCK=CLK
static ST7735Patched s_tft(&s_spi2, TFT_CS, TFT_DC, TFT_RST);

static const int DISP_W     = 128;
static const int DISP_ROW_H = 16;   // 10 rows × 16px = 160px — header + 9 data rows

// Cached values — initialised out-of-range to force first draw
static float     s_d_oil    = -99.0f;
static float     s_d_tps    = -99.0f;
static float     s_d_fuel1  = -99.0f;
static float     s_d_fuel2  = -99.0f;
static float     s_d_water  = -99.0f;
static float     s_d_dallas = -99.0f;
static uint8_t   s_d_pump   = 0xFF;
static uint16_t  s_d_rpm    = 0xFFFF;
static uint8_t   s_d_gear   = GEAR_UNKNOWN;
static CanHealth s_d_can    = (CanHealth)0xFF;

// ── Internal helpers ──────────────────────────────────────────────────────────

static void _dispRow(int row, float val, const char* unit = "") {
    int y = DISP_ROW_H + row * DISP_ROW_H;
    s_tft.fillRect(50, y, DISP_W - 50, DISP_ROW_H, COL_BLACK);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f%s", val, unit);
    s_tft.setTextSize(1);
    s_tft.setTextColor(COL_WHITE, COL_BLACK);
    s_tft.setCursor(52, y + 6);
    s_tft.print(buf);
}

static void _dispRowU16(int row, uint16_t val) {
    int y = DISP_ROW_H + row * DISP_ROW_H;
    s_tft.fillRect(50, y, DISP_W - 50, DISP_ROW_H, COL_BLACK);
    char buf[8];
    snprintf(buf, sizeof(buf), "%u", val);
    s_tft.setTextSize(1);
    s_tft.setTextColor(COL_WHITE, COL_BLACK);
    s_tft.setCursor(52, y + 6);
    s_tft.print(buf);
}

static void _dispGear(int row, uint8_t gear) {
    int y = DISP_ROW_H + row * DISP_ROW_H;
    s_tft.fillRect(50, y, DISP_W - 50, DISP_ROW_H, COL_BLACK);
    char c;
    if      (gear == GEAR_NEUTRAL)                      c = 'N';
    else if (gear >= GEAR_1 && gear <= GEAR_6)          c = '0' + gear;
    else if (gear == GEAR_BETWEEN)                      c = '-';
    else                                                c = '?';
    s_tft.setTextSize(1);
    s_tft.setTextColor(COL_WHITE, COL_BLACK);
    s_tft.setCursor(52, y + 6);
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
    s_tft.fillRect(90, 0, DISP_W - 90, DISP_ROW_H, COL_DARKGREY);
    s_tft.setTextSize(1);
    s_tft.setTextColor(COL_LIGHTGREY, COL_DARKGREY);
    s_tft.setCursor(90, 7);
    s_tft.print("CAN:");
    s_tft.setTextColor(col, COL_DARKGREY);
    s_tft.print(txt);
}

// ── Public interface ──────────────────────────────────────────────────────────

void displayBegin() {
    s_tft.initR(INITR_REDTAB);
    s_tft.patchOffset(0, 1);
    s_tft.setRotation(0);   // portrait 128x160 — rotate module 90° on mount
    s_tft.fillScreen(COL_BLACK);

    s_tft.fillRect(0, 0, DISP_W, DISP_ROW_H, COL_DARKGREY);
    s_tft.setTextSize(1);
    s_tft.setTextColor(COL_WHITE, COL_DARKGREY);
    s_tft.setCursor(14, 7);
    s_tft.print("SENSOR NODE");

    s_tft.setTextColor(COL_LIGHTGREY, COL_BLACK);
    const char* labels[] = { "OIL:", "TPS:", "FUEL1:", "FUEL2:", "WATER:", "DALL:", "PUMP:", "RPM:", "GEAR:" };
    for (int i = 0; i < 9; i++) {
        s_tft.setCursor(2, DISP_ROW_H + i * DISP_ROW_H + 6);
        s_tft.print(labels[i]);
    }
}

void displayUpdate(float oilPressure, float tps, float fuel1, float fuel2,
                   float waterTempC, float dallasTemp, uint8_t pumpDuty,
                   uint16_t rpm, uint8_t gear, CanHealth canHealth) {
    if (canHealth    != s_d_can)    { s_d_can    = canHealth;    _dispCan(canHealth);             }
    if (oilPressure  != s_d_oil)    { s_d_oil    = oilPressure;  _dispRow(0, oilPressure);        }
    if (tps          != s_d_tps)    { s_d_tps    = tps;          _dispRow(1, tps);                }
    if (fuel1        != s_d_fuel1)  { s_d_fuel1  = fuel1;        _dispRow(2, fuel1);              }
    if (fuel2        != s_d_fuel2)  { s_d_fuel2  = fuel2;        _dispRow(3, fuel2);              }
    if (waterTempC   != s_d_water)  { s_d_water  = waterTempC;   _dispRow(4, waterTempC,  "C");   }
    if (dallasTemp   != s_d_dallas) { s_d_dallas = dallasTemp;   _dispRow(5, dallasTemp,  "C");   }
    if (rpm          != s_d_rpm)    { s_d_rpm    = rpm;          _dispRowU16(7, rpm);             }
    if (gear         != s_d_gear)   { s_d_gear   = gear;         _dispGear(8, gear);              }
    if (pumpDuty     != s_d_pump) {
        s_d_pump = pumpDuty;
        int y = DISP_ROW_H + 6 * DISP_ROW_H;
        s_tft.fillRect(50, y, DISP_W - 50, DISP_ROW_H, COL_BLACK);
        char buf[8];
        snprintf(buf, sizeof(buf), "%u%%", pumpDuty);
        s_tft.setTextSize(1);
        s_tft.setTextColor(COL_WHITE, COL_BLACK);
        s_tft.setCursor(52, y + 6);
        s_tft.print(buf);
    }
}
