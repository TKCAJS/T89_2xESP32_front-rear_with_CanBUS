/**
 * RearDisplay.h
 * Diagnostic TFT display for the rear node (ST7789 170x320, Adafruit_ST7789)
 *
 * Shows:
 *   - Current gear (large, centre)
 *   - CAN bus status
 *   - Last shift command received (UP / DOWN) with timestamp
 *   - Node status flags
 */

#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "can_ids.h"

#define DISP_BL_PIN     9   //blk
#define DISP_MOSI       13  //sda
#define DISP_SCLK       14  //scl
#define DISP_RST        12  //res
#define DISP_DC         11
#define DISP_CS         10

// Colour definitions (RGB565)
#define COL_BLACK       0x0000
#define COL_WHITE       0xFFFF
#define COL_DARKGREY    0x4208
#define COL_LIGHTGREY   0x7BEF
#define COL_GREEN       0x07E0
#define COL_ORANGE      0xFD20
#define COL_RED         0xF800

enum ShiftDirection {
    SHIFT_NONE,
    SHIFT_UP,
    SHIFT_DOWN
};

enum CanHealth {
    CAN_HEALTH_OK,       // running, ACKs received
    CAN_HEALTH_NO_BUS,   // transceiver present but no other nodes on bus
    CAN_HEALTH_NO_XCVR,  // no transceiver connected (physical layer errors)
    CAN_HEALTH_BUS_OFF,  // bus-off / recovering
    CAN_HEALTH_FAULT     // driver not running
};

class RearDisplay {
public:
    RearDisplay() :
        _tft(DISP_CS, DISP_DC, DISP_RST),
        _gear(GEAR_UNKNOWN), _canHealth(CAN_HEALTH_FAULT),
        _lastShift(SHIFT_NONE), _shiftAge(0),
        _nodeStatus(NODE_STATUS_OK), _needsFullRedraw(true),
        _gearDirty(true), _canDirty(true),
        _shiftDirty(true), _statusDirty(true) {}

    // -------------------------------------------------------------------------
    void begin() {
        pinMode(DISP_BL_PIN, OUTPUT);
        digitalWrite(DISP_BL_PIN, HIGH);

        SPI.begin(DISP_SCLK, -1, DISP_MOSI, DISP_CS);

        _tft.init(170, 320);
        _tft.setRotation(0);
        _tft.fillScreen(COL_BLACK);

        drawStaticLayout();
        _needsFullRedraw = false;

        Serial.println("[DISP] initialised");
    }

    // -------------------------------------------------------------------------
    void setGear(uint8_t gear) {
        if (gear != _gear) { _gear = gear; _gearDirty = true; }
    }

    void setCanHealth(CanHealth health) {
        if (health != _canHealth) { _canHealth = health; _canDirty = true; }
    }

    void setLastShift(ShiftDirection dir) {
        _lastShift  = dir;
        _shiftAge   = millis();
        _shiftDirty = true;
    }

    void setNodeStatus(uint8_t status) {
        if (status != _nodeStatus) { _nodeStatus = status; _statusDirty = true; }
    }

    // -------------------------------------------------------------------------
    void update() {
        if (_needsFullRedraw) {
            _tft.fillScreen(COL_BLACK);
            drawStaticLayout();
            _needsFullRedraw = false;
            _gearDirty = _canDirty = _shiftDirty = _statusDirty = true;
        }

        if (_gearDirty)   { drawGear();   _gearDirty   = false; }
        if (_canDirty)    { drawCan();    _canDirty    = false; }
        if (_shiftDirty)  { drawShift();  _shiftDirty  = false; }
        if (_statusDirty) { drawStatus(); _statusDirty = false; }

        // Clear shift indicator after 3 seconds
        if (_lastShift != SHIFT_NONE && (millis() - _shiftAge > 3000)) {
            _lastShift  = SHIFT_NONE;
            _shiftDirty = true;
        }
    }

    void forceRedraw() { _needsFullRedraw = true; }

private:
    Adafruit_ST7789 _tft;

    uint8_t        _gear;
    CanHealth      _canHealth;
    ShiftDirection _lastShift;
    uint32_t       _shiftAge;
    uint8_t        _nodeStatus;

    bool _needsFullRedraw;
    bool _gearDirty;
    bool _canDirty;
    bool _shiftDirty;
    bool _statusDirty;

    // -------------------------------------------------------------------------
    // Layout constants (170 wide, 320 tall, portrait)
    //
    //  ┌─────────────────┐  y=0
    //  │   REAR NODE     │  title bar     h=30
    //  ├─────────────────┤  y=30
    //  │                 │
    //  │      GEAR       │  large gear    h=140
    //  │                 │
    //  ├─────────────────┤  y=170
    //  │  CAN: XCVR OK   │  CAN status    h=60
    //  │       BUS OK    │
    //  ├─────────────────┤  y=230
    //  │  LAST: UP ^     │  shift event   h=40
    //  ├─────────────────┤  y=270
    //  │  STATUS: OK     │  node status   h=50
    //  └─────────────────┘  y=320

    static const int W        = 170;
    static const int Y_TITLE  = 0;
    static const int Y_GEAR   = 30;
    static const int Y_CAN    = 170;
    static const int Y_SHIFT  = 230;
    static const int Y_STATUS = 270;

    // -------------------------------------------------------------------------
    void drawTextCentred(const char *str, int cx, int y, uint8_t size, uint16_t colour) {
        _tft.setTextSize(size);
        _tft.setTextColor(colour, COL_BLACK);
        int16_t tw = strlen(str) * 6 * size;
        _tft.setCursor(cx - tw / 2, y);
        _tft.print(str);
    }

    void drawTextLeft(const char *str, int x, int y, uint8_t size, uint16_t colour) {
        _tft.setTextSize(size);
        _tft.setTextColor(colour, COL_BLACK);
        _tft.setCursor(x, y);
        _tft.print(str);
    }

    // -------------------------------------------------------------------------
    void drawStaticLayout() {
        _tft.fillRect(0, Y_TITLE, W, 30, COL_DARKGREY);
        drawTextCentred("REAR NODE", W / 2, Y_TITLE + 9, 2, COL_WHITE);

        drawTextLeft("GEAR",   4, Y_GEAR   + 2, 1, COL_LIGHTGREY);
        drawTextLeft("CAN",    4, Y_CAN    + 2, 1, COL_LIGHTGREY);
        drawTextLeft("SHIFT",  4, Y_SHIFT  + 2, 1, COL_LIGHTGREY);
        drawTextLeft("STATUS", 4, Y_STATUS + 2, 1, COL_LIGHTGREY);

        _tft.drawFastHLine(0, Y_CAN,    W, COL_LIGHTGREY);
        _tft.drawFastHLine(0, Y_SHIFT,  W, COL_LIGHTGREY);
        _tft.drawFastHLine(0, Y_STATUS, W, COL_LIGHTGREY);
    }

    // -------------------------------------------------------------------------
    void drawGear() {
        _tft.fillRect(0, Y_GEAR + 16, W, 140 - 16, COL_BLACK);

        if (_gear == GEAR_UNKNOWN) {
            drawTextCentred("?",         W / 2, Y_GEAR + 45, 7, COL_RED);
            drawTextCentred("NO SIGNAL", W / 2, Y_GEAR + 115, 1, COL_RED);
        } else if (_gear == GEAR_BETWEEN) {
            drawTextCentred("---",      W / 2, Y_GEAR + 55, 4, COL_ORANGE);
            drawTextCentred("SHIFTING", W / 2, Y_GEAR + 115, 1, COL_ORANGE);
        } else if (_gear == GEAR_NEUTRAL) {
            drawTextCentred("N", W / 2, Y_GEAR + 40, 8, COL_GREEN);
        } else {
            char buf[2];
            snprintf(buf, sizeof(buf), "%d", _gear);
            drawTextCentred(buf, W / 2, Y_GEAR + 40, 8, COL_WHITE);
        }
    }

    // -------------------------------------------------------------------------
    void drawCan() {
        _tft.fillRect(0, Y_CAN + 16, W, 44, COL_BLACK);

        // Line 1: transceiver presence
        bool xcvrOk = (_canHealth != CAN_HEALTH_NO_XCVR && _canHealth != CAN_HEALTH_FAULT);
        if (xcvrOk) {
            drawTextCentred("XCVR OK", W / 2, Y_CAN + 16, 2, COL_GREEN);
        } else {
            drawTextCentred("NO XCVR", W / 2, Y_CAN + 16, 2, COL_RED);
        }

        // Line 2: bus status
        switch (_canHealth) {
            case CAN_HEALTH_OK:
                drawTextCentred("BUS OK",  W / 2, Y_CAN + 36, 2, COL_GREEN);
                break;
            case CAN_HEALTH_NO_BUS:
                drawTextCentred("NO BUS",  W / 2, Y_CAN + 36, 2, COL_ORANGE);
                break;
            case CAN_HEALTH_BUS_OFF:
                drawTextCentred("BUS OFF", W / 2, Y_CAN + 36, 2, COL_RED);
                break;
            case CAN_HEALTH_NO_XCVR:
            case CAN_HEALTH_FAULT:
            default:
                drawTextCentred("------",  W / 2, Y_CAN + 36, 2, COL_DARKGREY);
                break;
        }
    }

    // -------------------------------------------------------------------------
    void drawShift() {
        _tft.fillRect(0, Y_SHIFT + 16, W, 24, COL_BLACK);
        switch (_lastShift) {
            case SHIFT_UP:
                drawTextCentred("UP  ^", W / 2, Y_SHIFT + 22, 2, COL_GREEN);
                break;
            case SHIFT_DOWN:
                drawTextCentred("DOWN v", W / 2, Y_SHIFT + 22, 2, COL_ORANGE);
                break;
            default:
                drawTextCentred("---", W / 2, Y_SHIFT + 22, 2, COL_LIGHTGREY);
                break;
        }
    }

    // -------------------------------------------------------------------------
    void drawStatus() {
        _tft.fillRect(0, Y_STATUS + 16, W, 34, COL_BLACK);
        if (_nodeStatus == NODE_STATUS_OK) {
            drawTextCentred("OK", W / 2, Y_STATUS + 22, 2, COL_GREEN);
            return;
        }
        int y = Y_STATUS + 18;
        if (_nodeStatus & NODE_STATUS_SENSOR_ERR)   { drawTextLeft("SENSOR ERR",   4, y, 1, COL_RED); y += 10; }
        if (_nodeStatus & NODE_STATUS_CAN_ERR)      { drawTextLeft("CAN ERR",      4, y, 1, COL_RED); y += 10; }
        if (_nodeStatus & NODE_STATUS_ACTUATOR_ERR) { drawTextLeft("ACTUATOR ERR", 4, y, 1, COL_RED); y += 10; }
        if (_nodeStatus & NODE_STATUS_OVERTEMP)     { drawTextLeft("OVERTEMP",     4, y, 1, COL_RED); y += 10; }
        if (_nodeStatus & NODE_STATUS_WATCHDOG)     { drawTextLeft("WATCHDOG RST", 4, y, 1, COL_RED); y += 10; }
        if (_nodeStatus & NODE_STATUS_CONFIG_ERR)   { drawTextLeft("CONFIG ERR",   4, y, 1, COL_RED); }
    }
};
