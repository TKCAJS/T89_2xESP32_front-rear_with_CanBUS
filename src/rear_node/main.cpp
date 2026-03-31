/**
 * rear_node/main.cpp
 * Honda CBR1000RR Racing Telemetry — Rear Node (NODE_REAR 0x03)
 *
 * Hardware:
 *   - ESP32-S3
 *   - SN65HVD230 CAN transceiver  TX→GPIO4, RX→GPIO5
 *   - Rotary gear switch: 7 pins, active LOW, internal pullup
 *       Neutral→GPIO6, G1→GPIO7, G2→GPIO8, G3→GPIO9,
 *       G4→GPIO10, G5→GPIO11, G6→GPIO12
 *   - ST7789 170x320 TFT (Adafruit_ST7789)
 *       MOSI→13, SCLK→14, CS→16, DC→17, RST→18, BL→15
 *   - Relay outputs (active HIGH)
 *       Upshift→GPIO21, Downshift→GPIO47, Ignition cut→GPIO48
 *
 * CAN frames transmitted:
 *   CAN_REAR_GEAR_POS   immediately on change + 1 Hz refresh
 *   CAN_REAR_GEAR_RAW   10 Hz debug
 *   CAN_REAR_STATUS     1 Hz
 *   CAN_HB_REAR         1 Hz heartbeat
 *
 * CAN frames received:
 *   CAN_REAR_CMD_SHIFT_UP   logged to display
 *   CAN_REAR_CMD_SHIFT_DN   logged to display
 */

#include <Arduino.h>
#include <SPI.h>
#include "esp_task_wdt.h"
#include "can_ids.h"
#include "RearDisplay.h"
#include "RearCan.h"
#include "GearShift.h"

// =============================================================================
// PIN CONFIGURATION
// =============================================================================

#define PIN_GEAR_NEUTRAL    6
#define PIN_GEAR_1          7
#define PIN_GEAR_2          8
#define PIN_GEAR_3          9
#define PIN_GEAR_4          10
#define PIN_GEAR_5          11
#define PIN_GEAR_6          12

// =============================================================================
// TIMING
// =============================================================================

#define GEAR_POLL_MS            10
#define GEAR_DEBOUNCE_MS        30
#define CAN_GEAR_POS_INTERVAL   1000
#define CAN_GEAR_RAW_INTERVAL   100
#define CAN_STATUS_INTERVAL     1000
#define CAN_HEARTBEAT_INTERVAL  CAN_HEARTBEAT_INTERVAL_MS
#define DISPLAY_UPDATE_MS       50      // 20 Hz display refresh

// =============================================================================
// GLOBALS
// =============================================================================

static const uint8_t GEAR_PINS[7] = {
    PIN_GEAR_NEUTRAL, PIN_GEAR_1, PIN_GEAR_2, PIN_GEAR_3,
    PIN_GEAR_4, PIN_GEAR_5, PIN_GEAR_6
};

uint8_t  g_currentGear   = GEAR_UNKNOWN;
static uint8_t  g_rawPins       = 0xFF;

static uint8_t  g_pendingGear   = GEAR_UNKNOWN;
static uint32_t g_debounceStart = 0;

static uint32_t g_lastGearPoll   = 0;
static uint32_t g_lastGearPosTx  = 0;
static uint32_t g_lastGearRawTx  = 0;
static uint32_t g_lastStatusTx   = 0;
static uint32_t g_lastHeartbeat  = 0;
static uint32_t g_lastDisplayUpd = 0;
static uint32_t g_lastCanHealth  = 0;

// Shared with RearCan.h
uint8_t    g_nodeStatus  = NODE_STATUS_OK;
bool       g_canReady    = false;
CanHealth  g_canHealth   = CAN_HEALTH_FAULT;
RearDisplay g_display;

// =============================================================================
// GEAR SENSOR
// =============================================================================

void gearPinsInit() {
    for (int i = 0; i < 7; i++) {
        pinMode(GEAR_PINS[i], INPUT_PULLUP);
    }
}

uint8_t gearPinsRead() {
    int activePin   = -1;
    int activeCount = 0;
    uint8_t rawMask = 0;

    for (int i = 0; i < 7; i++) {
        if (digitalRead(GEAR_PINS[i]) == LOW) {
            rawMask |= (1 << i);
            activePin = i;
            activeCount++;
        }
    }
    g_rawPins = rawMask;

    if (activeCount == 0) return GEAR_UNKNOWN;
    if (activeCount > 1)  return GEAR_BETWEEN;
    return (uint8_t)activePin;
}

bool gearUpdate() {
    uint8_t reading = gearPinsRead();

    if (reading != g_pendingGear) {
        g_pendingGear   = reading;
        g_debounceStart = millis();
        return false;
    }
    if ((millis() - g_debounceStart) < GEAR_DEBOUNCE_MS) return false;
    if (reading == g_currentGear) return false;

    g_currentGear = reading;
    return true;
}

// =============================================================================
// TASKS
// =============================================================================

void taskGearPoll(uint32_t now) {
    if (now - g_lastGearPoll < GEAR_POLL_MS) return;
    g_lastGearPoll = now;
    if (gearUpdate()) {
        g_display.setGear(g_currentGear);
        if (g_canReady) {
            sendGearPos(g_currentGear);
            g_lastGearPosTx = now;
        }
    }
}

void taskCanHealth(uint32_t now) {
    if (!(g_canReady || g_canHealth != CAN_HEALTH_FAULT)) return;
    if (g_lastCanHealth == 0) g_lastCanHealth = now;
    if (now - g_lastCanHealth < 500 || now < 2000) return;
    g_lastCanHealth = now;
    canHealthPoll();
}

void taskCanTx(uint32_t now) {
    if (!g_canReady) return;
    canReceivePoll();
    if (now - g_lastGearPosTx  >= CAN_GEAR_POS_INTERVAL)  { sendGearPos(g_currentGear);            g_lastGearPosTx  = now; }
    if (now - g_lastGearRawTx  >= CAN_GEAR_RAW_INTERVAL)  { sendGearRaw(g_rawPins, g_currentGear); g_lastGearRawTx  = now; }
    if (now - g_lastStatusTx   >= CAN_STATUS_INTERVAL)    { sendStatus(g_currentGear, g_rawPins);  g_lastStatusTx   = now; }
    if (now - g_lastHeartbeat  >= CAN_HEARTBEAT_INTERVAL) { sendHeartbeat();                       g_lastHeartbeat  = now; }
}

void taskGearShift() {
    gearShiftUpdate();
    ShiftResult result = gearShiftTakeResult();
    if (result.ready && g_canReady) {
        sendShiftAck(result);
    }
}

void taskDisplay(uint32_t now) {
    g_display.setCanHealth(g_canHealth);
    g_display.setNodeStatus(g_nodeStatus);
    if (now - g_lastDisplayUpd < DISPLAY_UPDATE_MS) return;
    g_lastDisplayUpd = now;
    g_display.update();
}

// =============================================================================
// SETUP / LOOP
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(500);

    // Extend watchdog timeout to 10 seconds
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms     = 10000,
        .idle_core_mask = 0,
        .trigger_panic  = false
    };
    esp_task_wdt_reconfigure(&wdt_config);

    Serial.println("[REAR NODE] Booting...");

    gearShiftInit();
    Serial.println("[RELAY] Pins initialised");

    gearPinsInit();
    Serial.println("[GEAR] Pins initialised");

    g_canReady = canInit();
    if (!g_canReady) {
        g_canHealth   = CAN_HEALTH_FAULT;
        g_nodeStatus |= NODE_STATUS_CAN_ERR;
        Serial.println("[REAR NODE] WARNING: CAN init failed");
    }

    g_display.begin();
    g_display.setCanHealth(g_canHealth);
    g_display.setGear(g_currentGear);
    g_display.update();

    Serial.println("[REAR NODE] Ready");
}

void loop() {
    esp_task_wdt_reset();
    yield();

    uint32_t now = millis();
    taskGearShift();
    taskGearPoll(now);
    taskCanHealth(now);
    taskCanTx(now);
    taskDisplay(now);
}
