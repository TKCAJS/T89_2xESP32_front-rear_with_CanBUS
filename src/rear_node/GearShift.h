/**
 * GearShift.h
 * Gear change actuator for the rear node.
 *
 * Shift sequence:
 *   1. Fire ignition cut relay for ign_cut_ms
 *   2. Fire shift relay (up or down) for shift_ms
 *   3. Release both relays
 *   4. Wait SHIFT_SETTLE_MS for gear sensor to debounce
 *   5. Read actual gear, compare to expected, set result
 *
 * Both durations are specified in the incoming CAN packet.
 * Limits are enforced: shift 30–200 ms, ign cut 10–100 ms.
 *
 * Non-blocking — call gearShiftUpdate() every loop iteration.
 * When a shift completes, gearShiftTakeResult() returns the outcome once.
 */

#pragma once
#include <Arduino.h>
#include "can_ids.h"

#define PIN_RELAY_UPSHIFT   21
#define PIN_RELAY_DOWNSHIFT 47
#define PIN_RELAY_IGN_CUT   48

#define SHIFT_MS_MIN        30
#define SHIFT_MS_MAX        200
#define IGN_CUT_MS_MIN      10
#define IGN_CUT_MS_MAX      100
#define SHIFT_SETTLE_MS     100     // wait after relays release before reading gear

// Actual gear after shift is read from main.cpp
extern uint8_t g_currentGear;

// =============================================================================

enum ShiftState {
    SHIFT_IDLE,
    SHIFT_IGN_CUT,      // ignition cut relay active
    SHIFT_ACTUATING,    // shift relay active
    SHIFT_SETTLING,     // relays released, waiting for sensor to settle
    SHIFT_COMPLETE
};

struct ShiftResult {
    bool    ready;
    uint8_t dir;
    uint8_t expectedGear;
    uint8_t actualGear;
    bool    success;
};

static ShiftState  s_shiftState    = SHIFT_IDLE;
static uint8_t     s_shiftDir      = 0;
static uint16_t    s_shiftMs       = 0;
static uint16_t    s_ignCutMs      = 0;
static uint32_t    s_shiftStateTs  = 0;
static uint8_t     s_expectedGear  = GEAR_UNKNOWN;
static ShiftResult s_shiftResult   = { false, 0, 0, 0, false };

// =============================================================================

void gearShiftInit() {
    pinMode(PIN_RELAY_UPSHIFT,   OUTPUT); digitalWrite(PIN_RELAY_UPSHIFT,   LOW);
    pinMode(PIN_RELAY_DOWNSHIFT, OUTPUT); digitalWrite(PIN_RELAY_DOWNSHIFT, LOW);
    pinMode(PIN_RELAY_IGN_CUT,   OUTPUT); digitalWrite(PIN_RELAY_IGN_CUT,   LOW);
}

// Called from canReceivePoll() when a shift command arrives.
// current_gear: gear position at time of command, used to compute expected result
void gearShiftRequest(uint8_t dir, uint16_t shift_ms, uint16_t ign_cut_ms, uint8_t current_gear) {
    if (s_shiftState != SHIFT_IDLE) {
        Serial.println("[SHIFT] Ignored — shift already in progress");
        return;
    }

    s_shiftDir  = dir;
    s_shiftMs   = constrain(shift_ms,   SHIFT_MS_MIN,   SHIFT_MS_MAX);
    s_ignCutMs  = constrain(ign_cut_ms, IGN_CUT_MS_MIN, IGN_CUT_MS_MAX);

    if (dir == SHIFT_UP) {
        s_expectedGear = (current_gear < GEAR_6) ? current_gear + 1 : GEAR_6;
    } else {
        s_expectedGear = (current_gear > GEAR_NEUTRAL) ? current_gear - 1 : GEAR_NEUTRAL;
    }

    Serial.printf("[SHIFT] %s  shift=%ums  ign_cut=%ums  expect_gear=%u\n",
        (dir == SHIFT_UP) ? "UP" : "DOWN", s_shiftMs, s_ignCutMs, s_expectedGear);

    digitalWrite(PIN_RELAY_IGN_CUT, HIGH);
    s_shiftStateTs = millis();
    s_shiftState   = SHIFT_IGN_CUT;
}

bool gearShiftBusy() {
    return s_shiftState != SHIFT_IDLE;
}

// Returns the shift result once after each completed shift, then clears it.
ShiftResult gearShiftTakeResult() {
    ShiftResult r = s_shiftResult;
    s_shiftResult.ready = false;
    return r;
}

// Call every loop iteration
void gearShiftUpdate() {
    if (s_shiftState == SHIFT_IDLE) return;

    uint32_t elapsed = millis() - s_shiftStateTs;

    switch (s_shiftState) {
        case SHIFT_IGN_CUT:
            if (elapsed >= s_ignCutMs) {
                if (s_shiftDir == SHIFT_UP) {
                    digitalWrite(PIN_RELAY_UPSHIFT, HIGH);
                } else {
                    digitalWrite(PIN_RELAY_DOWNSHIFT, HIGH);
                }
                s_shiftStateTs = millis();
                s_shiftState   = SHIFT_ACTUATING;
            }
            break;

        case SHIFT_ACTUATING:
            if (elapsed >= s_shiftMs) {
                digitalWrite(PIN_RELAY_UPSHIFT,   LOW);
                digitalWrite(PIN_RELAY_DOWNSHIFT, LOW);
                digitalWrite(PIN_RELAY_IGN_CUT,   LOW);
                s_shiftStateTs = millis();
                s_shiftState   = SHIFT_SETTLING;
            }
            break;

        case SHIFT_SETTLING:
            if (elapsed >= SHIFT_SETTLE_MS) {
                s_shiftResult.ready        = true;
                s_shiftResult.dir          = s_shiftDir;
                s_shiftResult.expectedGear = s_expectedGear;
                s_shiftResult.actualGear   = g_currentGear;
                s_shiftResult.success      = (g_currentGear == s_expectedGear);
                Serial.printf("[SHIFT] Complete — expected=%u  actual=%u  %s\n",
                    s_expectedGear, g_currentGear,
                    s_shiftResult.success ? "OK" : "FAILED");
                s_shiftState = SHIFT_IDLE;
            }
            break;

        default:
            break;
    }
}
