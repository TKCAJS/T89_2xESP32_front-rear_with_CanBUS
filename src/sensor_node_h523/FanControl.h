/**
 * FanControl.h
 * Two staged aux-fan relay outputs driving a mini relay board.
 * ACTIVE HIGH (bench-verified 2026-07-19: this board energizes on a high
 * input): pin LOW = relay off; floating (boot/reset) = off. Wiring:
 * IN1=PB4 (FAN1), IN2=PB5 (FAN2), board VCC=3.3V, no pull resistors.
 *
 * Staging (offsets from the CAN-adjustable pump target temp):
 *   FAN1 on : temp >= target+3 C while the pump sits at its 90% cap,
 *             sustained FAN_DWELL_MS — i.e. the pump alone has lost.
 *   FAN1 off: temp <= target          (hysteresis)
 *   FAN2 on : temp >= target+8 C sustained FAN_DWELL_MS (pump is at cap
 *             well before this — no duty gate needed)
 *   FAN2 off: temp <= target+4
 *
 * CAUTION: same NTC blind spot as PumpControl.h — an unplugged sensor
 * reads 20 C and keeps the fans OFF.
 * PB4 is JTAG-NJTRST at reset; pinMode() releases it (SWD unaffected).
 * PB5 is not 5V-tolerant on the F103 — fine as a 3.3V output; only
 * relevant if the relay board were ever rewired open-drain at 5V.
 */
#pragma once
#include <Arduino.h>
#include "PumpControl.h"

#define PIN_FAN1  PB4
#define PIN_FAN2  PB5

static const float    FAN1_ON_OFS  = 3.0f;    // °C above target
static const float    FAN1_OFF_OFS = 0.0f;
static const float    FAN2_ON_OFS  = 8.0f;
static const float    FAN2_OFF_OFS = 4.0f;
static const uint32_t FAN_DWELL_MS = 5000;    // on-condition must hold this long

static bool s_fan1On = false;
static bool s_fan2On = false;

static inline void fanWrite(uint32_t pin, bool on) {
    digitalWrite(pin, on ? HIGH : LOW);   // active high
}

void fanInit() {
    // ODR first so the inputs never see an ON blip when the driver
    // switches on, then one click per relay as a power-on wiring
    // check — audible on the bench, a 250 ms fan twitch in the vehicle.
    digitalWrite(PIN_FAN1, LOW);
    digitalWrite(PIN_FAN2, LOW);
    pinMode(PIN_FAN1, OUTPUT);
    pinMode(PIN_FAN2, OUTPUT);
    for (uint32_t pin : { (uint32_t)PIN_FAN1, (uint32_t)PIN_FAN2 }) {
        fanWrite(pin, true);
        delay(250);
        fanWrite(pin, false);
        delay(250);
    }
}

// Engine temp + target + pump duty -> relay states. Call every sensor tick.
void fanUpdate(float tempC, uint8_t targetC, uint8_t pumpDuty, uint32_t nowMs) {
    static uint32_t fan1Since = 0, fan2Since = 0;   // 0 = on-condition not held

    bool want1 = (tempC >= targetC + FAN1_ON_OFS) && (pumpDuty >= PUMP_DUTY_MAX);
    if (!s_fan1On) {
        if (!want1)              fan1Since = 0;
        else if (fan1Since == 0) fan1Since = nowMs ? nowMs : 1;
        else if (nowMs - fan1Since >= FAN_DWELL_MS) s_fan1On = true;
    } else if (tempC <= targetC + FAN1_OFF_OFS) {
        s_fan1On  = false;
        fan1Since = 0;
    }

    bool want2 = tempC >= targetC + FAN2_ON_OFS;
    if (!s_fan2On) {
        if (!want2)              fan2Since = 0;
        else if (fan2Since == 0) fan2Since = nowMs ? nowMs : 1;
        else if (nowMs - fan2Since >= FAN_DWELL_MS) s_fan2On = true;
    } else if (tempC <= targetC + FAN2_OFF_OFS) {
        s_fan2On  = false;
        fan2Since = 0;
    }

    fanWrite(PIN_FAN1, s_fan1On);
    fanWrite(PIN_FAN2, s_fan2On);
}
