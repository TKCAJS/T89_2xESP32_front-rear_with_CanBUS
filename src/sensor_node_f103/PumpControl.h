/**
 * PumpControl.h
 * Engine-temperature cooling control for the smart (Infineon-controlled) pump.
 * The returned value is a PWM duty *command* to the pump controller, not a
 * motor drive. Plain functions, no classes — matches the rest of the sensor node.
 *
 * Measured pump duty→behaviour (device-specific, bench-verified):
 *     0%        -> controller FAILSAFE, runs 100% (full speed). Never command
 *                  0% unless full cooling is intended; loss-of-signal = full.
 *     ~10%      -> pump effectively OFF.
 *     15..90%   -> usable proportional band (15 = min speed, 90 = max speed).
 *
 * Control = feedforward curve + slow integral trim.
 *
 * Feedforward: 4-zone curve (engine water temp from the NTC sensor, see
 * NtcTemp.h). Breakpoints track the adjustable target temperature at fixed
 * offsets; at the default target of 82 C the curve is 65 / 80 / 85 / 101 C.
 * The target is set from the display node over CAN (CAN_SENS_CMD_TARGET_TEMP).
 * Everything stays inside the 15..90% band, so the pump always circulates at
 * least minimum flow:
 *     <= target-17 C            : 15%           minimum flow — warm-up; prevents
 *                                                localized boiling in the head
 *     target-17 .. target-2 C   : 15% -> 40%    transition — ease in cooler radiator
 *                                                fluid, avoid thermal shock
 *     target-2  .. target+3 C   : 40% -> 75%    target zone — active modulation
 *     target+3  .. target+19 C  : 75% -> 90%    high heat loading — max controlled
 *     >= target+19 C            : 90%
 *
 * Integral trim: the curve alone is proportional-only, so the engine settles
 * wherever heat production balances cooling at the curve's duty — possibly
 * well below target (observed: stable ~70 C at ~22% with an 82 C target).
 * A slow integrator biases the curve's output so the steady state converges
 * on the target: below target it bleeds duty down toward the 15% floor so
 * the engine can finish warming; above target it adds cooling on top of the
 * curve. Anti-windup: the bias only integrates while the output is not
 * already saturated in the error's direction, so it stays untouched through
 * a cold warm-up (output pinned at 15%) and cannot wind up during a heat
 * soak at 90%. The trim acts on a tens-of-seconds timescale and the bias is
 * hard-clamped; a target change over CAN keeps the accumulated bias and
 * simply re-converges.
 *
 * CAUTION: the NTC clamps to its table ends (20..150 C), so an unplugged
 * sensor reads 20 C and commands MINIMUM flow — there is no fault value that
 * could force max cooling here.
 */
#pragma once
#include <Arduino.h>

// ── Tunables (vehicle calibration) ───────────────────────────────────────────
// Adjustable target (curve center) and its accepted range.
static const uint8_t PUMP_TARGET_DEFAULT = 82;    // °C — reproduces original curve
static const uint8_t PUMP_TARGET_MIN     = 65;    // °C — CAN command clamp
static const uint8_t PUMP_TARGET_MAX     = 105;   // °C — CAN command clamp

// Breakpoint offsets from the target (°C) and their duty commands (%).
static const float   PUMP_T0_OFS = -17.0f;  static const uint8_t PUMP_D0 = 15;   // min flow
static const float   PUMP_T1_OFS = -2.0f;   static const uint8_t PUMP_D1 = 40;   // enter target zone
static const float   PUMP_T2_OFS = 3.0f;    static const uint8_t PUMP_D2 = 75;   // leave target zone
static const float   PUMP_T3_OFS = 19.0f;   static const uint8_t PUMP_D3 = 90;   // max controlled

static const uint8_t PUMP_DUTY_MIN = PUMP_D0;   // pump never commanded below this
static const uint8_t PUMP_DUTY_MAX = PUMP_D3;   // max cooling failsafe

// Integral trim: duty % per °C of error per second. At 0.2 a steady 3 C error
// moves the command 0.6%/s, so the trim acts on a tens-of-seconds timescale
// (full 25% swing in ~40 s at that error). The clamp bounds both the
// worst-case unwind time and how far a stuck error can push the command.
static const float PUMP_KI_PCT_PER_C_SEC = 0.2f;
static const float PUMP_BIAS_LIMIT_PCT   = 25.0f;

static inline float pumpLerp(float t, float tA, float tB, uint8_t dA, uint8_t dB) {
    return dA + (t - tA) / (tB - tA) * (float)(dB - dA);
}

// Feedforward: engine water temperature (°C) + target (°C) -> curve duty (%).
static float pumpCurve(float tempC, uint8_t targetC) {
    float t0 = targetC + PUMP_T0_OFS;
    float t1 = targetC + PUMP_T1_OFS;
    float t2 = targetC + PUMP_T2_OFS;
    float t3 = targetC + PUMP_T3_OFS;
    if (tempC <= t0)  return PUMP_D0;   // minimum flow
    if (tempC >= t3)  return PUMP_D3;   // max cooling
    if (tempC <  t1)  return pumpLerp(tempC, t0, t1, PUMP_D0, PUMP_D1);
    if (tempC <  t2)  return pumpLerp(tempC, t1, t2, PUMP_D1, PUMP_D2);
    return pumpLerp(tempC, t2, t3, PUMP_D2, PUMP_D3);
}

// Engine water temperature (°C) + target (°C) -> pump duty command (%).
// Call periodically from the sensor-read tick; integral state and dt
// (from millis()) live inside, so the call site stays a pure function call.
uint8_t pumpDutyForTemp(float tempC, uint8_t targetC) {
    static float    biasPct  = 0.0f;
    static uint32_t lastMs   = 0;
    static bool     haveLast = false;

    uint32_t nowMs = millis();
    float dtSec = haveLast ? (float)(nowMs - lastMs) / 1000.0f : 0.0f;
    lastMs   = nowMs;
    haveLast = true;

    float curve = pumpCurve(tempC, targetC);
    float error = tempC - targetC;          // positive = too hot
    float out   = curve + biasPct;

    // Anti-windup: freeze the integrator while the output is saturated in the
    // direction the error is pushing.
    bool satLo = out <= PUMP_DUTY_MIN;
    bool satHi = out >= PUMP_DUTY_MAX;
    if (!((satLo && error < 0.0f) || (satHi && error > 0.0f))) {
        biasPct += PUMP_KI_PCT_PER_C_SEC * error * dtSec;
        biasPct  = constrain(biasPct, -PUMP_BIAS_LIMIT_PCT, PUMP_BIAS_LIMIT_PCT);
        out = curve + biasPct;
    }

    out = constrain(out, (float)PUMP_DUTY_MIN, (float)PUMP_DUTY_MAX);
    return (uint8_t)(out + 0.5f);
}
