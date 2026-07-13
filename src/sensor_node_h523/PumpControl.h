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
 * 4-zone curve (engine water temp from the NTC sensor, see NtcTemp.h).
 * Breakpoints track the adjustable target temperature at fixed offsets; at the
 * default target of 82 C the curve is 65 / 80 / 85 / 101 C. The target is set
 * from the display node over CAN (CAN_SENS_CMD_TARGET_TEMP). Everything stays
 * inside the 15..90% band, so the pump always circulates at least minimum flow:
 *     <= target-17 C            : 15%           minimum flow — warm-up; prevents
 *                                                localized boiling in the head
 *     target-17 .. target-2 C   : 15% -> 40%    transition — ease in cooler radiator
 *                                                fluid, avoid thermal shock
 *     target-2  .. target+3 C   : 40% -> 75%    target zone — active modulation
 *     target+3  .. target+19 C  : 75% -> 90%    high heat loading — max controlled
 *     >= target+19 C            : 90%
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

static inline uint8_t pumpLerp(float t, float tA, float tB, uint8_t dA, uint8_t dB) {
    float frac = (t - tA) / (tB - tA);
    return (uint8_t)(dA + frac * (dB - dA) + 0.5f);
}

// Engine water temperature (°C) + target (°C) -> pump duty command (%).
uint8_t pumpDutyForTemp(float tempC, uint8_t targetC) {
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
