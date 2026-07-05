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
 * 4-zone curve (engine water temp from DS18B20). Everything stays inside the
 * 15..90% band, so the pump always circulates at least minimum flow:
 *     <= 65 C          : 15%           minimum flow — warm-up; prevents localized
 *                                       boiling in the cylinder head
 *     65 .. 80 C       : 15% -> 40%    transition — ease in cooler radiator fluid,
 *                                       avoid thermal shock to the block
 *     80 .. 85 C       : 40% -> 75%    target zone (~82 C) — active modulation
 *     85 .. 101 C      : 75% -> 90%    high heat loading — max controlled cooling
 *     >= 101 C         : 90%
 *     sensor fault     : 90% (forced)  DS18B20 disconnect reads ~ -127 C
 */
#pragma once
#include <Arduino.h>

// ── Tunables (vehicle calibration) ───────────────────────────────────────────
// Breakpoint temperatures (°C) and their duty commands (%).
static const float   PUMP_T0 = 65.0f;   static const uint8_t PUMP_D0 = 15;   // min flow
static const float   PUMP_T1 = 80.0f;   static const uint8_t PUMP_D1 = 40;   // enter target zone
static const float   PUMP_T2 = 85.0f;   static const uint8_t PUMP_D2 = 75;   // leave target zone
static const float   PUMP_T3 = 101.0f;  static const uint8_t PUMP_D3 = 90;   // max controlled

static const uint8_t PUMP_DUTY_MIN = PUMP_D0;   // pump never commanded below this
static const uint8_t PUMP_DUTY_MAX = PUMP_D3;   // max cooling / sensor-fault failsafe

static inline uint8_t pumpLerp(float t, float tA, float tB, uint8_t dA, uint8_t dB) {
    float frac = (t - tA) / (tB - tA);
    return (uint8_t)(dA + frac * (dB - dA) + 0.5f);
}

// Engine water temperature (°C) -> pump duty command (%).
uint8_t pumpDutyForTemp(float tempC) {
    if (tempC <= -100.0f)  return PUMP_DUTY_MAX;   // DS18B20 fault -> forced max cooling
    if (tempC <= PUMP_T0)  return PUMP_D0;         // <= 65 C: minimum flow
    if (tempC >= PUMP_T3)  return PUMP_D3;         // >= 101 C: max cooling
    if (tempC <  PUMP_T1)  return pumpLerp(tempC, PUMP_T0, PUMP_T1, PUMP_D0, PUMP_D1);
    if (tempC <  PUMP_T2)  return pumpLerp(tempC, PUMP_T1, PUMP_T2, PUMP_D1, PUMP_D2);
    return pumpLerp(tempC, PUMP_T2, PUMP_T3, PUMP_D2, PUMP_D3);
}
