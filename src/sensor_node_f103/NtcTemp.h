#pragma once
#include <Arduino.h>

// Water-temp NTC → °C. Divider: 4.7 kΩ 1% pull-up from the 3.3 V ADC rail to
// the pin, NTC from the pin to GND. The sensor is 2-wire: one wire to board
// GND, the other to the pin — no block-ground return, so no ground-offset
// error. Counts fall as temperature rises.
//
//   R_ntc = NTC_R_SERIES · counts / (NTC_ADC_MAX − counts)
//
// R/T table measured on the actual sensors, 20–150 °C. Linear interpolation
// in R between rows stays within ~0.3 °C of the true curve — no log() needed,
// so libm/float-printf stay unlinked on this 64 KB part.

#define NTC_R_SERIES 4700   // Ω, 1%
#define NTC_ADC_MAX  4095   // analogReadResolution(12) — set in setup()

static const struct { uint32_t ohms; int16_t tempC; } NTC_TABLE[] = {
    { 62320,  20 }, { 40340,  30 }, { 26650,  40 }, { 17950,  50 },
    { 12320,  60 }, {  8610,  70 }, {  6140,  80 }, {  4430,  90 },
    {  3320, 100 }, {  2550, 110 }, {  1970, 120 }, {  1510, 130 },
    {  1170, 140 }, {   900, 150 },
};
#define NTC_TABLE_N (sizeof(NTC_TABLE) / sizeof(NTC_TABLE[0]))

// Clamps to the table ends: an open (unplugged) sensor pins the input high and
// reads 20 °C, a shorted one reads 150 °C.
static float ntcTempC(uint16_t counts) {
    if (counts >= NTC_ADC_MAX) counts = NTC_ADC_MAX - 1;
    uint32_t r = (uint32_t)NTC_R_SERIES * counts / (NTC_ADC_MAX - counts);

    if (r >= NTC_TABLE[0].ohms) return NTC_TABLE[0].tempC;
    for (unsigned i = 1; i < NTC_TABLE_N; i++) {
        if (r >= NTC_TABLE[i].ohms) {
            float span = NTC_TABLE[i - 1].ohms - NTC_TABLE[i].ohms;
            float frac = (NTC_TABLE[i - 1].ohms - r) / span;
            return NTC_TABLE[i - 1].tempC
                 + frac * (NTC_TABLE[i].tempC - NTC_TABLE[i - 1].tempC);
        }
    }
    return NTC_TABLE[NTC_TABLE_N - 1].tempC;
}
