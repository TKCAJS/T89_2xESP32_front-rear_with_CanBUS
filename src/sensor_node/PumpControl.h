/**
 * PumpControl.h
 * Temperature-based cooling pump duty schedule for the sensor node.
 * Plain functions, no classes — matches the rest of the sensor node.
 */
#pragma once
#include <Arduino.h>

// Cooling pump duty as a function of radiator temperature (°C):
//        t < 50  ->  20 %
//   50 <= t < 70 ->  30 %
//        t >= 70 -> 100 %
// A disconnected DS18B20 reads ~ -127 °C — fail to 100 % (max cooling).
uint8_t pumpDutyForTemp(float tempC) {
    if (tempC <= -100.0f) return 100;   // sensor fault → failsafe full
    if (tempC >=   70.0f) return 100;
    if (tempC >=   50.0f) return 30;
    return 20;
}
