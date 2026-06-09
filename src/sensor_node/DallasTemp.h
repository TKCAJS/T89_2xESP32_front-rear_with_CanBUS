/**
 * DallasTemp.h
 * DS18B20 (OneWire) radiator temperature for the sensor node.
 * Non-blocking 1 Hz conversion. Plain functions, no classes.
 */
#pragma once
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// TODO: assign final pin
#define ONEWIRE_PIN       PC0
#define DALLAS_PERIOD_MS  1000   // request once per second

static OneWire           s_oneWire(ONEWIRE_PIN);
static DallasTemperature s_dallas(&s_oneWire);
static uint32_t          s_lastDallasReq = 0;
static float             s_dallasTemp    = 0.0f;

void dallasBegin() {
    s_dallas.setWaitForConversion(false);   // non-blocking conversions
    s_dallas.begin();
    s_dallas.requestTemperatures();
    s_lastDallasReq = millis();
}

// Reads the previous conversion and kicks off the next, at 1 Hz.
// Returns true when a fresh reading was taken this call.
bool dallasUpdate(uint32_t now) {
    if (now - s_lastDallasReq < DALLAS_PERIOD_MS) return false;
    s_dallasTemp    = s_dallas.getTempCByIndex(0);   // result of last request
    s_dallas.requestTemperatures();                  // start next conversion
    s_lastDallasReq = now;
    return true;
}

float dallasTempC() { return s_dallasTemp; }
