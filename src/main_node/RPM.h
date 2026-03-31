#ifndef RPM_H
#define RPM_H

#include <Arduino.h>
#include "driver/pcnt.h"

class RPM {
private:
    int rpmPin;
    float pulsesPerRevolution;
    float smoothingAlpha;  // 0.0 = no smoothing, 1.0 = max smoothing
    float currentRpm;

    unsigned long lastCalcTime;
    unsigned long lastPulseTime;
    const unsigned long calcInterval = 100;     // ms
    const unsigned long pulseTimeout = 2000;    // ms

    pcnt_unit_t pcntUnit = PCNT_UNIT_0;

public:
    RPM(int pin, float pulsesPerRev = 12.0, float smoothing = 0.2)
        : rpmPin(pin),
          pulsesPerRevolution(pulsesPerRev),
          smoothingAlpha(smoothing),
          currentRpm(0),
          lastCalcTime(0),
          lastPulseTime(0)
    {}

    void begin() {
        pinMode(rpmPin, INPUT);

        pcnt_config_t pcntConfig;
        pcntConfig.pulse_gpio_num = (gpio_num_t)rpmPin;
        pcntConfig.ctrl_gpio_num = PCNT_PIN_NOT_USED;
        pcntConfig.unit = pcntUnit;
        pcntConfig.channel = PCNT_CHANNEL_0;
        pcntConfig.pos_mode = PCNT_COUNT_INC;
        pcntConfig.neg_mode = PCNT_COUNT_DIS;
        pcntConfig.lctrl_mode = PCNT_MODE_KEEP;
        pcntConfig.hctrl_mode = PCNT_MODE_KEEP;
        pcntConfig.counter_h_lim = 10000;
        pcntConfig.counter_l_lim = 0;

        pcnt_unit_config(&pcntConfig);
        pcnt_set_filter_value(pcntUnit, 100);  // Debounce pulses ~1us
        pcnt_filter_enable(pcntUnit);

        pcnt_counter_pause(pcntUnit);
        pcnt_counter_clear(pcntUnit);
        pcnt_counter_resume(pcntUnit);

        lastCalcTime = millis();
        lastPulseTime = millis();

        Serial.println("RPM sensor (PCNT) initialized");
        Serial.println("RPM Pin: " + String(rpmPin));
        Serial.println("RPM Pulses/Rev: " + String(pulsesPerRevolution, 1));
    }

    void update() {
        unsigned long now = millis();
        if (now - lastCalcTime >= calcInterval) {
            calculateRpm();
            lastCalcTime = now;
        }

        // Timeout: no pulses received recently
        if (now - lastPulseTime > pulseTimeout) {
            currentRpm = 0;
        }
    }

    float getRpm() const {
        return currentRpm;
    }

    void setPulsesPerRevolution(float pulses) {
        pulsesPerRevolution = pulses;
    }

    float getPulsesPerRevolution() const {
        return pulsesPerRevolution;
    }

    void setSmoothing(float alpha) {
        smoothingAlpha = constrain(alpha, 0.0, 1.0);
    }

    float getSmoothing() const {
        return smoothingAlpha;
    }

private:
    void calculateRpm() {
        int16_t pulseCount = 0;
        pcnt_get_counter_value(pcntUnit, &pulseCount);
        pcnt_counter_clear(pcntUnit);

        if (pulseCount > 0) {
            lastPulseTime = millis();
        }

        float timeMinutes = calcInterval / 60000.0;  // ms to minutes
        float instantRpm = (pulseCount / timeMinutes) / pulsesPerRevolution;

        // Exponential moving average (smoothing)
        currentRpm = (smoothingAlpha * instantRpm) + ((1.0 - smoothingAlpha) * currentRpm);

        // Clamp tiny noise values
        if (currentRpm < 10) currentRpm = 0;
    }
};

#endif // RPM_H
