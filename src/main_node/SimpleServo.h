#pragma once
#include <Arduino.h>

// Minimal servo driver using ledcAttach/ledcWrite (arduino-esp32 3.x)
// Replaces ESP32Servo to avoid MCPWM crash on pioarduino/ESP-IDF 5.x
//
// 50Hz PWM, 14-bit resolution (ESP32-S3 max at 50Hz):
//   Period = 20ms, 1 tick = 20000us / 16384 = 1.2207us
//   500us  (0°)   = 410 ticks
//   2500us (180°) = 2048 ticks

#define SIMPLE_SERVO_FREQ       50
#define SIMPLE_SERVO_BITS       14
#define SIMPLE_SERVO_MIN_TICKS  410
#define SIMPLE_SERVO_MAX_TICKS  2048

class SimpleServo {
    int _pin = -1;
    int _minAngle = 0;
    int _maxAngle = 180;

public:
    void attach(int pin) {
        _pin = pin;
        ledcAttach(pin, SIMPLE_SERVO_FREQ, SIMPLE_SERVO_BITS);
    }

    void setLimits(int minAngle, int maxAngle) {
        _minAngle = minAngle;
        _maxAngle = maxAngle;
    }

    void write(int angle) {
        if (_pin < 0) return;
        angle = constrain(angle, _minAngle, _maxAngle);
        uint32_t ticks = map(angle, 0, 180, SIMPLE_SERVO_MIN_TICKS, SIMPLE_SERVO_MAX_TICKS);
        ledcWrite(_pin, ticks);
    }

    void detach() {
        if (_pin < 0) return;
        ledcDetach(_pin);
        _pin = -1;
    }

    bool attached() { return _pin >= 0; }
};
