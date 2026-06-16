#ifndef HALL_SENSOR_CONTROL_H
#define HALL_SENSOR_CONTROL_H

#include <Arduino.h>
#include <Preferences.h>
#include "SimpleServo.h"
#include "HallResponseTypes.h"

class HallSensorControl {
private:
    int hallPin;
    int hallPin2;   // second clutch paddle (right)
    HallResponseCurve curveType;
    float curveStrength;
    SimpleServo* clutchServo;
    int clutchIdlePos;
    int clutchFullyPull;
    int hallMin;
    int hallMax;
    int pin2RawMin;   // pin4 raw ADC at idle (lever released) — calibrated
    int pin2RawMax;   // pin4 raw ADC at pulled (lever fully in) — calibrated
    bool servoOverride;  // true = slider/web controls servo; hall sensor backs off
    // Piecewise breakpoints
    int hallBiteStart;   // ADC value where biting zone input begins
    int hallBiteEnd;     // ADC value where biting zone input ends
    int servoBiteStart;  // servo angle at start of biting zone (just-engaged)
    int servoBiteEnd;    // servo angle at end of biting zone (just-disengaged)

    // Added to the captured idle reading so sensor noise at rest can't push
    // hallValue above hallMin/pin2RawMin and twitch the servo off idle.
    static const int HALL_IDLE_DEADBAND = 30;

public:
    HallSensorControl(int pin, int pin2) : hallPin(pin), hallPin2(pin2), curveType(HALL_LOGARITHMIC),
                                curveStrength(2.0), clutchServo(nullptr),
                                clutchIdlePos(0), clutchFullyPull(180),
                                hallMin(780), hallMax(3330),
                                pin2RawMin(1000), pin2RawMax(3600), servoOverride(false),
                                hallBiteStart(1000), hallBiteEnd(3000),
                                servoBiteStart(98), servoBiteEnd(115) {}
    
    void begin(SimpleServo* servo) {
        clutchServo = servo;
        pinMode(hallPin, INPUT);
        pinMode(hallPin2, INPUT);
        loadConfiguration();
        
        Serial.println("Hall Sensor Control initialized:");
        Serial.println("  Pin: " + String(hallPin));
        Serial.println("  Curve: " + getCurveTypeName());
        Serial.println("  Strength: " + String(curveStrength, 2));
    }
    
    void setConfiguration(int idlePos, int engagePos) {
        clutchIdlePos = idlePos;
        clutchFullyPull = engagePos;
    }
    
    void setServoOverride(bool active) { servoOverride = active; }
    bool isServoOverride() const { return servoOverride; }

    void updateClutchControl(bool isIdle) {
        if (servoOverride) return;  // slider/web has control
        if (!isIdle) return;
        
        int hallValue = max((int)analogRead(hallPin), pin2Scaled());
        float servoPos;

        if (curveType == HALL_PIECEWISE) {
            servoPos = hallToPiecewise(hallValue);
        } else {
            servoPos = hallToServoNonLinear(hallValue, hallMin, hallMax,
                                           clutchIdlePos, clutchFullyPull,
                                           curveType, curveStrength);
        }

        clutchServo->writeFloat(servoPos);
    }
    
    int getRawValue() {
        return max((int)analogRead(hallPin), pin2Scaled());
    }
    
    // Configuration functions
    void setCurveType(const String& curveTypeStr) {
        String type = curveTypeStr;
        type.toLowerCase();
        
        if (type == "linear") {
            curveType = HALL_LINEAR;
            Serial.println("Hall curve set to LINEAR");
        } else if (type == "log" || type == "logarithmic") {
            curveType = HALL_LOGARITHMIC;
            Serial.println("Hall curve set to LOGARITHMIC (more sensitive initially)");
        } else if (type == "exp" || type == "exponential") {
            curveType = HALL_EXPONENTIAL;
            Serial.println("Hall curve set to EXPONENTIAL (more sensitive at end)");
        } else if (type == "smooth") {
            curveType = HALL_SMOOTH_STEP;
            Serial.println("Hall curve set to SMOOTH STEP (S-curve)");
        } else if (type == "custom") {
            curveType = HALL_CUSTOM;
            Serial.println("Hall curve set to CUSTOM POWER");
        } else if (type == "piecewise") {
            curveType = HALL_PIECEWISE;
            Serial.println("Hall curve set to PIECEWISE");
        } else {
            Serial.println("Invalid curve type. Use: linear, log, exp, smooth, custom");
            return;
        }
        
        saveConfiguration();
    }
    
    void setCurveStrength(float strength) {
        strength = constrain(strength, 0.1, 5.0);
        curveStrength = strength;
        Serial.println("Hall curve strength set to: " + String(curveStrength, 2));
        saveConfiguration();
    }
    
    String getCurveTypeName() const {
        switch (curveType) {
            case HALL_LINEAR: return "Linear";
            case HALL_LOGARITHMIC: return "Logarithmic";
            case HALL_EXPONENTIAL: return "Exponential";
            case HALL_SMOOTH_STEP: return "Smooth Step";
            case HALL_CUSTOM:     return "Custom Power";
            case HALL_PIECEWISE:  return "Piecewise";
            default:              return "Unknown";
        }
    }
    
    HallResponseCurve getCurveType() const {
        return curveType;
    }
    
    float getCurveStrength() const {
        return curveStrength;
    }

    void setHallRange(int min, int max) {
        hallMin = constrain(min, 0, 4095);
        hallMax = constrain(max, 0, 4095);
        saveConfiguration();
        Serial.println("Hall range set: " + String(hallMin) + "-" + String(hallMax));
    }

    int getHallMin() const { return hallMin; }
    int getHallMax() const { return hallMax; }

    void setPiecewiseZone(int hBiteStart, int hBiteEnd, int sBiteStart, int sBiteEnd) {
        hallBiteStart  = constrain(hBiteStart, 0, 4095);
        hallBiteEnd    = constrain(hBiteEnd,   0, 4095);
        servoBiteStart = constrain(sBiteStart, 0, 180);
        servoBiteEnd   = constrain(sBiteEnd,   0, 180);
        saveConfiguration();
        Serial.println("Piecewise zone: hall " + String(hallBiteStart) + "-" + String(hallBiteEnd) +
                       " -> servo " + String(servoBiteStart) + "-" + String(servoBiteEnd) + "deg");
    }

    int getHallBiteStart()  const { return hallBiteStart; }
    int getHallBiteEnd()    const { return hallBiteEnd; }
    int getServoBiteStart() const { return servoBiteStart; }
    int getServoBiteEnd()   const { return servoBiteEnd; }

    // Pin4 (right paddle) getters
    int getPin2Raw()     const { return analogRead(hallPin2); }
    int getPin2Scaled()  const { return pin2Scaled(); }
    int getPin2RawMin()  const { return pin2RawMin; }
    int getPin2RawMax()  const { return pin2RawMax; }

    // One-shot capture: hold paddle in position, click. Saves immediately.
    String capturePin1Idle()   { hallMin    = constrain(analogRead(hallPin)  + HALL_IDLE_DEADBAND, 0, 4095); saveConfiguration(); return String(hallMin); }
    String capturePin1Pulled() { hallMax    = analogRead(hallPin);    saveConfiguration(); return String(hallMax); }
    String capturePin2Idle()   { pin2RawMin = constrain(analogRead(hallPin2) + HALL_IDLE_DEADBAND, 0, 4095); saveConfiguration(); return String(pin2RawMin); }
    String capturePin2Pulled() { pin2RawMax = analogRead(hallPin2); saveConfiguration(); return String(pin2RawMax); }

    void runTest() {
        Serial.println("=== HALL SENSOR TEST MODE ===");
        Serial.println("Move the hall sensor and observe the response");
        Serial.println("Send any character to exit test mode");
        Serial.println("Format: Raw | Linear | Curved | Servo");
        
        while (!Serial.available()) {
            int hallValue = max((int)analogRead(hallPin), pin2Scaled());
            
            float linearServo = mapf(hallValue, hallMin, hallMax, clutchIdlePos, clutchFullyPull);
            float curvedServo = (curveType == HALL_PIECEWISE)
                ? hallToPiecewise(hallValue)
                : hallToServoNonLinear(hallValue, hallMin, hallMax,
                                      clutchIdlePos, clutchFullyPull,
                                      curveType, curveStrength);

            Serial.printf("%4d | %5.1f° | %5.1f° | %5.1f°\n",
                          hallValue, linearServo, curvedServo, curvedServo);
            
            delay(100);
            yield();
        }
        
        // Clear the serial buffer
        while (Serial.available()) {
            Serial.read();
        }
        
        Serial.println("=== HALL TEST MODE EXITED ===");
    }
    
    void printInfo() {
        Serial.println("=== HALL SENSOR CURVE CONFIGURATION ===");
        Serial.print("Current curve type: ");
        Serial.println(getCurveTypeName());
        Serial.println("Curve strength: " + String(curveStrength, 2));
        Serial.println("Available commands:");
        Serial.println("  curve linear       - Set linear response");
        Serial.println("  curve log          - Set logarithmic response");  
        Serial.println("  curve exp          - Set exponential response");
        Serial.println("  curve smooth       - Set smooth step response");
        Serial.println("  curve custom       - Set custom power curve");
        Serial.println("  strength <value>   - Set curve strength (0.1-5.0)");
        Serial.println("  halltest           - Show live hall sensor values");
        Serial.println("=========================================");
    }
    
private:
    // Map pin4 raw reading onto pin5's calibrated scale so max() is meaningful.
    int pin2Scaled() const {
        int raw = analogRead(hallPin2);
        return constrain(map(raw, pin2RawMin, pin2RawMax, hallMin, hallMax), hallMin, hallMax);
    }

    // Floating-point map — Arduino's map() is integer-only, which quantised the
    // servo to whole degrees. This preserves sub-degree resolution.
    static float mapf(float x, float inMin, float inMax, float outMin, float outMax) {
        if (inMax == inMin) return outMin;
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    float hallToPiecewise(int hallValue) {
        hallValue = constrain(hallValue, hallMin, hallMax);
        if (hallValue <= hallBiteStart) {
            return mapf(hallValue, hallMin, hallBiteStart, clutchIdlePos, servoBiteStart);
        } else if (hallValue <= hallBiteEnd) {
            return mapf(hallValue, hallBiteStart, hallBiteEnd, servoBiteStart, servoBiteEnd);
        } else {
            return mapf(hallValue, hallBiteEnd, hallMax, servoBiteEnd, clutchFullyPull);
        }
    }

    /**
     * Apply non-linear curve to normalized input (0.0 to 1.0)
     * Returns value from 0.0 to 1.0
     */
    float applyHallCurve(float normalizedInput, HallResponseCurve curveType, float strength) {
        // Constrain input to valid range
        normalizedInput = constrain(normalizedInput, 0.0, 1.0);
        
        switch (curveType) {
            case HALL_LINEAR:
                return normalizedInput;
            
            case HALL_LOGARITHMIC: {
                // Logarithmic curve: more sensitive at the beginning, less at the end
                if (strength <= 0) strength = 0.1; // Prevent division by zero
                return log(1.0 + normalizedInput * (exp(strength) - 1.0)) / log(exp(strength));
            }
            
            case HALL_EXPONENTIAL: {
                // Exponential curve: less sensitive at beginning, more at the end  
                if (strength <= 0) strength = 0.1;
                return (exp(normalizedInput * strength) - 1.0) / (exp(strength) - 1.0);
            }
            
            case HALL_SMOOTH_STEP: {
                // Smooth S-curve (smoothstep function)
                float smoothed = normalizedInput * normalizedInput * (3.0 - 2.0 * normalizedInput);
                return normalizedInput + (smoothed - normalizedInput) * (strength / 3.0);
            }
            
            case HALL_CUSTOM: {
                // Custom power curve
                if (strength <= 0) strength = 1.0;
                return pow(normalizedInput, strength);
            }
            
            default:
                return normalizedInput;
        }
    }
    
    /**
     * Convert hall sensor reading to servo position with non-linear response
     */
    float hallToServoNonLinear(int hallValue, int hallMin, int hallMax, int servoMin, int servoMax,
                            HallResponseCurve curveType, float curveStrength) {
        // Constrain hall value to expected range
        hallValue = constrain(hallValue, hallMin, hallMax);

        // Normalize hall value to 0.0 - 1.0 range
        float normalized = (float)(hallValue - hallMin) / (float)(hallMax - hallMin);

        // Apply non-linear curve
        float curved = applyHallCurve(normalized, curveType, curveStrength);

        // Scale to servo range (sub-degree resolution preserved)
        return servoMin + curved * (servoMax - servoMin);
    }
    
    void saveConfiguration() {
        Preferences prefs;
        prefs.begin("gearbox", false);
        prefs.putInt("hallCurveType", (int)curveType);
        prefs.putFloat("hallCurveStr", curveStrength);
        prefs.putInt("hallMin", hallMin);
        prefs.putInt("hallMax", hallMax);
        prefs.putInt("hallBiteStart",  hallBiteStart);
        prefs.putInt("hallBiteEnd",    hallBiteEnd);
        prefs.putInt("servoBiteStart", servoBiteStart);
        prefs.putInt("servoBiteEnd",   servoBiteEnd);
        prefs.putInt("pin2RawMin",     pin2RawMin);
        prefs.putInt("pin2RawMax",     pin2RawMax);
        prefs.end();
        Serial.println("Hall sensor curve configuration saved");
    }
    
    void loadConfiguration() {
        Preferences prefs;
        prefs.begin("gearbox", true);
        curveType = (HallResponseCurve)prefs.getInt("hallCurveType", HALL_LOGARITHMIC);
        curveStrength = prefs.getFloat("hallCurveStr", 2.0);
        hallMin       = prefs.getInt("hallMin",       780);
        hallMax       = prefs.getInt("hallMax",       3330);
        hallBiteStart = prefs.getInt("hallBiteStart", 1000);
        hallBiteEnd   = prefs.getInt("hallBiteEnd",   3000);
        servoBiteStart= prefs.getInt("servoBiteStart",98);
        servoBiteEnd  = prefs.getInt("servoBiteEnd",  115);
        pin2RawMin    = prefs.getInt("pin2RawMin",    1000);
        pin2RawMax    = prefs.getInt("pin2RawMax",    3600);
        prefs.end();
        
        Serial.println("Hall sensor curve configuration loaded:");
        Serial.println("  Type: " + getCurveTypeName());
        Serial.println("  Strength: " + String(curveStrength, 2));
    }
};

#endif

// end of code