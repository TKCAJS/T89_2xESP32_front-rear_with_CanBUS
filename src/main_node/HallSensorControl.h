#ifndef HALL_SENSOR_CONTROL_H
#define HALL_SENSOR_CONTROL_H

#include <Arduino.h>
#include <Preferences.h>
#include "SimpleServo.h"
#include "HallResponseTypes.h"

class HallSensorControl {
private:
    int hallPin;
    HallResponseCurve curveType;
    float curveStrength;
    SimpleServo* clutchServo;
    int clutchIdlePos;
    int clutchEngagePos;
    int hallMin;
    int hallMax;
    
public:
    HallSensorControl(int pin) : hallPin(pin), curveType(HALL_LOGARITHMIC),
                                curveStrength(2.0), clutchServo(nullptr),
                                clutchIdlePos(0), clutchEngagePos(180),
                                hallMin(780), hallMax(4000) {}
    
    void begin(SimpleServo* servo) {
        clutchServo = servo;
        pinMode(hallPin, INPUT);
        loadConfiguration();
        
        Serial.println("Hall Sensor Control initialized:");
        Serial.println("  Pin: " + String(hallPin));
        Serial.println("  Curve: " + getCurveTypeName());
        Serial.println("  Strength: " + String(curveStrength, 2));
    }
    
    void setConfiguration(int idlePos, int engagePos) {
        clutchIdlePos = idlePos;
        clutchEngagePos = engagePos;
    }
    
    void updateClutchControl(bool isIdle) {
        // Only control clutch manually when in idle state (not shifting)
        if (!isIdle) return;
        
        int hallValue = analogRead(hallPin);
        
        // Use non-linear mapping
        int servoPos = hallToServoNonLinear(hallValue, hallMin, hallMax,
                                           clutchIdlePos, clutchEngagePos,
                                           curveType, curveStrength);
        
        clutchServo->write(servoPos);
    }
    
    int getRawValue() {
        return analogRead(hallPin);
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
            case HALL_CUSTOM: return "Custom Power";
            default: return "Unknown";
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

    void runTest() {
        Serial.println("=== HALL SENSOR TEST MODE ===");
        Serial.println("Move the hall sensor and observe the response");
        Serial.println("Send any character to exit test mode");
        Serial.println("Format: Raw | Linear | Curved | Servo");
        
        while (!Serial.available()) {
            int hallValue = analogRead(hallPin);
            
            // Linear mapping (old way)
            int linearServo = map(hallValue, hallMin, hallMax, clutchIdlePos, clutchEngagePos);

            // Non-linear mapping (new way)
            int curvedServo = hallToServoNonLinear(hallValue, hallMin, hallMax,
                                                  clutchIdlePos, clutchEngagePos,
                                                  curveType, curveStrength);
            
            Serial.printf("%4d | %3d° | %3d° | %3d°\n", 
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
    int hallToServoNonLinear(int hallValue, int hallMin, int hallMax, int servoMin, int servoMax, 
                            HallResponseCurve curveType, float curveStrength) {
        // Constrain hall value to expected range
        hallValue = constrain(hallValue, hallMin, hallMax);
        
        // Normalize hall value to 0.0 - 1.0 range
        float normalized = (float)(hallValue - hallMin) / (float)(hallMax - hallMin);
        
        // Apply non-linear curve
        float curved = applyHallCurve(normalized, curveType, curveStrength);
        
        // Scale to servo range and return as integer
        return (int)(servoMin + curved * (servoMax - servoMin));
    }
    
    void saveConfiguration() {
        Preferences prefs;
        prefs.begin("gearbox", false);
        prefs.putInt("hallCurveType", (int)curveType);
        prefs.putFloat("hallCurveStr", curveStrength);
        prefs.putInt("hallMin", hallMin);
        prefs.putInt("hallMax", hallMax);
        prefs.end();
        Serial.println("Hall sensor curve configuration saved");
    }
    
    void loadConfiguration() {
        Preferences prefs;
        prefs.begin("gearbox", true);
        curveType = (HallResponseCurve)prefs.getInt("hallCurveType", HALL_LOGARITHMIC);
        curveStrength = prefs.getFloat("hallCurveStr", 2.0);
        hallMin = prefs.getInt("hallMin", 780);
        hallMax = prefs.getInt("hallMax", 4000);
        prefs.end();
        
        Serial.println("Hall sensor curve configuration loaded:");
        Serial.println("  Type: " + getCurveTypeName());
        Serial.println("  Strength: " + String(curveStrength, 2));
    }
};

#endif

// end of code