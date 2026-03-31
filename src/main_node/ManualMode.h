#ifndef MANUAL_MODE_H
#define MANUAL_MODE_H

#include <Arduino.h>
#include "SimpleServo.h"
#include "HallSensorControl.h"

// Manual mode relay timing
#define MANUAL_RELAY_DURATION_MS 100

class ManualMode {
private:
    // Hardware references
    HallSensorControl* hallSensor;
    SimpleServo* clutchServo;
    
    // Pin definitions
    int pinNeutralDown;
    int pinNeutralUp;
    int pinShiftDown;
    int pinShiftUp;
    int pinDownshiftRelay;
    int pinUpshiftRelay;
    
    // State variables
    bool manualModeEnabled;
    bool lastNeutralDownState;
    bool lastNeutralUpState;
    bool lastShiftDownState;
    bool lastShiftUpState;
    
    // Relay control
    bool relayActive;
    unsigned long relayStartTime;
    int activeRelayPin;
    int relayDuration;
    
    // Toggle detection
    unsigned long lastToggleCheck;
    static const unsigned long TOGGLE_HOLD_TIME = 1000; // 1 second hold required
    
    // Status display
    unsigned long lastStatusUpdate;
    static const unsigned long STATUS_INTERVAL = 5000; // Update every 5 seconds
    
public:
    ManualMode(int nDownPin, int nUpPin, int sDownPin, int sUpPin, 
               int downRelayPin, int upRelayPin) 
        : hallSensor(nullptr), clutchServo(nullptr),
          pinNeutralDown(nDownPin), pinNeutralUp(nUpPin),
          pinShiftDown(sDownPin), pinShiftUp(sUpPin),
          pinDownshiftRelay(downRelayPin), pinUpshiftRelay(upRelayPin),
          manualModeEnabled(false), relayActive(false), relayStartTime(0),
          activeRelayPin(-1), relayDuration(0), lastToggleCheck(0),
          lastStatusUpdate(0) {
        
        // Initialize button states
        lastNeutralDownState = HIGH;
        lastNeutralUpState = HIGH;
        lastShiftDownState = HIGH;
        lastShiftUpState = HIGH;
    }
    
    void begin(HallSensorControl* hall, SimpleServo* servo) {
        hallSensor = hall;
        clutchServo = servo;
        
        Serial.println("Manual Mode Controller initialized");
        Serial.println("Toggle: Press both neutral buttons simultaneously for 1 second");
        Serial.println("Manual mode: DISABLED (automatic mode active)");
    }
    
    void update() {
        // Check for manual mode toggle
        checkModeToggle();
        
        if (manualModeEnabled) {
            // Manual mode - direct input/output control
            updateManualMode();
        }
        
        // Update relay control (works in both modes)
        updateRelayControl();
        
        // Periodic status update
        updateStatus();
    }
    
    bool isManualModeEnabled() const {
        return manualModeEnabled;
    }
    
    void setManualMode(bool enabled) {
        if (enabled != manualModeEnabled) {
            manualModeEnabled = enabled;
            
            if (manualModeEnabled) {
                Serial.println("=======================================");
                Serial.println("MANUAL MODE ENABLED");
                Serial.println("Direct control active:");
                Serial.println("- Hall sensor → Clutch servo (no limits)");
                Serial.println("- Shift Down button → Downshift relay");
                Serial.println("- Shift Up button → Upshift relay");
                Serial.println("- Neutral buttons → Manual mode toggle");
                Serial.println("=======================================");
            } else {
                Serial.println("=======================================");
                Serial.println("MANUAL MODE DISABLED");
                Serial.println("Automatic state machine control restored");
                Serial.println("=======================================");
            }
        }
    }
    
    void printStatus() {
        Serial.println("=== MANUAL MODE STATUS ===");
        Serial.println("Mode: " + String(manualModeEnabled ? "MANUAL" : "AUTOMATIC"));
        Serial.println("Relay Active: " + String(relayActive ? "YES" : "NO"));
        if (relayActive) {
            Serial.println("Active Relay Pin: " + String(activeRelayPin));
            Serial.println("Time Remaining: " + String(relayDuration - (millis() - relayStartTime)) + "ms");
        }
        Serial.println("===========================");
    }
    
private:
    void checkModeToggle() {
        bool neutralDownPressed = (digitalRead(pinNeutralDown) == LOW);
        bool neutralUpPressed = (digitalRead(pinNeutralUp) == LOW);
        
        // Both neutral buttons pressed simultaneously
        if (neutralDownPressed && neutralUpPressed) {
            if (lastToggleCheck == 0) {
                // Start timing the hold
                lastToggleCheck = millis();
                Serial.println("Manual mode toggle: Hold both neutral buttons...");
            } else if (millis() - lastToggleCheck >= TOGGLE_HOLD_TIME) {
                // Held long enough - toggle mode
                setManualMode(!manualModeEnabled);
                lastToggleCheck = 0; // Reset
                
                // Wait for buttons to be released to prevent immediate re-toggle
                while (digitalRead(pinNeutralDown) == LOW || digitalRead(pinNeutralUp) == LOW) {
                    delay(10);
                    yield();
                }
            }
        } else {
            // Buttons not both pressed - reset timer
            if (lastToggleCheck != 0) {
                Serial.println("Manual mode toggle cancelled");
                lastToggleCheck = 0;
            }
        }
    }
    
    void updateManualMode() {
        // Direct hall sensor to clutch control
        updateDirectClutchControl();
        
        // Direct button to relay control
        updateDirectButtonControl();
    }
    
    void updateDirectClutchControl() {
        if (!hallSensor || !clutchServo) return;
        
        // Get raw hall sensor value
        int hallValue = hallSensor->getRawValue();
        
        // Direct mapping: hall sensor directly controls servo position
        // Map hall range (780-4000) to servo range (0-180)
        int servoPosition = map(hallValue, 780, 4000, 0, 180);
        servoPosition = constrain(servoPosition, 0, 180);
        
        // Set servo position directly - no state machine interference
        clutchServo->write(servoPosition);
    }
    
    void updateDirectButtonControl() {
        // Read current button states
        bool shiftDownState = (digitalRead(pinShiftDown) == LOW);
        bool shiftUpState = (digitalRead(pinShiftUp) == LOW);
        
        // Shift Down button - fire downshift relay
        if (shiftDownState && !lastShiftDownState) {
            if (!relayActive) { // Don't interrupt active relay
                activateRelay(pinDownshiftRelay, MANUAL_RELAY_DURATION_MS);
                Serial.println("MANUAL: Downshift relay fired");
            }
        }
        
        // Shift Up button - fire upshift relay
        if (shiftUpState && !lastShiftUpState) {
            if (!relayActive) { // Don't interrupt active relay
                activateRelay(pinUpshiftRelay, MANUAL_RELAY_DURATION_MS);
                Serial.println("MANUAL: Upshift relay fired");
            }
        }
        
        // Update button states
        lastShiftDownState = shiftDownState;
        lastShiftUpState = shiftUpState;
    }
    
    void activateRelay(int pin, int duration) {
        digitalWrite(pin, HIGH); // Activate relay
        relayActive = true;
        relayStartTime = millis();
        activeRelayPin = pin;
        relayDuration = duration;
        
        Serial.println("Manual relay activated on pin " + String(pin) + " for " + String(duration) + "ms");
    }
    
    void deactivateRelay() {
        if (relayActive) {
            digitalWrite(activeRelayPin, LOW); // Deactivate relay
            Serial.println("Manual relay deactivated on pin " + String(activeRelayPin));
            relayActive = false;
        }
    }
    
    void updateRelayControl() {
        if (relayActive) {
            if (millis() - relayStartTime >= relayDuration) {
                deactivateRelay();
            }
        }
    }
    
    void updateStatus() {
        if (manualModeEnabled && millis() - lastStatusUpdate >= STATUS_INTERVAL) {
            lastStatusUpdate = millis();
            
            // Show manual mode status
            if (hallSensor) {
                int hallValue = hallSensor->getRawValue();
                int servoPos = map(hallValue, 780, 4000, 0, 180);
                servoPos = constrain(servoPos, 0, 180);
                
                Serial.println("MANUAL MODE: Hall=" + String(hallValue) + 
                              ", Servo=" + String(servoPos) + "°, " +
                              "Relay=" + String(relayActive ? "ACTIVE" : "IDLE"));
            }
        }
    }
};

#endif

// end of code