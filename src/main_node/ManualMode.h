#ifndef MANUAL_MODE_H
#define MANUAL_MODE_H

#include <Arduino.h>
#include "SimpleServo.h"
#include "HallSensorControl.h"
#include "MainCan.h"

// Manual mode shift pulse duration sent to rear node
#define MANUAL_SHIFT_DURATION_MS 100

extern void canSendShiftUp(uint16_t shiftMs, uint16_t ignCutMs);
extern void canSendShiftDown(uint16_t shiftMs);

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

    // State variables
    bool manualModeEnabled;
    bool lastNeutralDownState;
    bool lastNeutralUpState;
    bool lastShiftDownState;
    bool lastShiftUpState;
    
    // Shift timing (for status display only)
    bool relayActive;
    unsigned long relayStartTime;
    bool activeShiftIsUp;
    int relayDuration;
    
    // Toggle detection
    unsigned long lastToggleCheck;
    static const unsigned long TOGGLE_HOLD_TIME = 1000; // 1 second hold required
    
    // Status display
    unsigned long lastStatusUpdate;
    static const unsigned long STATUS_INTERVAL = 5000; // Update every 5 seconds
    
public:
    ManualMode(int nDownPin, int nUpPin, int sDownPin, int sUpPin)
        : hallSensor(nullptr), clutchServo(nullptr),
          pinNeutralDown(nDownPin), pinNeutralUp(nUpPin),
          pinShiftDown(sDownPin), pinShiftUp(sUpPin),
          manualModeEnabled(false), relayActive(false), relayStartTime(0),
          activeShiftIsUp(false), relayDuration(0), lastToggleCheck(0),
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
        Serial.println("Shift Active: " + String(relayActive ? "YES" : "NO"));
        if (relayActive) {
            Serial.println("Direction: " + String(activeShiftIsUp ? "UP" : "DOWN"));
            Serial.println("Time Remaining: " + String(relayDuration - (millis() - relayStartTime)) + "ms");
        }
        Serial.println("===========================");
    }
    
private:
    void checkModeToggle() {
        bool togglePressed = (digitalRead(pinNeutralDown) == LOW);

        if (togglePressed) {
            if (lastToggleCheck == 0) {
                lastToggleCheck = millis();
                Serial.println("Manual mode toggle: hold for 1 second...");
            } else if (millis() - lastToggleCheck >= TOGGLE_HOLD_TIME) {
                setManualMode(!manualModeEnabled);
                lastToggleCheck = 0;
                while (digitalRead(pinNeutralDown) == LOW) {
                    delay(10);
                    yield();
                }
            }
        } else {
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
        
        // Shift Down button
        if (shiftDownState && !lastShiftDownState) {
            if (!relayActive) {
                canSendShiftDown(MANUAL_SHIFT_DURATION_MS);
                relayActive     = true;
                relayStartTime  = millis();
                activeShiftIsUp = false;
                relayDuration   = MANUAL_SHIFT_DURATION_MS;
                Serial.println("MANUAL: Downshift CAN sent");
            }
        }

        // Shift Up button
        if (shiftUpState && !lastShiftUpState) {
            if (!relayActive) {
                canSendShiftUp(MANUAL_SHIFT_DURATION_MS, IGN_CUT_DEFAULT_MS);
                relayActive     = true;
                relayStartTime  = millis();
                activeShiftIsUp = true;
                relayDuration   = MANUAL_SHIFT_DURATION_MS;
                Serial.println("MANUAL: Upshift CAN sent");
            }
        }
        
        // Update button states
        lastShiftDownState = shiftDownState;
        lastShiftUpState = shiftUpState;
    }
    
    void updateRelayControl() {
        if (relayActive && millis() - relayStartTime >= relayDuration) {
            relayActive = false;
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
                              "Shift=" + String(relayActive ? "ACTIVE" : "IDLE"));
            }
        }
    }
};

#endif

// end of code