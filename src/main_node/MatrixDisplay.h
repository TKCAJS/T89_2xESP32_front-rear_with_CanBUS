#ifndef MATRIX_DISPLAY_H
#define MATRIX_DISPLAY_H

#include <Arduino.h>
#include <Adafruit_NeoMatrix.h>
#include "MatrixStartupAnimation.h"

// Matrix Configuration
#define MATRIX_PIN 16
#define MATRIX_WIDTH 8
#define MATRIX_HEIGHT 8

class MatrixDisplay {
private:
    Adafruit_NeoMatrix* matrix;
    
    // Display state variables
    bool showShiftNotification;
    char shiftNotificationChar;
    unsigned long shiftNotificationStart;
    uint8_t notifR, notifG, notifB;
    unsigned long lastRainbowUpdate;
    uint16_t rainbowOffset;
    
    // Configuration
    static const unsigned long SHIFT_NOTIFICATION_DURATION = 300;
    static const unsigned long CYCLE_TIME = 1000;
    static const unsigned long FLASH_DURATION = 70;
    static const unsigned long FLASH_GAP = 150;
    
    // External references needed
    bool* wifiEnabled;
    bool* canGearValid;
    bool* manualModeEnabled;

    volatile bool animRunning = false;

    static void animationTask(void* param) {
        MatrixDisplay* self = static_cast<MatrixDisplay*>(param);
        runMatrixStartupAnimation(self->matrix);
        self->animRunning = false;
        vTaskDelete(nullptr);
    }

public:
    MatrixDisplay() : matrix(nullptr), showShiftNotification(false),
                     shiftNotificationChar(' '), shiftNotificationStart(0),
                     notifR(255), notifG(255), notifB(255),
                     lastRainbowUpdate(0), rainbowOffset(0),
                     wifiEnabled(nullptr), canGearValid(nullptr),
                     manualModeEnabled(nullptr), animRunning(false) {}

    void begin(bool* wifiEnabledPtr, bool* canGearValidPtr, bool* manualModePtr = nullptr, bool startupAnim = true) {
        wifiEnabled = wifiEnabledPtr;
        canGearValid = canGearValidPtr;
        manualModeEnabled = manualModePtr;

        matrix = new Adafruit_NeoMatrix(MATRIX_WIDTH, MATRIX_HEIGHT, MATRIX_PIN,
            NEO_MATRIX_TOP + NEO_MATRIX_LEFT + NEO_MATRIX_ROWS + NEO_MATRIX_PROGRESSIVE,
            NEO_GRB + NEO_KHZ800);

        matrix->begin();
        matrix->setBrightness(50);
        matrix->setTextWrap(false);
        matrix->setTextColor(matrix->Color(255, 255, 255));
        matrix->setTextSize(1);
        matrix->fillScreen(0);
        matrix->show();

        if (startupAnim) {
            animRunning = true;
            xTaskCreate(animationTask, "MatrixAnim", 4096, this, 1, nullptr);
        }
    }
    
    void update(const String& currentGearName) {
        if (animRunning) return;
        unsigned long currentMillis = millis();

        // Check if we should stop displaying the shift notification
        if (showShiftNotification && (currentMillis - shiftNotificationStart >= SHIFT_NOTIFICATION_DURATION)) {
            showShiftNotification = false;
        }
        
        // Update display every 50ms
        if (currentMillis - lastRainbowUpdate >= 50) {
            lastRainbowUpdate = currentMillis;
            
            if (showShiftNotification) {
                matrix->fillScreen(0);
                matrix->setTextColor(matrix->Color(notifR, notifG, notifB));
                matrix->setCursor(0, 0);
                matrix->print(shiftNotificationChar);
                
            } else {
                // ALWAYS show current gear when not showing notification
                matrix->fillScreen(0); // Clear screen (black)
                
                // Handle gear sensor disconnected state
                if (!(*canGearValid)) {
                    matrix->setTextColor(matrix->Color(255, 0, 0)); // Red text for error
                    matrix->setCursor(0, 0);
                    matrix->print("?");
                } else {
                    // Set color based on gear: Red for 1-6, Green for N
                    if (currentGearName == "N") {
                        matrix->setTextColor(matrix->Color(0, 255, 0)); // Green for Neutral
                    } else {
                        matrix->setTextColor(matrix->Color(255, 0, 0)); // Red for gear numbers 1-6
                    }
                    
                    // Center the gear character on the 8x8 display
                    matrix->setCursor(0, 0);
                    matrix->print(currentGearName);
                }
            }
            
            // ADD MANUAL MODE INDICATOR - Flash bottom row amber
            addManualModeIndicator(currentMillis);
            
            // ADD HEARTBEAT TO TOP-RIGHT PIXEL (position 7,0)
            addHeartbeat(currentMillis);
            
            matrix->show();
        }
    }
    
    void updateWithTachometer(const String& currentGearName, float currentRpm) {
        if (animRunning) return;
        unsigned long currentMillis = millis();

        // Check if we should stop displaying the shift notification
        if (showShiftNotification && (currentMillis - shiftNotificationStart >= SHIFT_NOTIFICATION_DURATION)) {
            showShiftNotification = false;
        }
        
        // Update display every 50ms
        if (currentMillis - lastRainbowUpdate >= 50) {
            lastRainbowUpdate = currentMillis;
            
            // ALWAYS update the tachometer (left column) first
            updateTachometer(currentRpm);
            
            if (showShiftNotification) {
                // Display the shift notification (U/D) temporarily in center area
                // Clear only the gear display area (not the tachometer column)
                for (int x = 1; x < 8; x++) {
                    for (int y = 0; y < 8; y++) {
                        matrix->drawPixel(x, y, matrix->Color(0, 0, 0));
                    }
                }
                
                matrix->setTextColor(matrix->Color(notifR, notifG, notifB));
                matrix->setCursor(2, 0);
                matrix->print(shiftNotificationChar);
                
            } else {
                // ALWAYS show current gear when not showing notification
                // Clear only the gear display area (not the tachometer column)
                for (int x = 1; x < 8; x++) {
                    for (int y = 0; y < 8; y++) {
                        matrix->drawPixel(x, y, matrix->Color(0, 0, 0));
                    }
                }
                
                // Get current gear name and display it
                // Handle gear sensor disconnected state
                if (!(*canGearValid)) {
                    matrix->setTextColor(matrix->Color(255, 0, 0)); // Red text for error
                    matrix->setCursor(2, 0);  // Adjusted for non-tachometer area
                    matrix->print("?");
                } else {
                    // Set color based on gear: Red for 1-6, Green for N
                    if (currentGearName == "N") {
                        matrix->setTextColor(matrix->Color(0, 255, 0)); // Green for Neutral
                    } else {
                        matrix->setTextColor(matrix->Color(255, 0, 0)); // Red for gear numbers 1-6
                    }
                    
                    // Center the gear character in non-tachometer area
                    matrix->setCursor(2, 0);  // Adjusted for non-tachometer area
                    matrix->print(currentGearName);
                }
            }
            
            // ADD MANUAL MODE INDICATOR - Flash bottom row amber
            addManualModeIndicator(currentMillis);
            
            // ADD HEARTBEAT TO TOP-RIGHT PIXEL (position 7,0)
            addHeartbeat(currentMillis);
            
            matrix->show();
        }
    }
    
    void displayShiftNotification(char notificationChar, uint8_t r = 255, uint8_t g = 255, uint8_t b = 255) {
        shiftNotificationChar = notificationChar;
        notifR = r; notifG = g; notifB = b;
        showShiftNotification = true;
        shiftNotificationStart = millis();
    }

    void displayShiftLetter(char letter) {
        displayShiftNotification(letter);
    }
    
private:
    void updateTachometer(float rpm) {
        int ledsToLight = 0;
        uint32_t color = matrix->Color(0, 0, 0);
        bool shouldFlash = false;

        if (rpm >= 100) {
            if (rpm < 3000) {
                ledsToLight = 1;
                color = matrix->Color(0, 255, 0); // Green
            } else if (rpm < 5000) {
                ledsToLight = 2;
                color = matrix->Color(0, 255, 0); // Green
            } else if (rpm < 7500) {
                ledsToLight = 3;
                color = matrix->Color(0, 255, 0); // Green
            } else if (rpm < 9000) {
                ledsToLight = 4;
                color = matrix->Color(0, 255, 0); // Yellow
            } else if (rpm < 10500) {
                ledsToLight = 5;
                color = matrix->Color(255, 255, 0); // Yellow
            } else if (rpm < 12000) {
                ledsToLight = 6;
                color = matrix->Color(255, 165, 0); // Orange
            } else if (rpm < 13500) {
                ledsToLight = 7;
                color = matrix->Color(255, 0, 0); // Red
            } else {
                ledsToLight = 8;
                color = matrix->Color(255, 0, 0); // Red
                shouldFlash = true;
            }
        }
        
        // Clear the left column (x=0)
        for (int y = 0; y < 8; y++) {
            matrix->drawPixel(0, y, matrix->Color(0, 0, 0));
        }
        
        // Light up the appropriate LEDs from bottom to top
        for (int i = 0; i < ledsToLight; i++) {
            int y = 7 - i; // Bottom to top (y=7 is bottom, y=0 is top)
            
            if (shouldFlash && (millis() / 200) % 2) {
                // Flash by turning off every 200ms for redline
                matrix->drawPixel(0, y, matrix->Color(0, 0, 0));
            } else {
                matrix->drawPixel(0, y, color);
            }
        }
    }
    
    void addManualModeIndicator(unsigned long currentMillis) {
        // Only show manual mode indicator if manual mode is enabled and reference is valid
        if (manualModeEnabled && *manualModeEnabled) {
            // Flash the bottom row (y=7) amber at 500ms intervals
            bool flashOn = (currentMillis / 500) % 2;
            uint32_t amberColor = flashOn ? matrix->Color(255, 191, 0) : matrix->Color(0, 0, 0);
            
            // Light up entire bottom row (y=7, all x positions)
            for (int x = 0; x < 8; x++) {
                matrix->drawPixel(x, 7, amberColor);
            }
        }
        // If manual mode is disabled or reference is null, bottom row stays black (already cleared)
    }
    
    void addHeartbeat(unsigned long currentMillis) {
        // Copy the same heartbeat logic from the main LED
        unsigned long elapsed = currentMillis % CYCLE_TIME;
        
        if (elapsed < FLASH_DURATION || 
            (elapsed > FLASH_DURATION + FLASH_GAP && 
             elapsed < FLASH_DURATION * 2 + FLASH_GAP)) {
            // Heartbeat active - show red
            matrix->drawPixel(7, 0, matrix->Color(255, 0, 0));
        } else {
            // Heartbeat background - blue intensity based on WiFi status
            int blueLevel = (*wifiEnabled) ? 255 : 5;  // Bright blue if WiFi on, dim if off
            matrix->drawPixel(7, 0, matrix->Color(0, 10, blueLevel));
        }
    }
    
    // Color wheel function for rainbow colors (kept for potential future use)
    uint16_t wheel(byte wheelPos) {
        wheelPos = 255 - wheelPos;
        if (wheelPos < 85) {
            return matrix->Color(255 - wheelPos * 3, 0, wheelPos * 3);
        }
        if (wheelPos < 170) {
            wheelPos -= 85;
            return matrix->Color(0, wheelPos * 3, 255 - wheelPos * 3);
        }
        wheelPos -= 170;
        return matrix->Color(wheelPos * 3, 255 - wheelPos * 3, 0);
    }
};

#endif

// end of code