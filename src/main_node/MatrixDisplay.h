#ifndef MATRIX_DISPLAY_H
#define MATRIX_DISPLAY_H

#include <Arduino.h>
#include <Adafruit_NeoMatrix.h>

// Matrix Configuration
#define MATRIX_PIN 38
#define MATRIX_WIDTH 8
#define MATRIX_HEIGHT 8

class MatrixDisplay {
private:
    Adafruit_NeoMatrix* matrix;
    
    // Display state variables
    bool showShiftNotification;
    char shiftNotificationChar;
    unsigned long shiftNotificationStart;
    unsigned long lastRainbowUpdate;
    uint16_t rainbowOffset;
    
    // Configuration
    static const unsigned long SHIFT_NOTIFICATION_DURATION = 300;
    static const unsigned long CYCLE_TIME = 1000;
    static const unsigned long FLASH_DURATION = 70;
    static const unsigned long FLASH_GAP = 150;
    
    // External references needed
    bool* wifiEnabled;
    bool* pcf8575Connected;
    bool* manualModeEnabled;  // NEW: Manual mode reference
    
public:
    MatrixDisplay() : matrix(nullptr), showShiftNotification(false), 
                     shiftNotificationChar(' '), shiftNotificationStart(0),
                     lastRainbowUpdate(0), rainbowOffset(0),
                     wifiEnabled(nullptr), pcf8575Connected(nullptr),
                     manualModeEnabled(nullptr) {}
    
    void begin(bool* wifiEnabledPtr, bool* pcf8575ConnectedPtr, bool* manualModePtr = nullptr) {
        wifiEnabled = wifiEnabledPtr;
        pcf8575Connected = pcf8575ConnectedPtr;
        manualModeEnabled = manualModePtr;
        
        // Initialize Dotstar Matrix with corrected wiring pattern
        matrix = new Adafruit_NeoMatrix(MATRIX_WIDTH, MATRIX_HEIGHT, MATRIX_PIN,
            NEO_MATRIX_TOP + NEO_MATRIX_RIGHT + NEO_MATRIX_ROWS + NEO_MATRIX_PROGRESSIVE,
            NEO_GRB + NEO_KHZ800);
        
        Serial.println("Initializing Dotstar 8x8 Matrix...");
        matrix->begin();
        matrix->setBrightness(50); // Set to 50% brightness to avoid being too bright
        matrix->setTextWrap(false); // Don't wrap text
        matrix->setTextColor(matrix->Color(255, 255, 255)); // White text
        matrix->setTextSize(1); // Use default size
        matrix->fillScreen(0); // Clear screen
        matrix->show();
    }
    
    void update(const String& currentGearName) {
        unsigned long currentMillis = millis();
        
        // Check if we should stop displaying the shift notification
        if (showShiftNotification && (currentMillis - shiftNotificationStart >= SHIFT_NOTIFICATION_DURATION)) {
            showShiftNotification = false;
        }
        
        // Update display every 50ms
        if (currentMillis - lastRainbowUpdate >= 50) {
            lastRainbowUpdate = currentMillis;
            
            if (showShiftNotification) {
                // Display the shift notification (U/D) temporarily
                matrix->fillScreen(0); // Clear screen (black)
                matrix->setTextColor(matrix->Color(255, 255, 255)); // White text
                
                // Center the character on the 8x8 display
                matrix->setCursor(0, 0);
                matrix->print(shiftNotificationChar);
                
            } else {
                // ALWAYS show current gear when not showing notification
                matrix->fillScreen(0); // Clear screen (black)
                
                // Handle gear sensor disconnected state
                if (!(*pcf8575Connected)) {
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
                
                matrix->setTextColor(matrix->Color(255, 255, 255)); // White text
                
                // Center the character on the remaining area (columns 1-7)
                matrix->setCursor(2, 0);  // Adjusted for non-tachometer area
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
                if (!(*pcf8575Connected)) {
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
    
    void displayShiftNotification(char notificationChar) {
        shiftNotificationChar = notificationChar;
        showShiftNotification = true;
        shiftNotificationStart = millis();
    }
    
    void displayShiftLetter(char letter) {
        displayShiftNotification(letter);
    }
    
    // Matrix tachometer methods from MatrixTachometer.h
    void testSweep() {
        Serial.println("=== MATRIX TACHOMETER TEST SWEEP ===");
        Serial.println("Animating RPM sweep from 0 to 13500 RPM...");
        
        for (int rpm = 0; rpm <= 13500; rpm += 250) {
            updateTachometer(rpm);
            matrix->show();
            delay(50);
            yield();
        }
        
        Serial.println("Sweep complete - returning to normal operation");
    }
    
    void printRpmThresholds() {
        Serial.println("=== MATRIX TACHOMETER RPM THRESHOLDS ===");
        Serial.println("RPM Range    | LEDs | Color");
        Serial.println("-------------|------|-------");
        Serial.println("0 - 1000     |  0   | Off");
        Serial.println("1000 - 2500  |  1   | Green");
        Serial.println("2500 - 4000  |  2   | Green");
        Serial.println("4000 - 5500  |  3   | Green");
        Serial.println("5500 - 7000  |  4   | Yellow");
        Serial.println("7000 - 8500  |  5   | Yellow");
        Serial.println("8500 - 10000 |  6   | Orange");
        Serial.println("10000 - 11500|  7   | Red");
        Serial.println("11500 - 13000|  8   | Red");
        Serial.println("13000+       |  8   | Red (Flashing)");
        Serial.println("=========================================");
    }
    
    String getRpmRangeInfo(float currentRpm) {
        String info = "Current RPM: " + String(currentRpm, 0);
        
        if (currentRpm < 1000) {
            info += " | Range: Idle | LEDs: 0 | Color: Off";
        } else if (currentRpm < 2500) {
            info += " | Range: Low | LEDs: 1 | Color: Green";
        } else if (currentRpm < 4000) {
            info += " | Range: Low-Mid | LEDs: 2 | Color: Green";
        } else if (currentRpm < 5500) {
            info += " | Range: Mid | LEDs: 3 | Color: Green";
        } else if (currentRpm < 7000) {
            info += " | Range: Mid-High | LEDs: 4 | Color: Yellow";
        } else if (currentRpm < 8500) {
            info += " | Range: High | LEDs: 5 | Color: Yellow";
        } else if (currentRpm < 10000) {
            info += " | Range: High+ | LEDs: 6 | Color: Orange";
        } else if (currentRpm < 11500) {
            info += " | Range: Very High | LEDs: 7 | Color: Red";
        } else if (currentRpm < 13000) {
            info += " | Range: Near Redline | LEDs: 8 | Color: Red";
        } else {
            info += " | Range: REDLINE! | LEDs: 8 | Color: Red (Flashing)";
        }
        
        return info;
    }
    
private:
    void updateTachometer(float rpm) {
        // Calculate how many LEDs should be lit (0-8 LEDs in left column)
        int ledsToLight = 0;
        uint32_t color = matrix->Color(0, 0, 0); // Default off
        bool shouldFlash = false;
        
        if (rpm >= 1000) {
            if (rpm < 2500) {
                ledsToLight = 1;
                color = matrix->Color(0, 255, 0); // Green
            } else if (rpm < 4000) {
                ledsToLight = 2;
                color = matrix->Color(0, 255, 0); // Green
            } else if (rpm < 5500) {
                ledsToLight = 3;
                color = matrix->Color(0, 255, 0); // Green
            } else if (rpm < 7000) {
                ledsToLight = 4;
                color = matrix->Color(255, 255, 0); // Yellow
            } else if (rpm < 8500) {
                ledsToLight = 5;
                color = matrix->Color(255, 255, 0); // Yellow
            } else if (rpm < 10000) {
                ledsToLight = 6;
                color = matrix->Color(255, 165, 0); // Orange
            } else if (rpm < 11500) {
                ledsToLight = 7;
                color = matrix->Color(255, 0, 0); // Red
            } else if (rpm < 13000) {
                ledsToLight = 8;
                color = matrix->Color(255, 0, 0); // Red
            } else {
                ledsToLight = 8;
                color = matrix->Color(255, 0, 0); // Red
                shouldFlash = true; // Flash for redline
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