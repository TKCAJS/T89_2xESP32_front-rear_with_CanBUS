#ifndef MATRIX_TACHOMETER_H
#define MATRIX_TACHOMETER_H

#include <Adafruit_NeoMatrix.h>

// RPM thresholds for tachometer bar graph
#define TACH_IDLE_RPM       100
#define TACH_LOW_RPM        3000
#define TACH_MID_LOW_RPM    5500
#define TACH_MID_RPM        7500
#define TACH_MID_HIGH_RPM   9000
#define TACH_HIGH_RPM      10500
#define TACH_REDLINE_RPM   13000
#define TACH_OVERREV_RPM   13500

class MatrixTachometer {
private:
    Adafruit_NeoMatrix* matrix;
    
    // RPM thresholds array for easy calculation
    const int rpmThresholds[8] = {
        TACH_IDLE_RPM,      // Pixel 0 (bottom)
        TACH_LOW_RPM,       // Pixel 1
        TACH_MID_LOW_RPM,   // Pixel 2
        TACH_MID_RPM,       // Pixel 3
        TACH_MID_HIGH_RPM,  // Pixel 4
        TACH_HIGH_RPM,      // Pixel 5
        TACH_REDLINE_RPM,   // Pixel 6
        TACH_OVERREV_RPM    // Pixel 7 (top)
    };
    
    // Color definitions for different RPM ranges
    uint16_t getPixelColor(int pixelIndex, float currentRpm) {
        // Determine if this pixel should be lit based on RPM
        bool shouldLight = (currentRpm >= rpmThresholds[pixelIndex]);
        
        if (!shouldLight) {
            return matrix->Color(0, 0, 0); // Off (black)
        }
        
        // Color based on RPM range
        if (pixelIndex <= 4) {
            // Green for normal operating range (1000-9000 RPM)
            return matrix->Color(0, 255, 0);
        } else if (pixelIndex <= 5) {
            // Amber for high RPM (11000 RPM)
            return matrix->Color(255, 150, 0);
        } else {
            // Red for redline and over-rev (13000+ RPM)
            return matrix->Color(255, 0, 0);
        }
    }
    
    // Calculate brightness based on how close to next threshold
    uint8_t calculateBrightness(int pixelIndex, float currentRpm) {
        if (currentRpm < rpmThresholds[pixelIndex]) {
            return 0; // Off
        }
        
        // For the highest lit pixel, calculate fade-in effect
        if (pixelIndex < 7 && currentRpm < rpmThresholds[pixelIndex + 1]) {
            float progress = (currentRpm - rpmThresholds[pixelIndex]) / 
                           (rpmThresholds[pixelIndex + 1] - rpmThresholds[pixelIndex]);
            return 100 + (155 * progress); // Fade from 100 to 255 brightness
        }
        
        return 255; // Full brightness for passed thresholds
    }

public:
    MatrixTachometer(Adafruit_NeoMatrix* matrixPtr) : matrix(matrixPtr) {}
    
    void updateTachometer(float currentRpm) {
        // Update left column (column 0) with tachometer bar graph
        // Note: Matrix coordinates are (x, y) where (0,0) is top-left
        // We want bottom-up display, so pixel 0 is at y=7, pixel 7 is at y=0
        
        for (int i = 0; i < 8; i++) {
            int yPos = 7 - i; // Invert Y coordinate for bottom-up display
            
            uint16_t pixelColor = getPixelColor(i, currentRpm);
            uint8_t brightness = calculateBrightness(i, currentRpm);
            
            if (brightness > 0 && pixelColor != matrix->Color(0, 0, 0)) {
                // Apply brightness scaling to color components
                uint8_t r = ((pixelColor >> 11) & 0x1F) * brightness / 255;
                uint8_t g = ((pixelColor >> 5) & 0x3F) * brightness / 255;
                uint8_t b = (pixelColor & 0x1F) * brightness / 255;
                
                // Convert back to 16-bit color
                pixelColor = matrix->Color(r << 3, g << 2, b << 3);
            }
            
            matrix->drawPixel(0, yPos, pixelColor);
        }
    }
    
    // Method to clear the tachometer column
    void clearTachometer() {
        for (int i = 0; i < 8; i++) {
            matrix->drawPixel(0, 7 - i, matrix->Color(0, 0, 0));
        }
    }
    
    // Method to test the tachometer with a sweep
    void testSweep() {
        for (int rpm = 0; rpm <= 14000; rpm += 200) {
            updateTachometer(rpm);
            matrix->show();
            delay(50);
        }
        
        // Hold at redline for a moment
        delay(500);
        
        // Sweep back down
        for (int rpm = 14000; rpm >= 0; rpm -= 200) {
            updateTachometer(rpm);
            matrix->show();
            delay(30);
        }
    }
    
    // Get RPM range information for debugging
    String getRpmRangeInfo(float currentRpm) {
        String info = "RPM: " + String(currentRpm, 0) + " | Range: ";
        
        if (currentRpm < TACH_IDLE_RPM) {
            info += "Below Idle";
        } else if (currentRpm < TACH_LOW_RPM) {
            info += "Idle";
        } else if (currentRpm < TACH_MID_LOW_RPM) {
            info += "Low";
        } else if (currentRpm < TACH_MID_RPM) {
            info += "Mid-Low";
        } else if (currentRpm < TACH_MID_HIGH_RPM) {
            info += "Mid";
        } else if (currentRpm < TACH_HIGH_RPM) {
            info += "Mid-High";
        } else if (currentRpm < TACH_REDLINE_RPM) {
            info += "High";
        } else if (currentRpm < TACH_OVERREV_RPM) {
            info += "REDLINE";
        } else {
            info += "OVER-REV";
        }
        
        return info;
    }
    
    // Serial command to show RPM thresholds
    void printRpmThresholds() {
        Serial.println("=== MATRIX TACHOMETER RPM THRESHOLDS ===");
        Serial.println("Pixel | RPM    | Color  | Description");
        Serial.println("------|--------|--------|------------------");
        Serial.printf("  0   | %5d  | GREEN  | Idle (bottom)\n", TACH_IDLE_RPM);
        Serial.printf("  1   | %5d  | GREEN  | Low RPM\n", TACH_LOW_RPM);
        Serial.printf("  2   | %5d  | GREEN  | Mid-Low RPM\n", TACH_MID_LOW_RPM);
        Serial.printf("  3   | %5d  | GREEN  | Mid RPM\n", TACH_MID_RPM);
        Serial.printf("  4   | %5d  | GREEN  | Mid-High RPM\n", TACH_MID_HIGH_RPM);
        Serial.printf("  5   | %5d  | AMBER  | High RPM\n", TACH_HIGH_RPM);
        Serial.printf("  6   | %5d  | RED    | Redline\n", TACH_REDLINE_RPM);
        Serial.printf("  7   | %5d  | RED    | Over-Rev (top)\n", TACH_OVERREV_RPM);
        Serial.println("=========================================");
        Serial.println("Note: Left column (column 0) is used for tachometer");
        Serial.println("Commands: 'tachotest' - Test sweep, 'tachoinfo' - Show thresholds");
    }
};

#endif // MATRIX_TACHOMETER_H

// end of code