#ifndef GEAR_SENSOR_CONTROL_H
#define GEAR_SENSOR_CONTROL_H

#include <Arduino.h>
#include <Wire.h>

// Forward declaration of the global ISR function
void IRAM_ATTR gearSensorISR();

class GearSensorControl {
    // Make the ISR function a friend so it can access private members
    friend void gearSensorISR();
    
private:
    // Configuration
    int pcfAddress;
    int interruptPin;
    unsigned long gearDebounceDelay;
    
    // State variables
    volatile bool gearChangeInterrupt;
    int currentGear;
    int lastGear;
    unsigned long lastGearChangeTime;
    String gearNames[8] = {"N", "1", "2", "3", "4", "5", "6", "ERR"};
    
    // Connection monitoring
    bool pcfConnected;
    unsigned long lastSensorCheck;
    unsigned long lastSuccessfulRead;
    static const unsigned long SENSOR_CHECK_INTERVAL = 1000;
    static const unsigned long SENSOR_TIMEOUT = 2000;
    
    // Callback function for gear changes
    void (*gearChangeCallback)(int newGear, int oldGear);
    
public:
    // Static instance for ISR access
    static GearSensorControl* instance;
    
    GearSensorControl(int address = 0x20, int intPin = 3, unsigned long debounceMs = 10) 
        : pcfAddress(address), interruptPin(intPin), gearDebounceDelay(debounceMs),
          gearChangeInterrupt(false), currentGear(0), lastGear(0), lastGearChangeTime(0),
          pcfConnected(false), lastSensorCheck(0), lastSuccessfulRead(0),
          gearChangeCallback(nullptr) {
        instance = this; // Set static instance for ISR
    }
    
    void begin(void (*callback)(int, int) = nullptr) {
        gearChangeCallback = callback;
        
        Serial.println("Initializing PCF8575 gear position sensor...");
        
        // Set up interrupt pin for gear change detection
        pinMode(interruptPin, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(interruptPin), gearSensorISR, FALLING);
        
        // Test if PCF8575 is connected
        bool sensorFound = testConnection();
        
        if (sensorFound) {
            // Initialize PCF8575 - all pins as inputs with pull-ups
            Wire.beginTransmission(pcfAddress);
            Wire.write(0xFF);  // Set all pins high (input with pull-up)
            Wire.write(0xFF);
            Wire.endTransmission();
            
            // Read initial gear position
           
            uint16_t initialData = readPCF8575();
            currentGear = decodeGearPosition(initialData);
            
            if (currentGear >= 0) {
                lastGear = currentGear;
                pcfConnected = true;
                lastSuccessfulRead = millis();
                Serial.println("PCF8575 initialized successfully");
                Serial.println("Initial gear position: " + gearNames[currentGear]);
            } else {
                currentGear = 0; // Default to Neutral if invalid reading
                pcfConnected = false;
                Serial.println("PCF8575 found but invalid gear reading - defaulting to Neutral");
            }
        } else {
            // Sensor not connected - use safe defaults
            currentGear = 0; // Default to Neutral
            lastGear = 0;
            pcfConnected = false;
            Serial.println("WARNING: PCF8575 not detected at startup");
            Serial.println("System will continue monitoring and auto-recover when connected");
        }
        
        Serial.println("PCF8575 Address: 0x" + String(pcfAddress, HEX));
        Serial.println("Interrupt Pin: " + String(interruptPin));
        Serial.println("Disconnect detection: 30s timeout, 10s check interval");
    }
    
    void update() {
        unsigned long currentTime = millis();
        
        // Only process interrupts if sensor is connected, but always try to recover if disconnected
        if (!pcfConnected) {
            // Try to recover every 10 seconds if sensor was disconnected
            if (currentTime - lastSensorCheck >= SENSOR_CHECK_INTERVAL) {
                bool connectionStatus = testConnection();
                if (connectionStatus) {
                    Serial.println("PCF8575 sensor reconnected - reinitializing...");
                    
                    
                    // Reinitialize the sensor
                    Wire.beginTransmission(pcfAddress);
                    Wire.write(0xFF);
                    Wire.write(0xFF);
                    Wire.endTransmission();
                    
                    pcfConnected = true;
                    lastSuccessfulRead = currentTime;
                    
                    // Try to read current gear position
                    uint16_t pcfData = readPCF8575();
                    if (pcfData != 0xFFFF) {
                        int newGear = decodeGearPosition(pcfData);
                        if (newGear >= 0 && newGear <= 6) {
                            if (newGear != currentGear) {
                                handleGearChange(newGear, currentGear);
                                lastGear = currentGear;
                                currentGear = newGear;
                            }
                            Serial.println("Sensor recovery successful - current gear: " + gearNames[currentGear]);
                        }
                    }
                }
                lastSensorCheck = currentTime;
            }
            gearChangeInterrupt = false;
            return;
        }
        
        // Sensor is connected - process normally
        if (gearChangeInterrupt) {
            gearChangeInterrupt = false;
            
            // Add debouncing delay
            if (currentTime - lastGearChangeTime >= gearDebounceDelay) {
                
                // Read current gear position
                uint16_t pcfData = readPCF8575();
                
                if (pcfData != 0xFFFF) {
                    // Successful read - update timestamp
                    lastSuccessfulRead = currentTime;
                    
                    int newGear = decodeGearPosition(pcfData);
                    
                    // Check if gear actually changed and is valid
                    if (newGear != currentGear && newGear >= 0 && newGear <= 6) {
                        handleGearChange(newGear, currentGear);
                        lastGear = currentGear;
                        currentGear = newGear;
                        lastGearChangeTime = currentTime;
                    }
                } else {
                    // Failed to read - but don't immediately mark as disconnected
                    if (currentTime - lastSuccessfulRead > SENSOR_TIMEOUT) {
                        Serial.println("PCF8575 sensor timeout - marking as disconnected");
                        pcfConnected = false;
                    }
                }
            }
        }
        
        // Periodic "heartbeat" read to verify sensor is still working
        if (currentTime - lastSensorCheck >= SENSOR_CHECK_INTERVAL) {
            bool connectionStatus = testConnection();
            
            if (!connectionStatus) {
                if (currentTime - lastSuccessfulRead > SENSOR_TIMEOUT) {
                    Serial.println("PCF8575 sensor connection test failed - marking as disconnected");
                    pcfConnected = false;
                }
            } else {
                // Connection test passed - do a quick read to update lastSuccessfulRead
                uint16_t pcfData = readPCF8575();
                if (pcfData != 0xFFFF) {
                    lastSuccessfulRead = currentTime;
                }
            }
            
            lastSensorCheck = currentTime;
        }
    }
    
    // Getters
    int getCurrentGear() const { return currentGear; }
    String getCurrentGearName() const { 
        if (currentGear >= 0 && currentGear <= 6) {
            return gearNames[currentGear];
        }
        return gearNames[7]; // "ERR"
    }
    bool isConnected() const { return pcfConnected; }
    
    String getStatusForWeb() {
        if (!pcfConnected) {
            return "SENSOR DISCONNECTED";
        } else if (currentGear >= 0 && currentGear <= 6) {
            return gearNames[currentGear];
        } else {
            return "ERROR";
        }
    }
    
    void printStatus() {
        Serial.println("=== GEAR SENSOR STATUS ===");
        Serial.println("PCF8575 Address: 0x" + String(pcfAddress, HEX));
        Serial.println("Connected: " + String(pcfConnected ? "YES" : "NO"));
        Serial.println("Current Gear: " + String(currentGear) + " (" + gearNames[constrain(currentGear, 0, 7)] + ")");
        Serial.println("Last Successful Read: " + String((millis() - lastSuccessfulRead) / 1000) + "s ago");
        
        if (pcfConnected) {
            uint16_t rawData = readPCF8575();
            Serial.println("Raw PCF8575 Data: 0x" + String(rawData, HEX));
            Serial.println("Binary: " + String(rawData, BIN));
            
            // Show individual pin states
            for (int i = 0; i <= 6; i++) {
                bool pinState = rawData & (1 << i);
                Serial.println("  P" + String(i, DEC) + " (Gear " + gearNames[i] + "): " + 
                              String(pinState ? "HIGH" : "LOW (ACTIVE)"));
            }
        }
        Serial.println("===========================");
    }
    
private:
    bool testConnection() {
        Wire.beginTransmission(pcfAddress);
        byte error = Wire.endTransmission();
        return (error == 0);
    }
    
    uint16_t readPCF8575() {
        Wire.requestFrom(pcfAddress, 2);
        
        unsigned long startTime = millis();
        if (Wire.available() < 2) {
            if (millis() - startTime <= 50) {
                return 0xFFFF; // Try again later
            }
        }
        
        if (Wire.available() >= 2) {
            uint8_t lowByte = Wire.read();
            uint8_t highByte = Wire.read();
            return (uint16_t(highByte) << 8) | lowByte;
        }
        
        return 0xFFFF;
    }
    
    int decodeGearPosition(uint16_t pcfData) {
        // Check for invalid/error data
        if (pcfData == 0xFFFF) {
            return -1; // Error condition
        }
        
        // Check each gear position pin (active LOW)
        // P00 = Neutral, P01 = Gear 1, P02 = Gear 2, etc.
        for (int i = 0; i <= 6; i++) {
            if (!(pcfData & (1 << i))) {  // Check if pin is LOW (active)
                return i;  // Return gear number (0 = Neutral, 1-6 = Gears)
            }
        }
        
        // If all pins are high (no gear detected), keep current gear
        return currentGear;
    }
    
    void handleGearChange(int newGear, int oldGear) {
        // Validate gear values before processing
        if (newGear < 0 || newGear > 6 || oldGear < 0 || oldGear > 6) {
            Serial.println("Invalid gear change detected - ignoring");
            return;
        }
        
        // Handle gear change event
        Serial.print("Gear changed from ");
        Serial.print(gearNames[oldGear]);
        Serial.print(" to ");
        Serial.println(gearNames[newGear]);
        
        // Call the callback function if set
        if (gearChangeCallback) {
            gearChangeCallback(newGear, oldGear);
        }
    }
};

// Static member definition - moved outside class
GearSensorControl* GearSensorControl::instance = nullptr;

// Global ISR function definition
void IRAM_ATTR gearSensorISR() {
    if (GearSensorControl::instance) {
        GearSensorControl::instance->gearChangeInterrupt = true;
    }
}

#endif

// end of code