// 206    Solved multiple timing issues - still work in progress
// 205_1  altered downshift to use servo data instead of delays
// T89_gearbox_205.ino - MODULAR REFACTOR: Separated components into dedicated header files
// 205.0  MAJOR REFACTOR: Modularized code into separate header files for improved maintainability
//        - Moved matrix display functions to MatrixDisplay.h
//        - Moved hall sensor control to HallSensorControl.h
//        - Moved gear sensor functions to GearSensorControl.h
//        - Moved serial commands to SerialCommands.h
//        - Significantly reduced main file complexity while maintaining all functionality
// 205.1  ADDED: Manual Mode for direct racing control

#include <Arduino.h>

// Version tracking
#define SOFTWARE_VERSION 206.0

// Pin definitions for ESP32-S3 - FIXED PIN ASSIGNMENTS
#define PIN_NEUTRAL_DOWN    10   // Switch 1 - Neutral Down
#define PIN_NEUTRAL_UP      11   // Switch 2 - Neutral Up  
#define PIN_SHIFT_DOWN      12   // Switch 3 - Shift Down
#define PIN_SHIFT_UP        13   // Switch 4 - Shift Up
#define PIN_HALL_SENSOR     5    // Hall sensor analog input
#define PIN_CLUTCH_SERVO    6    // Servo output
#define PIN_DOWNSHIFT_RELAY 7    // Downshift relay
#define PIN_UPSHIFT_RELAY   8    // Upshift relay
#define PIN_WIFI_SWITCH     9    // WiFi toggle switch input (momentary)
#define PIN_CLUTCH_POSITION 15   // Analog voltage input for clutch position monitoring

// Relay configuration
#define RELAY_ACTIVE_LOW false
#define RELAY_ON (RELAY_ACTIVE_LOW ? LOW : HIGH)
#define RELAY_OFF (RELAY_ACTIVE_LOW ? HIGH : LOW)

// Clutch position threshold
#define CLUTCH_THRESHOLD_VOLTAGE 1.8

// Standard includes
#include <LittleFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>

// Project includes - MODULAR COMPONENTS (Order matters for dependencies)
#include "Speed.h"
#include "RPM.h"
#include "ShiftLogger.h"
#include "GearboxStateMachine.h"
#include "SimpleServo.h"
#include "HallResponseTypes.h"      // NEW: Shared enum definitions
#include "HallSensorControl.h"      // NEW: Hall sensor control
#include "GearSensorControl.h"      // NEW: Gear sensor control  
#include "MatrixDisplay.h"          // NEW: Matrix display functions
#include "WebInterface.h"           // WebInterface needs HallResponseTypes
#include "SerialCommands.h"         // NEW: Serial command processing (MUST be last)
#include "ManualMode.h"             // NEW: Manual mode control

// Global variables for Speed and RPM sensors
volatile unsigned long g_speedPulseCount = 0;
volatile unsigned long g_rpmPulseCount = 0;

// ISR functions for Speed and RPM sensors
void IRAM_ATTR speedISR() {
    g_speedPulseCount++;
}

void IRAM_ATTR rpmISR() {
    g_rpmPulseCount++;
}

// LED and timing configuration
#define PIN 48
#define NUM_LEDS 1
#define FLASH_DURATION 70
#define FLASH_GAP 150
#define CYCLE_TIME 1000

// Speed and RPM input pins
#define PIN_RPM_INPUT      14
#define PIN_MPH_INPUT      16

// I2C configuration
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 45
#define PCF8575_ADDRESS 0x20
#define PCF8575_INTERRUPT_PIN 3

// Global state for matrix display
bool pcf8575Connected = false;
bool manualModeActive = false;  // NEW: Manual mode status for matrix

// Create controller instances - MOVED AFTER INCLUDES TO ENSURE COMPLETE TYPES
Speed speedSensor(PIN_MPH_INPUT);
RPM rpmSensor(PIN_RPM_INPUT, 12.0, 0.3);
ShiftLogger shiftLogger;
GearboxStateMachine gearbox(PIN_DOWNSHIFT_RELAY, PIN_UPSHIFT_RELAY, PIN_HALL_SENSOR);

// Create web server and interface
WebServer server(80);
WebInterface webInterface(&server);

// NEW: Modular components - MOVED AFTER INCLUDES
MatrixDisplay matrixDisplay;
HallSensorControl hallSensor(PIN_HALL_SENSOR);
GearSensorControl gearSensor(PCF8575_ADDRESS, PCF8575_INTERRUPT_PIN);
SerialCommands serialCommands;

// NEW: Manual mode controller
ManualMode manualMode(PIN_NEUTRAL_DOWN, PIN_NEUTRAL_UP, 
                     PIN_SHIFT_DOWN, PIN_SHIFT_UP,
                     PIN_DOWNSHIFT_RELAY, PIN_UPSHIFT_RELAY);

// Configuration variables
int neutralDownMs = 40;
int neutralUpMs = 40;
int shiftDownMs = 150;
int shiftUpMs = 150;
int clutchIdlePos = 0;
int clutchEngagePos = 180;

// WiFi state
bool wifiEnabled = false;
bool lastWifiSwitchState = HIGH;
unsigned long lastWifiButtonPress = 0;
#define WIFI_DEBOUNCE_DELAY 300

// Clutch monitoring
bool clutchPulled = false;
float clutchVoltage = 0.0;

// WiFi Access Point credentials
const char* ssid = "T89_Gearbox";
const char* password = "12345678";

// Objects
Preferences prefs;
SimpleServo clutchServo;
Adafruit_NeoPixel pixels(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);

// LED colors
int colr = 255;
int colg = 255;
int colb = 5;

// Button state tracking
bool lastNeutralDownState = HIGH;
bool lastNeutralUpState = HIGH;
bool lastShiftDownState = HIGH;
bool lastShiftUpState = HIGH;

// Compatibility variables for WebInterface.h
bool shiftInProgress = false;
bool waitingForClutch = false;
int shiftSequenceState = 0;
bool autoDownshift = false;
unsigned long clutchStartTime = 0;
RelayState downshiftRelay;
RelayState upshiftRelay;
int currentGear = 0;
String gearNames[] = {"N", "1", "2", "3", "4", "5", "6", "ERR"};

// Hall sensor globals for web interface compatibility
HallResponseCurve hallCurveType = HALL_LOGARITHMIC;
float hallCurveStrength = 2.0;

// Function declarations
void setupPins();
void setupWiFiAP();
void disableWiFi();
void checkWiFiToggleSwitch();
void toggleWiFi();
void loadConfig();
void saveConfig();
void processInputs();
void checkServoPosition();
void setupWeb();
void updateCompatibilityVariables();

// Legacy functions for WebInterface compatibility
bool isShiftAllowed() { return gearbox.canAcceptShiftCommand(); }
bool canDownshift() { return gearbox.canAcceptShiftCommand() && clutchPulled; }
void setShiftInProgress(bool inProgress) { /* Now handled by state machine */ }
void startDownshiftWithClutchCheck(int durationMs) { 
    gearbox.processEvent(EVENT_NEUTRAL_DOWN_PRESSED); 
}
void startRelayActivation(int pin, int durationMs) { /* Now handled by state machine */ }
void engageClutch() { clutchServo.write(clutchEngagePos); }
void releaseClutch() { clutchServo.write(clutchIdlePos); }
void displayShiftLetter(char letter) { matrixDisplay.displayShiftLetter(letter); }
String getGearStatusForWeb() { return gearSensor.getStatusForWeb(); }
String getHallCurveTypeName() { return hallSensor.getCurveTypeName(); }
void saveHallCurveConfig() { /* Handled by HallSensorControl */ }

//===========================================
// SERIAL COMMANDS IMPLEMENTATION
//===========================================
// Implementation moved here to avoid incomplete type issues

void SerialCommands::processCommands() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        // Handle multi-part commands for hall sensor curves
        if (command.startsWith("curve ")) {
            String curveType = command.substring(6);
            if (hallSensor) {
                hallSensor->setCurveType(curveType);
            } else {
                Serial.println("Hall sensor not initialized");
            }
        }
        else if (command.startsWith("strength ")) {
            String strengthStr = command.substring(9);
            float strength = strengthStr.toFloat();
            if (strength > 0) {
                if (hallSensor) {
                    hallSensor->setCurveStrength(strength);
                } else {
                    Serial.println("Hall sensor not initialized");
                }
            } else {
                Serial.println("Invalid strength value. Use range 0.1-5.0");
            }
        }
        else if (command.equalsIgnoreCase("hallinfo")) {
            if (hallSensor) {
                hallSensor->printInfo();
            } else {
                Serial.println("Hall sensor not initialized");
            }
        }
        else if (command.equalsIgnoreCase("halltest")) {
            if (hallSensor) {
                hallSensor->runTest();
            } else {
                Serial.println("Hall sensor not initialized");
            }
        }
        // Matrix tachometer commands
        else if (command.equalsIgnoreCase("tachotest")) {
            Serial.println("Running tachometer test sweep...");
            matrixDisplay.testSweep();
            Serial.println("Tachometer test complete");
        }
        else if (command.equalsIgnoreCase("tachoinfo")) {
            matrixDisplay.printRpmThresholds();
        }
        else if (command.equalsIgnoreCase("rpminfo")) {
            float currentRpm = rpmSensor.getRpm();
            Serial.println(matrixDisplay.getRpmRangeInfo(currentRpm));
        }
        // State machine commands
        else if (command.equalsIgnoreCase("state") || command.equalsIgnoreCase("status")) {
            if (gearbox) {
                gearbox->printStateInfo();
            } else {
                Serial.println("Gearbox state machine not initialized");
            }
        }
        // Manual mode commands
        else if (command.equalsIgnoreCase("manual")) {
            manualMode.printStatus();
        }
        else if (command.equalsIgnoreCase("manual on")) {
            manualMode.setManualMode(true);
        }
        else if (command.equalsIgnoreCase("manual off")) {
            manualMode.setManualMode(false);
        }
        // Gear sensor diagnostic command
        else if (command.equalsIgnoreCase("gearsensor") || command.equalsIgnoreCase("gear")) {
            if (gearSensor) {
                gearSensor->printStatus();
            } else {
                Serial.println("Gear sensor not initialized");
            }
        }
        // Shift logger commands
        else if (command.equalsIgnoreCase("dump") || command.equalsIgnoreCase("logs")) {
            if (shiftLogger) {
                shiftLogger->dumpAllLogsToSerial();
            } else {
                Serial.println("Shift logger not initialized");
            }
        }
        else if (command.equalsIgnoreCase("csv")) {
            if (shiftLogger) {
                shiftLogger->exportLogsAsCSV();
            } else {
                Serial.println("Shift logger not initialized");
            }
        }
        else if (command.equalsIgnoreCase("stats")) {
            if (shiftLogger) {
                shiftLogger->printStatistics();
            } else {
                Serial.println("Shift logger not initialized");
            }
        }
        else if (command.equalsIgnoreCase("clear")) {
            if (shiftLogger) {
                Serial.println("Clearing all shift logs...");
                shiftLogger->clearLogs();
                Serial.println("Shift logs cleared!");
            } else {
                Serial.println("Shift logger not initialized");
            }
        }
        else if (command.equalsIgnoreCase("help")) {
            printHelp();
        }
        else {
            Serial.println("Unknown command. Type 'help' for available commands.");
        }
    }
}

void SerialCommands::printHelp() {
    Serial.println("Available commands:");
    Serial.println("=== STATE MACHINE ===");
    Serial.println("  state      - Show current state machine status");
    Serial.println("=== MANUAL MODE ===");
    Serial.println("  manual     - Show manual mode status");
    Serial.println("  manual on  - Force enable manual mode");
    Serial.println("  manual off - Force disable manual mode");
    Serial.println("  NOTE: Normal toggle is both neutral buttons held for 1 second");
    Serial.println("=== SHIFT LOGGER ===");
    Serial.println("  dump       - Show all shift logs in table format");
    Serial.println("  csv        - Export all logs as CSV");
    Serial.println("  stats      - Show shift statistics");
    Serial.println("  clear      - Clear all logged data");
    Serial.println("=== HALL SENSOR CURVES ===");
    Serial.println("  hallinfo   - Show hall sensor curve configuration");
    Serial.println("  halltest   - Test hall sensor response in real-time");
    Serial.println("  curve <type> - Set curve: linear, log, exp, smooth, custom");
    Serial.println("  strength <value> - Set curve strength (0.1-5.0)");
    Serial.println("=== MATRIX TACHOMETER ===");
    Serial.println("  tachotest  - Run tachometer test sweep animation");
    Serial.println("  tachoinfo  - Show RPM thresholds for bar graph");
    Serial.println("  rpminfo    - Show current RPM range information");
    Serial.println("=== GEAR SENSOR ===");
    Serial.println("  gearsensor - Show gear sensor diagnostic information");
    Serial.println("=== GENERAL ===");
    Serial.println("  help       - Show this help");
    Serial.println("NOTE: Hall sensor curves can now be configured via web interface!");
}

// Gear change callback for sensor
void onGearChanged(int newGear, int oldGear) {
    currentGear = newGear;
    gearbox.setCurrentGear(newGear);
    shiftLogger.onGearChanged(newGear, oldGear);
}

// Update pcf8575Connected status
void updateSensorConnectionStatus() {
    pcf8575Connected = gearSensor.isConnected();
}

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("========================================");
    Serial.println("ESP32-S3 T89 Gearbox Controller v" + String(SOFTWARE_VERSION, 1));
    Serial.println("MODULAR ARCHITECTURE - Enhanced Maintainability");
    Serial.println("NEW: Manual Mode Control Added");
    Serial.println("========================================");

    // Initialize LittleFS
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount Failed");
        return;
    }
    
    setupPins();
    
    // Initialize modular components
    Serial.println("Initializing modular components...");

    rpmSensor.begin();


    speedSensor.begin();

    shiftLogger.begin();

    gearbox.begin(&shiftLogger, &speedSensor, &rpmSensor, &clutchServo);

    gearSensor.begin(onGearChanged);
    currentGear = gearSensor.getCurrentGear();
    gearbox.setCurrentGear(currentGear);

    hallSensor.begin(&clutchServo);
    hallSensor.setConfiguration(clutchIdlePos, clutchEngagePos);

    matrixDisplay.begin(&wifiEnabled, &pcf8575Connected, &manualModeActive);

    serialCommands.begin(&hallSensor, &gearbox, &shiftLogger, &gearSensor);

    manualMode.begin(&hallSensor, &clutchServo);
    
    loadConfig();
    
    // Update configurations
    gearbox.setConfiguration(neutralDownMs, neutralUpMs, shiftDownMs, shiftUpMs,
                            clutchIdlePos, clutchEngagePos);
    hallSensor.setConfiguration(clutchIdlePos, clutchEngagePos);
    
    // Initialize WiFi (starts disabled)
    if (wifiEnabled) {
        setupWiFiAP();
    } else {
        colb = 5;
        Serial.println("WiFi Access Point disabled at startup (press WiFi button to enable)");
    }

    // Initialize LED
    pixels.begin();
    pixels.show();
    
    // Initialize clutch servo
    clutchServo.write(clutchIdlePos);
    
    Serial.println("========================================");
    Serial.println("System initialized successfully!");
    Serial.println("State Machine Active: " + gearbox.getStateName());
    Serial.println("Current Gear: " + gearbox.getCurrentGearName());
    Serial.println("Hall Curve: " + hallSensor.getCurveTypeName() + 
                   " (strength: " + String(hallSensor.getCurveStrength(), 2) + ")");
    Serial.println("Gear Sensor: " + String(pcf8575Connected ? "CONNECTED" : "DISCONNECTED"));
    Serial.println("Manual Mode: " + String(manualMode.isManualModeEnabled() ? "ENABLED" : "DISABLED"));
    Serial.println("Type 'help' for available commands");
    Serial.println("Manual Mode Toggle: Hold both neutral buttons for 1 second");
    Serial.println("========================================");
}

void loop() {
    // Update manual mode controller (highest priority)
    manualMode.update();
    
    // Only run automatic systems if NOT in manual mode
    if (!manualMode.isManualModeEnabled()) {
        // Update state machine
        gearbox.update();

        if (gearbox.getCurrentState() == DOWNSHIFT_CLUTCH_ENGAGING) {
            Serial.println("Servo engaging - Voltage: " + String(clutchVoltage, 3) + 
                        "V, Threshold: 1.8V, Pulled: " + String(clutchPulled ? "YES" : "NO"));
        }
        
        // Update modular components
        gearSensor.update();
        updateSensorConnectionStatus(); // Update connection status for matrix display
        hallSensor.updateClutchControl(gearbox.isIdle());
        
        // Update other systems
        shiftLogger.update();
        
        // Update compatibility variables for web interface
        updateCompatibilityVariables();
        
        // Process user input
        serialCommands.processCommands();
        checkWiFiToggleSwitch();
        if (wifiEnabled) { server.handleClient(); }
        processInputs();
        checkServoPosition();
    } else {
        // In manual mode - only update essential systems
        serialCommands.processCommands();
        checkWiFiToggleSwitch();
        if (wifiEnabled) { server.handleClient(); }
        
        // Update sensor connection status for display
        gearSensor.update();
        updateSensorConnectionStatus();
    }
    
    // Always update these systems (independent of mode)
    // Update manual mode status for matrix display
    manualModeActive = manualMode.isManualModeEnabled();
    matrixDisplay.updateWithTachometer(gearbox.getCurrentGearName(), rpmSensor.getRpm());
    speedSensor.update();
    rpmSensor.update();

    // LED heartbeat effect
    unsigned long elapsed = millis() % CYCLE_TIME;
    
    if (elapsed < FLASH_DURATION || 
        (elapsed > FLASH_DURATION + FLASH_GAP && 
         elapsed < FLASH_DURATION * 2 + FLASH_GAP)) {
        pixels.setPixelColor(0, pixels.Color(colr, 0, 0));
    } else {
        pixels.setPixelColor(0, pixels.Color(0, 10, colb));
    }
    
    pixels.show();
    yield();
}

void setupPins() {
    Serial.println("=== PIN ASSIGNMENTS ===");
    Serial.println("Hall Sensor: Pin " + String(PIN_HALL_SENSOR));
    Serial.println("Clutch Servo: Pin " + String(PIN_CLUTCH_SERVO));
    Serial.println("I2C SDA: Pin " + String(I2C_SDA_PIN));
    Serial.println("I2C SCL: Pin " + String(I2C_SCL_PIN));
    Serial.println("========================");

    // Configure input pins
    pinMode(PIN_NEUTRAL_DOWN, INPUT_PULLUP);
    pinMode(PIN_NEUTRAL_UP, INPUT_PULLUP);
    pinMode(PIN_SHIFT_DOWN, INPUT_PULLUP);
    pinMode(PIN_SHIFT_UP, INPUT_PULLUP);
    pinMode(PIN_WIFI_SWITCH, INPUT_PULLUP);

    // Configure analog inputs
    pinMode(PIN_HALL_SENSOR, INPUT);
    pinMode(PIN_CLUTCH_POSITION, INPUT);

    // Configure output pins
    pinMode(PIN_DOWNSHIFT_RELAY, OUTPUT);
    pinMode(PIN_UPSHIFT_RELAY, OUTPUT);
    digitalWrite(PIN_DOWNSHIFT_RELAY, RELAY_OFF);
    digitalWrite(PIN_UPSHIFT_RELAY, RELAY_OFF);

    clutchServo.attach(PIN_CLUTCH_SERVO);

    // Initialize I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    Serial.println("Relay Configuration:");
    Serial.println("  Type: " + String(RELAY_ACTIVE_LOW ? "Active-LOW" : "Active-HIGH"));
    Serial.println("  ON state: " + String(RELAY_ON ? "HIGH" : "LOW"));
    Serial.println("  OFF state: " + String(RELAY_OFF ? "HIGH" : "LOW"));
}

void setupWiFiAP() {
    WiFi.mode(WIFI_AP);
    
    bool apStarted = WiFi.softAP(ssid, password);
    
    if (apStarted) {
        Serial.println("WiFi Access Point started successfully");
        Serial.print("AP SSID: ");
        Serial.println(ssid);
        Serial.print("AP IP address: ");
        Serial.println(WiFi.softAPIP());
        
        colb = 255;
        setupWeb();
    } else {
        Serial.println("Failed to start WiFi Access Point");
        colb = 5;
        wifiEnabled = false;
    }
}

void disableWiFi() {
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    server.stop();
    colb = 5;
    Serial.println("WiFi Access Point disabled");
}

void toggleWiFi() {
    if (wifiEnabled) {
        wifiEnabled = false;
        disableWiFi();
        matrixDisplay.displayShiftNotification('F');
        Serial.println("WiFi Access Point toggled OFF");
    } else {
        wifiEnabled = true;
        setupWiFiAP();
        matrixDisplay.displayShiftNotification('W');
        Serial.println("WiFi Access Point toggled ON");
    }
}

void checkWiFiToggleSwitch() {
    bool currentWifiSwitchState = digitalRead(PIN_WIFI_SWITCH);
    unsigned long currentTime = millis();
    
    if (currentWifiSwitchState == LOW && lastWifiSwitchState == HIGH) {
        if (currentTime - lastWifiButtonPress > WIFI_DEBOUNCE_DELAY) {
            toggleWiFi();
            lastWifiButtonPress = currentTime;
        }
    }
    
    lastWifiSwitchState = currentWifiSwitchState;
}

void checkServoPosition() {
    int analogValue = analogRead(PIN_CLUTCH_POSITION);
    clutchVoltage = (analogValue * 3.3) / 4095.0;
    bool newClutchPulled = (clutchVoltage < CLUTCH_THRESHOLD_VOLTAGE);
    
    // Detect clutch state change and send event
    if (newClutchPulled != clutchPulled) {
        clutchPulled = newClutchPulled;
        gearbox.setClutchPulled(clutchPulled);
        
        // Send the appropriate event to the state machine
        if (clutchPulled) {
            gearbox.processEvent(EVENT_CLUTCH_PULLED);
        } else {
            gearbox.processEvent(EVENT_CLUTCH_RELEASED);
        }
    }
}

void processInputs() {
    bool neutralDownState = digitalRead(PIN_NEUTRAL_DOWN);
    bool neutralUpState = digitalRead(PIN_NEUTRAL_UP);
    bool shiftDownState = digitalRead(PIN_SHIFT_DOWN);
    bool shiftUpState = digitalRead(PIN_SHIFT_UP);
    
    // Send events to state machine on button press
    if (neutralDownState == LOW && lastNeutralDownState == HIGH) {
        gearbox.processEvent(EVENT_NEUTRAL_DOWN_PRESSED);
    }
    
    if (neutralUpState == LOW && lastNeutralUpState == HIGH) {
        gearbox.processEvent(EVENT_NEUTRAL_UP_PRESSED);
    }
    
    if (shiftDownState == LOW && lastShiftDownState == HIGH) {
        gearbox.processEvent(EVENT_SHIFT_DOWN_PRESSED);
    }
    
    if (shiftUpState == LOW && lastShiftUpState == HIGH) {
        gearbox.processEvent(EVENT_SHIFT_UP_PRESSED);
    }
    
    // Update last button states
    lastNeutralDownState = neutralDownState;
    lastNeutralUpState = neutralUpState;
    lastShiftDownState = shiftDownState;
    lastShiftUpState = shiftUpState;
}

void setupWeb() {
    webInterface.setupRoutes();
    server.begin();
    Serial.println("HTTP server started with web gauges");
}

void loadConfig() {
    prefs.begin("gearbox", false);
    
    neutralDownMs = prefs.getInt("neutralDownMs", 40);
    neutralUpMs = prefs.getInt("neutralUpMs", 40);
    shiftDownMs = prefs.getInt("shiftDownMs", 150);
    shiftUpMs = prefs.getInt("shiftUpMs", 150);
    clutchIdlePos = prefs.getInt("clutchIdlePos", 0);
    clutchEngagePos = prefs.getInt("clutchEngagePos", 180);
    prefs.end();

    Serial.println("Configuration loaded:");
    Serial.println("  Neutral Down: " + String(neutralDownMs) + "ms");
    Serial.println("  Neutral Up: " + String(neutralUpMs) + "ms");
    Serial.println("  Shift Down: " + String(shiftDownMs) + "ms");
    Serial.println("  Shift Up: " + String(shiftUpMs) + "ms");
    Serial.println("  Clutch Idle: " + String(clutchIdlePos) + "°");
    Serial.println("  Clutch Engage: " + String(clutchEngagePos) + "°");
}

void saveConfig() {
    prefs.begin("gearbox", false);
    
    prefs.putInt("neutralDownMs", neutralDownMs);
    prefs.putInt("neutralUpMs", neutralUpMs);
    prefs.putInt("shiftDownMs", shiftDownMs);
    prefs.putInt("shiftUpMs", shiftUpMs);
    prefs.putInt("clutchIdlePos", clutchIdlePos);
    prefs.putInt("clutchEngagePos", clutchEngagePos);
    prefs.end();

    gearbox.setConfiguration(neutralDownMs, neutralUpMs, shiftDownMs, shiftUpMs,
                            clutchIdlePos, clutchEngagePos);
    
    Serial.println("Configuration saved to preferences");
}

void updateCompatibilityVariables() {
    // Update compatibility variables for WebInterface.h
    shiftInProgress = gearbox.isShiftInProgress();
    waitingForClutch = gearbox.isWaitingForClutch();
    shiftSequenceState = gearbox.isShifting() ? 1 : 0;
    autoDownshift = gearbox.getCurrentState() == DOWNSHIFT_CLUTCH_ENGAGING || 
                    gearbox.getCurrentState() == DOWNSHIFT_CLUTCH_ENGAGED ||
                    gearbox.getCurrentState() == DOWNSHIFT_SHIFTING;
    
    // Update hall sensor globals for web interface
    hallCurveType = hallSensor.getCurveType();
    hallCurveStrength = hallSensor.getCurveStrength();
}

// end of code