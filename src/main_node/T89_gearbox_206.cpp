// 213    HARDWARE: clutch position now read via ADS1115 (16-bit, differential A0-A1) over
//        I2C SDA8/SCL14 @0x48, 2:1 divider on servo feedback; ~0.003V jitter vs noisy GPIO15.
//        Falls back to legacy GPIO15 analogRead if ADS absent. Hall pin 4 moved to central
//        pin map (PIN_HALL_SENSOR_2) and injected like pin 5.
// 212    NVS config web UI; dual hall calibration (per-paddle idle/pulled capture)
// 211    Hall pin 4 rescaling to match pin 5 range; left/right/result shown on web page
// 210    Stacked downshift, piecewise clutch mapping, dual hall sensor, CAN bus recovery,
//        RPM glitch protection, matrix startup animation, neutral paddle no-clutch,
//        non-blocking CAN TX, rear display 180° rotation
// 207    Removed pcf8575 gear position and relay actuation, replaced with rear node with comms by CANBUS
// 206    Solved multiple timing issues - still work in progress
// 205_1  altered downshift to use servo data instead of delays
// T89_gearbox_205.ino - MODULAR REFACTOR: Separated components into dedicated header files
// 205.0  MAJOR REFACTOR: Modularized code into separate header files for improved maintainability
//        - Moved matrix display functions to MatrixDisplay.h
//        - Moved hall sensor control to HallSensorControl.h
//        - Moved gear sensor functions to GearSensorControl.h
//        - Moved serial commands to  SerialCommands.h
//        - Significantly reduced main file complexity while maintaining all functionality
// 205.1  ADDED: Manual Mode for direct racing control

#include <Arduino.h>
#include <driver/gpio.h>

// Version tracking
#define SOFTWARE_VERSION 213.0

// Pin definitions for ESP32-S3 - FIXED PIN ASSIGNMENTS
#define PIN_MANUAL_TOGGLE   10   // Switch 1 - Long press to toggle manual mode
#define PIN_NEUTRAL         11   // Switch 2 - Neutral (auto direction by gear)
#define PIN_SHIFT_DOWN      12   // Switch 3 - Shift Down
#define PIN_SHIFT_UP        13   // Switch 4 - Shift Up
#define PIN_HALL_SENSOR     5    // Clutch paddle hall (left)
#define PIN_HALL_SENSOR_2   4    // Clutch paddle hall (right)
#define PIN_CLUTCH_SERVO    6    // Servo output
#define PIN_WIFI_SWITCH     21   // WiFi toggle switch input (momentary)
#define PIN_CLUTCH_POSITION 15   // Legacy analog clutch position input (fallback if ADS1115 absent)
#define PIN_ADS_SDA         8    // ADS1115 I2C data
#define PIN_ADS_SCL         14   // ADS1115 I2C clock (not GPIO9 — see board notes)
// 0.591 divider (4.7k top / 6.8k bottom) on servo feedback into ADS A0, return ref on A1
// (differential), 0.1uF across A0-A1. 5V -> ~2.96V, safely under 3.3V VDD at GAIN_ONE.
// The divider exists only to keep the pin under 3.3V; we read the divided voltage as-is and
// do NOT scale it back to the source. Bite-point thresholds are captured live in these same
// (post-divider) volts, so the absolute scale is irrelevant — only relative position matters.

// Clutch servo travel limits (CLUTCH_SERVO_MIN / CLUTCH_SERVO_MAX) now live in
// CalConfig.h, so calValidate() can reject stored angles the servo cannot reach.

// Clutch voltage thresholds — calibrated via web interface
// Downshift trigger. Feedback is inverted: paddle max (clutch DISENGAGED) is the LOW
// end, so the gate fires on the voltage FALLING past this threshold, not rising.
// Set from the Downshift Trigger widget on the home page (/cmd?action=setDisengageV).
float clutchDisengageV   = 1.8f;  // below this = clutch DISENGAGED, safe to send the DS
float clutchJustEngagedV = 1.8f;  // above this = clutch ENGAGED; the bite point

// Standard includes
#include <LittleFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// Project includes - MODULAR COMPONENTS (Order matters for dependencies)
#include "RPM.h"
#include "ShiftLogger.h"
#include "GearboxStateMachine.h"
#include "SimpleServo.h"
#include "HallResponseTypes.h"      // NEW: Shared enum definitions
#include "HallSensorControl.h"      // NEW: Hall sensor control
#include "MatrixDisplay.h"          // NEW: Matrix display functions
#include "WebInterface.h"           // WebInterface needs HallResponseTypes
#include "SerialCommands.h"         // NEW: Serial command processing (MUST be last)
#include "ManualMode.h"             // NEW: Manual mode control
#include "MainCan.h"                // CAN bus interface to rear node

// LED and timing configuration
#define PIN 48
#define NUM_LEDS 1
#define FLASH_DURATION 70
#define FLASH_GAP 150
#define CYCLE_TIME 1000

// RPM input pin
#define PIN_RPM_INPUT      7

// Global state for matrix display (true when CAN gear is valid)
bool pcf8575Connected = false;
bool manualModeActive = false;  // NEW: Manual mode status for matrix

// Create controller instances - MOVED AFTER INCLUDES TO ENSURE COMPLETE TYPES
RPM rpmSensor(PIN_RPM_INPUT, 12.0, 0.3);
ShiftLogger shiftLogger;
GearboxStateMachine gearbox;
MainCan mainCan;

// Create web server and interface
WebServer server(80);
WebInterface webInterface(&server);

// NEW: Modular components - MOVED AFTER INCLUDES
MatrixDisplay matrixDisplay;
HallSensorControl hallSensor(PIN_HALL_SENSOR, PIN_HALL_SENSOR_2);
SerialCommands serialCommands;

// NEW: Manual mode controller
ManualMode manualMode(PIN_MANUAL_TOGGLE, PIN_NEUTRAL,
                     PIN_SHIFT_DOWN, PIN_SHIFT_UP);

// Configuration variables
int neutralDownMs = 40;
int neutralUpMs = 40;
int shiftDownMs = 150;
int shiftUpMs = 150;
// Placeholders until loadConfig() runs. Kept inside the servo's real travel — the old
// 185 was past even the nominal 180, so any use before configuration commanded an angle
// that SimpleServo would silently clamp.
int clutchIdlePos = CLUTCH_SERVO_MIN;
int clutchFullyPull = CLUTCH_SERVO_MAX;

// WiFi state
bool wifiEnabled = false;
#define WIFI_LONGPRESS_MS 1000   // hold the WiFi switch this long (ms) to toggle on/off

// Clutch monitoring
bool clutchDisengaged = false;
bool clutchJustEngaged = false;  // in the bite band: disengageV <= V <= justEngagedV
float clutchVoltage = 0.0;

// ADS1115 — clutch position feedback (16-bit, differential A0–A1 for EMI rejection)
Adafruit_ADS1115 ads;
bool adsReady = false;

// Hall sensor range mirrors (synced from hallSensor each loop)
int hallMin = 780;
int hallMax = 3330;

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
bool lastManualToggleState = HIGH;
bool lastNeutralState = HIGH;
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

// CAN send helpers — called by GearboxStateMachine and ManualMode via extern
void canSendShiftUp(uint16_t shiftMs, uint16_t ignCutMs, uint8_t targetGear) { mainCan.sendShiftUp(shiftMs, ignCutMs, targetGear); }
void canSendShiftDown(uint16_t shiftMs, uint8_t targetGear)                  { mainCan.sendShiftDown(shiftMs, targetGear); }
void canSendShiftStack(uint8_t targetGear)                                   { mainCan.sendShiftStack(targetGear); }

// Legacy functions for WebInterface compatibility
bool isShiftAllowed() { return gearbox.canAcceptShiftCommand(); }
bool canDownshift() { return gearbox.canAcceptShiftCommand() && clutchDisengaged; }
void setShiftInProgress(bool inProgress) { /* Now handled by state machine */ }
void startDownshiftWithClutchCheck(int durationMs) { 
    gearbox.processEvent(EVENT_NEUTRAL_DOWN_PRESSED); 
}

void clutchToMax() { clutchServo.write(clutchFullyPull); }
void clutchToIdle() { clutchServo.write(clutchIdlePos); }
void displayShiftLetter(char letter) { matrixDisplay.displayShiftLetter(letter); }
String getGearStatusForWeb() { return mainCan.getGearName(); }
float getRadiatorTempForWeb() { return mainCan.getRadiatorTemp(); }
uint8_t getPumpDutyForWeb() { return mainCan.getPumpDuty(); }
String getHallCurveTypeName() { return hallSensor.getCurveTypeName(); }
void saveHallCurveConfig() { /* Handled by HallSensorControl */ }

void saveHallRangeConfig() {
    hallSensor.setHallRange(hallMin, hallMax);
}

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
    Serial.println("=== GENERAL ===");
    Serial.println("  help       - Show this help");
    Serial.println("NOTE: Hall sensor curves can now be configured via web interface!");
}


// ---- Dedicated clutch-servo control task ----
// Pinned to core 0 (Arduino loop() runs on core 1) at high priority so the paddle->servo
// path tracks at a steady 200 Hz regardless of loop() load (web server, CAN, matrix).
// It only drives the servo when idle or in manual mode; during shift sequences the state
// machine (gearbox.update() in loop) owns the servo, so this task stands off whenever
// gearbox.isIdle() is false.
static void clutchControlTask(void* param) {
    const TickType_t period = pdMS_TO_TICKS(5);   // 200 Hz — plenty for clutch tracking
    for (;;) {
        if (manualMode.isManualModeEnabled()) {
            manualMode.updateClutch();              // direct paddle->servo (no limits)
        } else if (gearbox.isIdle()) {
            hallSensor.updateClutchControl(true);   // curve paddle->servo (respects servoOverride)
        }
        vTaskDelay(period);
    }
}

void setup() {
    // Emit valid idle PWM the instant we're out of reset, before anything else. The
    // ASME-MR servo needs ~2s to re-acquire its signal after the pulse stream drops
    // (which happens on every MCU reset), so starting it here instead of mid-setup
    // shaves that acquisition window. clutchIdlePos is still default here; loadConfig()
    // + clutchControlTask refine it once NVS is read.
    clutchServo.attach(PIN_CLUTCH_SERVO);
    clutchServo.setLimits(CLUTCH_SERVO_MIN, CLUTCH_SERVO_MAX);
    clutchServo.write(clutchIdlePos);

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

    // Initialize CAN bus
    mainCan.begin();

    // Initialize modular components
    Serial.println("Initializing modular components...");

    rpmSensor.begin();


    shiftLogger.begin();

    gearbox.begin(&shiftLogger, &rpmSensor, &clutchServo);

    hallSensor.begin(&clutchServo);
    hallSensor.setConfiguration(clutchIdlePos, clutchFullyPull);

    matrixDisplay.begin(&wifiEnabled, &pcf8575Connected, &manualModeActive);

    serialCommands.begin(&hallSensor, &gearbox, &shiftLogger);

    manualMode.begin(&hallSensor, &clutchServo);

    loadConfig();

    // Seed the CalConfig NVS blob from live globals (first boot) or load it.
    calInit(neutralDownMs, neutralUpMs, shiftUpMs, shiftDownMs, clutchIdlePos, clutchFullyPull);

    // Update configurations
    gearbox.setConfiguration(neutralDownMs, neutralUpMs, shiftDownMs, shiftUpMs,
                            clutchIdlePos, clutchFullyPull);
    hallSensor.setConfiguration(clutchIdlePos, clutchFullyPull);

    // Dedicated clutch control on core 0: steady high-rate paddle->servo tracking,
    // independent of loop() load (web/CAN/matrix).
    xTaskCreatePinnedToCore(clutchControlTask, "ClutchCtl", 4096, nullptr, 5, nullptr, 0);

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
    Serial.println("CAN Gear: " + mainCan.getGearName());
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
                        "V, Threshold: 1.8V, Pulled: " + String(clutchDisengaged ? "YES" : "NO"));
        }
        
        // Clutch paddle->servo is handled by clutchControlTask (core 0)

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
        
    }
    
    // Poll CAN for incoming messages (ACK, gear pos)
    mainCan.poll();

    // Update state machine gear from CAN (rear node is authoritative for gear position)
    if (mainCan.isGearValid()) {
        int canGear = (int)mainCan.getGear();
        if (canGear != currentGear) {
            int prev = currentGear;
            currentGear = canGear;
            gearbox.setCurrentGear(currentGear);
        }
    }

    // Always update these systems (independent of mode)
    manualModeActive = manualMode.isManualModeEnabled();
    pcf8575Connected = mainCan.isGearValid();
    matrixDisplay.updateWithTachometer(mainCan.getGearName(), rpmSensor.getRpm());
    rpmSensor.update();

    // Broadcast RPM on CAN (~20ms cadence, driven by loop rate)
    static unsigned long lastRpmTx = 0;
    if (millis() - lastRpmTx >= 20) {
        mainCan.sendRpm((uint16_t)rpmSensor.getRpm());
        lastRpmTx = millis();
    }

    // Broadcast shift mode on CAN (1Hz)
    static unsigned long lastShiftStatusTx = 0;
    if (millis() - lastShiftStatusTx >= 1000) {
        mainCan.sendShiftStatus(manualModeActive);
        lastShiftStatusTx = millis();
    }

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
    Serial.println("========================");

    // Configure input pins
    pinMode(PIN_MANUAL_TOGGLE, INPUT_PULLUP);
    pinMode(PIN_NEUTRAL, INPUT_PULLUP);
    pinMode(PIN_SHIFT_DOWN, INPUT_PULLUP);
    pinMode(PIN_SHIFT_UP, INPUT_PULLUP);
    pinMode(PIN_WIFI_SWITCH, INPUT_PULLUP);
    gpio_pulldown_dis((gpio_num_t)PIN_WIFI_SWITCH);
    gpio_pullup_en((gpio_num_t)PIN_WIFI_SWITCH);
    // No interrupt: WiFi switch is polled as a non-blocking long-press in checkWiFiToggleSwitch()

    // Configure analog inputs
    pinMode(PIN_HALL_SENSOR, INPUT);
    pinMode(PIN_CLUTCH_POSITION, INPUT_PULLDOWN);   // legacy fallback input, no longer wired — pin held defined

    // ADS1115 — clutch position feedback, differential A0–A1 (rejects ground bounce)
    Wire.begin(PIN_ADS_SDA, PIN_ADS_SCL);
    adsReady = ads.begin(0x48);
    if (adsReady) {
        ads.setGain(GAIN_ONE);                  // ±4.096V FSR — fits divided 0–2.5V
        ads.setDataRate(RATE_ADS1115_128SPS);
        ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);
        Serial.println("ADS1115: clutch position OK (diff A0-A1 @0x48)");
    } else {
        Serial.println("ADS1115: NOT FOUND — falling back to GPIO15 analogRead");
    }

    // Clutch servo is attached first thing in setup() so its idle PWM starts ASAP.

    Serial.println("CAN TX: GPIO" + String(CAN_TX_PIN) + "  RX: GPIO" + String(CAN_RX_PIN));
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
    // Non-blocking long-press detect: read the pin once per call, time the hold
    // with millis() across loop passes. Switch is active-LOW (INPUT_PULLUP).
    static bool holding = false;
    static unsigned long holdStart = 0;
    static bool toggledThisHold = false;

    bool pressed = (digitalRead(PIN_WIFI_SWITCH) == LOW);

    if (pressed) {
        if (!holding) {
            holding = true;
            holdStart = millis();
            toggledThisHold = false;
        } else if (!toggledThisHold && (millis() - holdStart >= WIFI_LONGPRESS_MS)) {
            toggleWiFi();            // fires once per continuous hold
            toggledThisHold = true;
        }
    } else {
        holding = false;            // released; require a fresh 2s hold next time
    }
}

void checkServoPosition() {
    if (adsReady) {
        int16_t counts = ads.getLastConversionResults();   // differential A0-A1, 16-bit
        if (counts < 0) counts = 0;                         // clamp tiny negative at rest
        // Report the divided voltage at the ADS input directly — no back-scaling. Thresholds
        // are captured live in these same post-divider volts, so absolute scale doesn't matter.
        clutchVoltage = ads.computeVolts(counts);
    } else {
        int analogValue = analogRead(PIN_CLUTCH_POSITION);  // fallback: legacy GPIO15
        clutchVoltage = (analogValue * 3.3) / 4095.0;
    }
    // Feedback is inverted: pulling drives the voltage DOWN. So the axis reads
    //   v <  clutchDisengageV    -> DISENGAGED (safe to send a shift)
    //   v <= clutchJustEngagedV  -> biting zone
    //   v >  clutchJustEngagedV  -> ENGAGED (driving)
    // which means clutchDisengageV < clutchJustEngagedV. The old test compared them
    // the other way round, so the band was unsatisfiable in the correct orientation.
    bool newClutchPulled  = (clutchVoltage < clutchDisengageV);
    clutchJustEngaged = (clutchVoltage >= clutchDisengageV && clutchVoltage <= clutchJustEngagedV);
    
    // Update clutch state - the state machine polls this in its clutch-wait states
    if (newClutchPulled != clutchDisengaged) {
        clutchDisengaged = newClutchPulled;
        gearbox.setClutchDisengaged(clutchDisengaged);
    }
}

void processInputs() {
    bool neutralBtnState = digitalRead(PIN_NEUTRAL);
    bool shiftDownState  = digitalRead(PIN_SHIFT_DOWN);
    bool shiftUpState    = digitalRead(PIN_SHIFT_UP);

    // Neutral button — direction decided by current gear
    if (neutralBtnState == LOW && lastNeutralState == HIGH) {
        if (currentGear == 1) {
            gearbox.processEvent(EVENT_NEUTRAL_UP_PRESSED);
        } else if (currentGear == 2) {
            gearbox.processEvent(EVENT_NEUTRAL_DOWN_PRESSED);
        } else {
            Serial.println("[NEUTRAL] Rejected — not in gear 1 or 2");
            matrixDisplay.displayShiftNotification('N', 255, 0, 0); // red = acknowledged but can't action
        }
    }

    if (shiftDownState == LOW && lastShiftDownState == HIGH) {
        gearbox.processEvent(EVENT_SHIFT_DOWN_PRESSED);
    }

    if (shiftUpState == LOW && lastShiftUpState == HIGH) {
        gearbox.processEvent(EVENT_SHIFT_UP_PRESSED);
    }

    lastNeutralState     = neutralBtnState;
    lastShiftDownState   = shiftDownState;
    lastShiftUpState     = shiftUpState;
}

void setupWeb() {
    webInterface.setupRoutes();
    server.begin();
    Serial.println("HTTP server started with web gauges");
}

void loadConfig() {
    prefs.begin("gearbox", false);
    
    neutralDownMs    = prefs.getInt("neutralDownMs", 40);
    neutralUpMs      = prefs.getInt("neutralUpMs", 40);
    shiftDownMs      = prefs.getInt("shiftDownMs", 150);
    shiftUpMs        = prefs.getInt("shiftUpMs", 150);
    clutchIdlePos    = prefs.getInt("clutchIdlePos",   CLUTCH_SERVO_MIN);
    clutchFullyPull  = prefs.getInt("clutchFullyPull", CLUTCH_SERVO_MAX);
    clutchDisengageV   = prefs.getFloat("clutchDisengV", 1.8f);
    clutchJustEngagedV = prefs.getFloat("clutchBiteV",   1.8f);
    prefs.end();

    Serial.println("Configuration loaded:");
    Serial.println("  Neutral Down: " + String(neutralDownMs) + "ms");
    Serial.println("  Neutral Up: " + String(neutralUpMs) + "ms");
    Serial.println("  Shift Down: " + String(shiftDownMs) + "ms");
    Serial.println("  Shift Up: " + String(shiftUpMs) + "ms");
    Serial.println("  Clutch Idle: " + String(clutchIdlePos) + "°");
    Serial.println("  Clutch Engage: " + String(clutchFullyPull) + "°");
}

void saveConfig() {
    prefs.begin("gearbox", false);
    
    prefs.putInt("neutralDownMs", neutralDownMs);
    prefs.putInt("neutralUpMs", neutralUpMs);
    prefs.putInt("shiftDownMs", shiftDownMs);
    prefs.putInt("shiftUpMs", shiftUpMs);
    prefs.putInt("clutchIdlePos", clutchIdlePos);
    prefs.putInt("clutchFullyPull", clutchFullyPull);
    prefs.putFloat("clutchDisengV", clutchDisengageV);
    prefs.putFloat("clutchBiteV",   clutchJustEngagedV);
    prefs.end();

    gearbox.setConfiguration(neutralDownMs, neutralUpMs, shiftDownMs, shiftUpMs,
                            clutchIdlePos, clutchFullyPull);
    
    Serial.println("Configuration saved to preferences");
}

void updateCompatibilityVariables() {
    // Update compatibility variables for WebInterface.h
    shiftInProgress = gearbox.isShifting();
    waitingForClutch = gearbox.isWaitingForClutch();
    shiftSequenceState = gearbox.isShifting() ? 1 : 0;
    autoDownshift = gearbox.getCurrentState() == DOWNSHIFT_CLUTCH_ENGAGING ||
                    gearbox.getCurrentState() == DOWNSHIFT_SHIFTING;
    
    // Update hall sensor globals for web interface
    hallCurveType = hallSensor.getCurveType();
    hallCurveStrength = hallSensor.getCurveStrength();
    hallMin = hallSensor.getHallMin();
    hallMax = hallSensor.getHallMax();
}

// end of code