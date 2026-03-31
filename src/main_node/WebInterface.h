#ifndef WEB_INTERFACE_H
#define WEB_INTERFACE_H

#include <WebServer.h>
#include <LittleFS.h>
#include "HallResponseTypes.h"
#include "SimpleServo.h"
#include "MainCan.h"

class WebInterface {
private:
    WebServer* server;
    
public:
    WebInterface(WebServer* webServer) : server(webServer) {}
    
    void setupRoutes() {
        server->on("/", HTTP_GET, [this]() { this->handleRoot(); });
        server->on("/update", HTTP_POST, [this]() { this->handleUpdate(); });
        server->on("/updateHallCurve", HTTP_POST, [this]() { this->handleHallCurveUpdate(); });
        server->on("/cmd", HTTP_GET, [this]() { this->handleCommand(); });
        server->on("/sensorData", HTTP_GET, [this]() { this->handleSensorData(); });
        server->on("/configData", HTTP_GET, [this]() { this->handleConfigData(); });
        server->on("/shiftStats", HTTP_GET, [this]() { this->handleShiftStats(); });
        server->on("/shiftLogs", HTTP_GET, [this]() { this->handleShiftLogs(); });
        server->on("/hello", HTTP_GET, [this]() { this->handleHelloPage(); });
        server->on("/calibration", HTTP_GET, [this]() { this->handleCalibrationPage(); });
    }
    
    void handleRoot() {
        if (LittleFS.exists("/index.html")) {
            File file = LittleFS.open("/index.html", "r");
            if (file) {
                String content = file.readString();
                file.close();
                server->send(200, "text/html", content);
                Serial.println("Served index.html from LittleFS");
            } else {
                server->send(500, "text/plain", "Error reading index.html file");
                Serial.println("Error: Could not read index.html file");
            }
        } else {
            server->send(404, "text/html", 
                "<html><body style='background:#1a1a1a;color:white;font-family:Arial;text-align:center;padding:50px;'>"
                "<h1>File Not Found</h1>"
                "<p>index.html not found in LittleFS</p>"
                "<p>Please upload the file to the ESP32 file system</p>"
                "<a href='/hello' style='color:#4CAF50;'>Hello Page</a> | "
                "<a href='/calibration' style='color:#4CAF50;'>Calibration Page</a>"
                "</body></html>");
            Serial.println("Error: index.html not found in LittleFS");
        }
    }
    
    void handleHelloPage() {
        if (LittleFS.exists("/hello.html")) {
            File file = LittleFS.open("/hello.html", "r");
            if (file) {
                String content = file.readString();
                file.close();
                server->send(200, "text/html", content);
                Serial.println("Served hello.html from LittleFS");
            } else {
                server->send(500, "text/plain", "Error reading hello.html file");
                Serial.println("Error: Could not read hello.html file");
            }
        } else {
            server->send(404, "text/html", 
                "<html><body style='background:#1a1a1a;color:white;font-family:Arial;text-align:center;padding:50px;'>"
                "<h1>File Not Found</h1>"
                "<p>hello.html not found in LittleFS</p>"
                "<p>Please upload the file to the ESP32 file system</p>"
                "<a href='/' style='color:#4CAF50;'>Return to Main Page</a>"
                "</body></html>");
            Serial.println("Error: hello.html not found in LittleFS");
        }
    }

    void handleCalibrationPage() {
        if (LittleFS.exists("/calibration.html")) {
            File file = LittleFS.open("/calibration.html", "r");
            if (file) {
                String content = file.readString();
                file.close();
                server->send(200, "text/html", content);
                Serial.println("Served calibration.html from LittleFS");
            } else {
                server->send(500, "text/plain", "Error reading calibration.html file");
                Serial.println("Error: Could not read calibration.html file");
            }
        } else {
            server->send(404, "text/html", 
                "<html><body style='background:#1a1a1a;color:white;font-family:Arial;text-align:center;padding:50px;'>"
                "<h1>File Not Found</h1>"
                "<p>calibration.html not found in LittleFS</p>"
                "<p>Please upload the file to the ESP32 file system</p>"
                "<a href='/' style='color:#4CAF50;'>Return to Main Page</a>"
                "</body></html>");
            Serial.println("Error: calibration.html not found in LittleFS");
        }
    }
    
    void handleUpdate();
    void handleHallCurveUpdate();
    void handleCommand();
    void handleSensorData();
    void handleConfigData();
    void handleShiftStats();
    void handleShiftLogs();
};

// Forward declarations for structures
struct RelayState {
  bool active = false;
  unsigned long startTime = 0;
  int pin = -1;
  int duration = 0;
};

// Global variables needed for the web interface callbacks
extern int neutralDownMs;
extern int neutralUpMs;
extern int shiftDownMs;
extern int shiftUpMs;
extern int clutchIdlePos;
extern int clutchEngagePos;
extern bool shiftInProgress;
extern bool waitingForClutch;
extern bool wifiEnabled;
extern float clutchVoltage;
extern bool clutchPulled;
extern int currentGear;
extern String gearNames[];
extern int shiftSequenceState;
extern RelayState downshiftRelay;
extern RelayState upshiftRelay;
extern bool autoDownshift;
extern unsigned long clutchStartTime;
extern Speed speedSensor;
extern RPM rpmSensor;
extern SimpleServo clutchServo;
extern ShiftLogger shiftLogger;

// Hall sensor curve globals
extern HallResponseCurve hallCurveType;
extern float hallCurveStrength;

// Function declarations for callbacks
extern bool isShiftAllowed();
extern bool canDownshift();
extern void setShiftInProgress(bool inProgress);
extern void startDownshiftWithClutchCheck(int durationMs);
extern void canSendShiftUp(uint16_t shiftMs, uint16_t ignCutMs);
extern void canSendShiftDown(uint16_t shiftMs);
extern void engageClutch();
extern void displayShiftLetter(char letter);
extern void saveConfig();
extern void loadConfig();
extern String getGearStatusForWeb();
extern String getHallCurveTypeName();
extern void saveHallCurveConfig();

// Implementation of web interface methods
void WebInterface::handleUpdate() {
    if (server->hasArg("neutralDownMs") && server->hasArg("neutralUpMs") && 
        server->hasArg("shiftDownMs") && server->hasArg("shiftUpMs") &&
        server->hasArg("clutchIdlePos") && server->hasArg("clutchEngagePos")) {

        int newNeutralDownMs = server->arg("neutralDownMs").toInt();
        int newNeutralUpMs = server->arg("neutralUpMs").toInt();
        int newShiftDownMs = server->arg("shiftDownMs").toInt();
        int newShiftUpMs = server->arg("shiftUpMs").toInt();
        int newClutchIdlePos = server->arg("clutchIdlePos").toInt();
        int newClutchEngagePos = server->arg("clutchEngagePos").toInt();

        if (newNeutralDownMs > 0 && newNeutralDownMs < 5000 &&
            newNeutralUpMs > 0 && newNeutralUpMs < 5000 &&
            newShiftDownMs > 0 && newShiftDownMs < 5000 &&
            newShiftUpMs > 0 && newShiftUpMs < 5000 &&
            newClutchIdlePos >= 0 && newClutchIdlePos <= 180 &&
            newClutchEngagePos >= 0 && newClutchEngagePos <= 180) {

            neutralDownMs = newNeutralDownMs;
            neutralUpMs = newNeutralUpMs;
            shiftDownMs = newShiftDownMs;
            shiftUpMs = newShiftUpMs;
            clutchIdlePos = newClutchIdlePos;
            clutchEngagePos = newClutchEngagePos;
        
            saveConfig();
        
            if (!shiftInProgress && shiftSequenceState == 0) {
                clutchServo.write(clutchIdlePos);
            }
        
            server->send(200, "text/plain", "Configuration updated successfully");
            Serial.println("Configuration updated via web interface");
        } else {
            server->send(400, "text/plain", "Invalid parameter values");
            Serial.println("Invalid configuration parameters received");
        }
    } else {
        server->send(400, "text/plain", "Missing required parameters");
        Serial.println("Missing parameters in configuration update");
    }
}

void WebInterface::handleHallCurveUpdate() {
    if (server->hasArg("hallCurveType") && server->hasArg("hallCurveStrength")) {
        int newCurveType = server->arg("hallCurveType").toInt();
        float newCurveStrength = server->arg("hallCurveStrength").toFloat();
        
        if (newCurveType >= 0 && newCurveType <= 4 &&
            newCurveStrength >= 0.1 && newCurveStrength <= 5.0) {
            
            hallCurveType = (HallResponseCurve)newCurveType;
            hallCurveStrength = newCurveStrength;
            
            saveHallCurveConfig();
            
            server->send(200, "text/plain", "Hall curve configuration updated successfully");
            Serial.println("Hall curve updated via web interface: " + getHallCurveTypeName() + 
                          " (strength: " + String(hallCurveStrength, 2) + ")");
        } else {
            server->send(400, "text/plain", "Invalid hall curve parameters");
            Serial.println("Invalid hall curve parameters received");
        }
    } else {
        server->send(400, "text/plain", "Missing required hall curve parameters");
        Serial.println("Missing parameters in hall curve update");
    }
}

void WebInterface::handleCommand() {
    String action = server->arg("action");
    
    // Handle calibration-specific commands
    if (action == "clutchIdle") {
        clutchServo.write(clutchIdlePos);
        server->send(200, "text/plain", "Clutch moved to idle position");
        return;
    } else if (action == "clutchEngage") {
        clutchServo.write(clutchEngagePos);
        server->send(200, "text/plain", "Clutch moved to engage position");
        return;
    } else if (action == "testDownshift") {
        if (!isShiftAllowed()) {
            server->send(423, "text/plain", "BLOCKED: Shift in progress");
            return;
        }
        canSendShiftDown(100);
        server->send(200, "text/plain", "Downshift CAN command sent (100ms)");
        return;
    } else if (action == "testUpshift") {
        if (!isShiftAllowed()) {
            server->send(423, "text/plain", "BLOCKED: Shift in progress");
            return;
        }
        canSendShiftUp(100, 0);
        server->send(200, "text/plain", "Upshift CAN command sent (100ms)");
        return;
    } else if (action == "testIgnitionCut") {
        // Test ignition cut relay - use the shift logger's method for proper timing
        shiftLogger.startIgnitionCut();
        server->send(200, "text/plain", "Ignition cut relay activated (50ms pulse)");
        return;
    }
    
    // Regular shift commands (existing code)
    if (!isShiftAllowed()) {
        server->send(423, "text/plain", "BLOCKED: Shift in progress");
        return;
    }
    
    if (action == "neutralDown") {
        if (!canDownshift()) {
            server->send(423, "text/plain", "BLOCKED: Clutch not pulled");
            return;
        }
        setShiftInProgress(true);
        shiftLogger.startShiftTiming(currentGear, 0, rpmSensor.getRpm(), 2);
        startDownshiftWithClutchCheck(neutralDownMs);
        displayShiftLetter('D');
    } else if (action == "neutralUp") {
        setShiftInProgress(true);
        shiftLogger.startShiftTiming(currentGear, 0, rpmSensor.getRpm(), 2);
        canSendShiftUp(neutralUpMs, 0);
        displayShiftLetter('U');
    } else if (action == "shiftDown") {
        setShiftInProgress(true);
        shiftLogger.startShiftTiming(currentGear, currentGear - 1, rpmSensor.getRpm(), 1);
        autoDownshift = true;
        engageClutch();
        clutchStartTime = millis();
        shiftSequenceState = 1;
        displayShiftLetter('D');
    } else if (action == "shiftUp") {
        setShiftInProgress(true);
        shiftLogger.startIgnitionCut();
        shiftLogger.startShiftTiming(currentGear, currentGear + 1, rpmSensor.getRpm(), 0);
        canSendShiftUp(shiftUpMs, IGN_CUT_DEFAULT_MS);
        displayShiftLetter('U');
    }
    
    server->send(200, "text/plain", "OK");
}

void WebInterface::handleSensorData() {
    String json = "{";
    json += "\"shiftInProgress\":" + String(shiftInProgress ? "true" : "false") + ",";
    json += "\"waitingForClutch\":" + String(waitingForClutch ? "true" : "false") + ",";
    json += "\"wifiEnabled\":" + String(wifiEnabled ? "true" : "false") + ",";
    json += "\"apIP\":\"" + WiFi.softAPIP().toString() + "\",";
    json += "\"hallValue\":" + String(analogRead(PIN_HALL_SENSOR)) + ",";
    json += "\"clutchVoltage\":" + String(3.3 - clutchVoltage, 2) + ",";
    json += "\"clutchPulled\":" + String(clutchPulled ? "true" : "false") + ",";
    json += "\"shiftSequenceState\":" + String(shiftSequenceState) + ",";
    json += "\"currentGear\":\"" + getGearStatusForWeb() + "\",";
    json += "\"softwareVersion\":" + String(SOFTWARE_VERSION) + ",";
    
    int hallValue = analogRead(PIN_HALL_SENSOR);
    int servoPosition = map(hallValue, 780, 4000, clutchIdlePos, clutchEngagePos);
    servoPosition = constrain(servoPosition, 0, 180);
    json += "\"servoPosition\":" + String(servoPosition) + ",";
    
    json += "\"currentRpm\":" + String(rpmSensor.getRpm(), 1) + ",";
    json += "\"currentMph\":" + String(speedSensor.getMph(), 1) + ",";
    json += "\"shiftTimingActive\":" + String(shiftLogger.isTimingActive() ? "true" : "false") + ",";
    json += "\"hallCurveName\":\"" + getHallCurveTypeName() + "\",";
    json += "\"hallCurveStrength\":" + String(hallCurveStrength, 1);
    json += "}";
    
    server->send(200, "application/json", json);
}

void WebInterface::handleConfigData() {
    String json = "{";
    json += "\"neutralDownMs\":" + String(neutralDownMs) + ",";
    json += "\"neutralUpMs\":" + String(neutralUpMs) + ",";
    json += "\"shiftDownMs\":" + String(shiftDownMs) + ",";
    json += "\"shiftUpMs\":" + String(shiftUpMs) + ",";
    json += "\"clutchIdlePos\":" + String(clutchIdlePos) + ",";
    json += "\"clutchEngagePos\":" + String(clutchEngagePos) + ",";
    json += "\"hallCurveType\":" + String((int)hallCurveType) + ",";
    json += "\"hallCurveStrength\":" + String(hallCurveStrength, 2);
    json += "}";
    
    server->send(200, "application/json", json);
}

void WebInterface::handleShiftStats() {
    server->send(200, "text/plain", shiftLogger.getStatistics());
}

void WebInterface::handleShiftLogs() {
    server->send(200, "application/json", shiftLogger.getRecentLogs(20));
}

#endif

// end of code