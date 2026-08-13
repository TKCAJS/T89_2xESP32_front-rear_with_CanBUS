#ifndef WEB_INTERFACE_H
#define WEB_INTERFACE_H

#include <WebServer.h>
#include <LittleFS.h>
#include "HallResponseTypes.h"
#include "SimpleServo.h"
#include "MainCan.h"
#include "CalConfig.h"

// Define embedded file symbols if using embedded web pages
#ifdef WEBINTERFACE_USE_EMBEDDED
  extern const uint8_t _binary_index_html_start[];
  extern const uint8_t _binary_index_html_end[];
  extern const uint8_t _binary_calibration_html_start[];
  extern const uint8_t _binary_calibration_html_end[];
  extern const uint8_t _binary_nvsconfig_html_start[];
  extern const uint8_t _binary_nvsconfig_html_end[];
  extern const uint8_t _binary_piecewise_html_start[];
  extern const uint8_t _binary_piecewise_html_end[];
#endif

class WebInterface {
private:
    WebServer* server;
    
public:
    WebInterface(WebServer* webServer) : server(webServer) {}

private:
    void serveEmbeddedFile(const uint8_t* dataStart, const uint8_t* dataEnd, const char* filename, const char* mimetype) {
      #ifdef WEBINTERFACE_USE_EMBEDDED
        size_t size = dataEnd - dataStart;
        server->sendHeader("Content-Length", String(size));
        server->send(200, mimetype, "");
        server->client().write(dataStart, size);
        Serial.println(String("Served ") + filename + " from embedded data");
      #else
        server->send(404, "text/plain", String("File not found: ") + filename);
      #endif
    }

    void servePage(const char* filename, const uint8_t* embeddedStart, const uint8_t* embeddedEnd) {
      #ifdef WEBINTERFACE_USE_EMBEDDED
        serveEmbeddedFile(embeddedStart, embeddedEnd, filename, "text/html");
      #else
        if (LittleFS.exists(filename)) {
            File file = LittleFS.open(filename, "r");
            if (file) {
                server->streamFile(file, "text/html");
                file.close();
                Serial.println(String("Served ") + filename + " from LittleFS");
            } else {
                server->send(500, "text/plain", String("Error reading ") + filename);
                Serial.println(String("Error: Could not read ") + filename);
            }
        } else {
            server->send(404, "text/html",
                "<html><body style='background:#1a1a1a;color:white;font-family:Arial;text-align:center;padding:50px;'>"
                "<h1>File Not Found</h1>"
                "<p>" + String(filename) + " not found</p>"
                "<p>Please upload the file to the ESP32 file system</p>"
                "<a href='/' style='color:#4CAF50;'>Return to Main Page</a>"
                "</body></html>");
            Serial.println(String("Error: ") + filename + " not found in LittleFS");
        }
      #endif
    }

public:
    void setupRoutes() {
        server->on("/", HTTP_GET, [this]() { this->handleRoot(); });
        server->on("/index.html", HTTP_GET, [this]() { this->handleRoot(); });
        server->on("/update", HTTP_POST, [this]() { this->handleUpdate(); });
        server->on("/updateHallCurve", HTTP_POST, [this]() { this->handleHallCurveUpdate(); });
        server->on("/cmd", HTTP_GET, [this]() { this->handleCommand(); });
        server->on("/sensorData", HTTP_GET, [this]() { this->handleSensorData(); });
        server->on("/configData", HTTP_GET, [this]() { this->handleConfigData(); });
        server->on("/shiftStats", HTTP_GET, [this]() { this->handleShiftStats(); });
        server->on("/shiftLogs", HTTP_GET, [this]() { this->handleShiftLogs(); });
        server->on("/hello", HTTP_GET, [this]() { this->handleHelloPage(); });
        server->on("/calibration", HTTP_GET, [this]() { this->handleCalibrationPage(); });
        server->on("/piecewise", HTTP_GET, [this]() { this->handlePiecewisePage(); });
        // NVS calibration config API (CalConfig blob, CRC-verified)
        server->on("/api/config",   HTTP_GET,  [this]() { this->handleApiGetConfig(); });
        server->on("/api/config",   HTTP_POST, [this]() { this->handleApiPostConfig(); });
        server->on("/api/defaults", HTTP_POST, [this]() { this->handleApiDefaults(); });
        server->on("/nvsconfig",    HTTP_GET,  [this]() { this->handleNvsConfigPage(); });
    }
    
    void handleRoot() {
      #ifdef WEBINTERFACE_USE_EMBEDDED
        servePage("/index.html", _binary_index_html_start, _binary_index_html_end);
      #else
        servePage("/index.html", nullptr, nullptr);
      #endif
    }
    
    void handleHelloPage() {
        servePage("/hello.html", nullptr, nullptr);
    }

    void handleCalibrationPage() {
      #ifdef WEBINTERFACE_USE_EMBEDDED
        servePage("/calibration.html", _binary_calibration_html_start, _binary_calibration_html_end);
      #else
        servePage("/calibration.html", nullptr, nullptr);
      #endif
    }

    void handlePiecewisePage() {
      #ifdef WEBINTERFACE_USE_EMBEDDED
        servePage("/piecewise.html", _binary_piecewise_html_start, _binary_piecewise_html_end);
      #else
        servePage("/piecewise.html", nullptr, nullptr);
      #endif
    }

    void handleUpdate();
    void handleHallCurveUpdate();
    void handleCommand();
    void handleSensorData();
    void handleConfigData();
    void handleShiftStats();
    void handleShiftLogs();
    // NVS CalConfig API
    void handleApiGetConfig();
    void handleApiPostConfig();
    void handleApiDefaults();
    void handleNvsConfigPage();
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
extern int clutchFullyPull;
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
extern RPM rpmSensor;
extern SimpleServo clutchServo;
extern ShiftLogger shiftLogger;

// Hall sensor curve globals
extern HallResponseCurve hallCurveType;
extern float hallCurveStrength;

// Hall sensor range globals
extern int hallMin;
extern int hallMax;

// Clutch voltage threshold globals
extern float clutchDisengageV;
extern float clutchJustEngagedV;
extern bool clutchJustEngaged;

// Function declarations for callbacks
extern bool isShiftAllowed();
extern bool canDownshift();
extern void setShiftInProgress(bool inProgress);
extern void startDownshiftWithClutchCheck(int durationMs);
extern void canSendShiftUp(uint16_t shiftMs, uint16_t ignCutMs, uint8_t targetGear = 0xFF);
extern void canSendShiftDown(uint16_t shiftMs, uint8_t targetGear = 0xFF);
extern void engageClutch();
extern void displayShiftLetter(char letter);
extern void saveConfig();
extern void loadConfig();
extern String getGearStatusForWeb();
extern float getRadiatorTempForWeb();
extern uint8_t getPumpDutyForWeb();
extern String getHallCurveTypeName();
extern void saveHallCurveConfig();
extern void saveHallRangeConfig();
extern HallSensorControl hallSensor;

// Implementation of web interface methods
void WebInterface::handleUpdate() {
    if (server->hasArg("neutralDownMs") && server->hasArg("neutralUpMs") && 
        server->hasArg("shiftDownMs") && server->hasArg("shiftUpMs") &&
        server->hasArg("clutchIdlePos") && server->hasArg("clutchFullyPull")) {

        int newNeutralDownMs = server->arg("neutralDownMs").toInt();
        int newNeutralUpMs = server->arg("neutralUpMs").toInt();
        int newShiftDownMs = server->arg("shiftDownMs").toInt();
        int newShiftUpMs = server->arg("shiftUpMs").toInt();
        int newClutchIdlePos = server->arg("clutchIdlePos").toInt();
        int newClutchFullyPull = server->arg("clutchFullyPull").toInt();

        if (newNeutralDownMs > 0 && newNeutralDownMs < 5000 &&
            newNeutralUpMs > 0 && newNeutralUpMs < 5000 &&
            newShiftDownMs > 0 && newShiftDownMs < 5000 &&
            newShiftUpMs > 0 && newShiftUpMs < 5000 &&
            newClutchIdlePos >= 0 && newClutchIdlePos <= 180 &&
            newClutchFullyPull >= 0 && newClutchFullyPull <= 180) {

            neutralDownMs = newNeutralDownMs;
            neutralUpMs = newNeutralUpMs;
            shiftDownMs = newShiftDownMs;
            shiftUpMs = newShiftUpMs;
            clutchIdlePos = newClutchIdlePos;
            clutchFullyPull = newClutchFullyPull;
        
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
        
        if (newCurveType >= 0 && newCurveType <= 5 &&
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
        clutchServo.write(clutchFullyPull);
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
    } else if (action == "enableServoOverride") {
        hallSensor.setServoOverride(true);
        server->send(200, "text/plain", "override:on");
        Serial.println("Servo override ON — hall sensor paused");
        return;
    } else if (action == "disableServoOverride") {
        hallSensor.setServoOverride(false);
        server->send(200, "text/plain", "override:off");
        Serial.println("Servo override OFF — hall sensor active");
        return;
    } else if (action == "setServoPos") {
        int pos = constrain(server->arg("pos").toInt(), 0, 180);
        clutchServo.write(pos);
        server->send(200, "text/plain", String(pos));
        return;
    } else if (action == "saveClutchIdle") {
        int pos = constrain(server->arg("pos").toInt(), 0, 180);
        clutchIdlePos = pos;
        saveConfig();
        clutchServo.write(clutchIdlePos);
        server->send(200, "text/plain", "Idle saved: " + String(pos) + "\xC2\xB0");
        Serial.println("Clutch idle calibrated: " + String(pos) + "°");
        return;
    } else if (action == "saveClutchEngage") {
        int pos = constrain(server->arg("pos").toInt(), 0, 180);
        clutchFullyPull = pos;
        saveConfig();
        server->send(200, "text/plain", "Engage saved: " + String(pos) + "\xC2\xB0");
        Serial.println("Clutch engage calibrated: " + String(pos) + "°");
        return;
    } else if (action == "savePiecewiseZone") {
        int hbs = server->arg("hBiteStart").toInt();
        int hbe = server->arg("hBiteEnd").toInt();
        int sbs = server->arg("sBiteStart").toInt();
        int sbe = server->arg("sBiteEnd").toInt();
        int blend = server->arg("blend").toInt();
        if (hbs >= 0 && hbe <= 4095 && hbs < hbe && sbs >= 0 && sbe <= 180 && sbs < sbe &&
            blend >= 0 && blend <= 2000) {
            hallSensor.setPiecewiseZone(hbs, hbe, sbs, sbe, blend);
            server->send(200, "text/plain", "Piecewise zone saved");
            Serial.println("Piecewise zone saved via web");
        } else {
            server->send(400, "text/plain", "Invalid piecewise zone parameters");
        }
        return;
    } else if (action == "setCurveType") {
        String type = server->arg("type");
        hallSensor.setCurveType(type);
        server->send(200, "text/plain", "Curve set: " + hallSensor.getCurveTypeName());
        return;
    } else if (action == "captureHallLIdle") {
        server->send(200, "text/plain", hallSensor.capturePin1Idle());
        return;
    } else if (action == "captureHallLPulled") {
        server->send(200, "text/plain", hallSensor.capturePin1Pulled());
        return;
    } else if (action == "captureHallRIdle") {
        server->send(200, "text/plain", hallSensor.capturePin2Idle());
        return;
    } else if (action == "captureHallRPulled") {
        server->send(200, "text/plain", hallSensor.capturePin2Pulled());
        return;
    } else if (action == "saveHallRange") {
        int min = server->arg("min").toInt();
        int max = server->arg("max").toInt();
        if (min >= 0 && max <= 4095 && min < max) {
            hallMin = min;
            hallMax = max;
            saveHallRangeConfig();
            server->send(200, "text/plain", "Hall range saved: " + String(min) + "-" + String(max));
            Serial.println("Hall range calibrated: " + String(min) + "-" + String(max));
        } else {
            server->send(400, "text/plain", "Invalid hall range");
        }
        return;
    } else if (action == "captureDisengageV") {
        clutchDisengageV = clutchVoltage;
        saveConfig();
        server->send(200, "text/plain", String(clutchDisengageV, 3));
        Serial.println("Clutch disengage threshold set: " + String(clutchDisengageV, 3) + "V");
        return;
    } else if (action == "captureJustEngagedV") {
        clutchJustEngagedV = clutchVoltage;
        saveConfig();
        server->send(200, "text/plain", String(clutchJustEngagedV, 3));
        Serial.println("Clutch just-engaged threshold set: " + String(clutchJustEngagedV, 3) + "V");
        return;
    } else if (action == "setJustEngagedV") {
        float v = server->arg("v").toFloat();
        v = constrain(v, 0.0f, 3.3f);
        clutchJustEngagedV = v;
        saveConfig();
        server->send(200, "text/plain", String(v, 3));
        Serial.println("Clutch just-engaged threshold set manually: " + String(v, 3) + "V");
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
    int hallLeft   = analogRead(PIN_HALL_SENSOR);
    int hallRight  = hallSensor.getPin2Raw();          // raw — for display only
    int hallValue  = max(hallLeft, hallSensor.getPin2Scaled()); // scaled max — drives servo
    json += "\"hallLeft\":"  + String(hallLeft)  + ",";
    json += "\"hallRight\":" + String(hallRight) + ",";
    json += "\"hallValue\":" + String(hallValue) + ",";
    json += "\"clutchVoltage\":" + String(clutchVoltage, 3) + ",";
    json += "\"clutchPulled\":" + String(clutchPulled ? "true" : "false") + ",";
    json += "\"shiftSequenceState\":" + String(shiftSequenceState) + ",";
    json += "\"currentGear\":\"" + getGearStatusForWeb() + "\",";
    json += "\"softwareVersion\":" + String(SOFTWARE_VERSION) + ",";

    // Actual last-commanded servo angle (whatever path wrote it), not a re-estimate
    json += "\"servoPosition\":" + String(clutchServo.getAngle(), 1) + ",";
    json += "\"clutchDisengageV\":" + String(clutchDisengageV, 3) + ",";
    json += "\"clutchJustEngagedV\":" + String(clutchJustEngagedV, 3) + ",";
    json += "\"clutchJustEngaged\":" + String(clutchJustEngaged ? "true" : "false") + ",";
    
    json += "\"currentRpm\":" + String(rpmSensor.getRpm(), 1) + ",";
    json += "\"currentTemp\":" + String(getRadiatorTempForWeb(), 1) + ",";
    json += "\"pumpDuty\":" + String(getPumpDutyForWeb()) + ",";
    json += "\"currentMph\":0,";
    json += "\"shiftTimingActive\":" + String(shiftLogger.isTimingActive() ? "true" : "false") + ",";
    json += "\"hallCurveName\":\"" + getHallCurveTypeName() + "\",";
    json += "\"hallCurveStrength\":" + String(hallCurveStrength, 1) + ",";
    json += "\"servoOverride\":" + String(hallSensor.isServoOverride() ? "true" : "false");
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
    json += "\"clutchFullyPull\":" + String(clutchFullyPull) + ",";
    json += "\"hallCurveType\":" + String((int)hallCurveType) + ",";
    json += "\"hallCurveStrength\":" + String(hallCurveStrength, 2) + ",";
    json += "\"hallMin\":" + String(hallMin) + ",";
    json += "\"hallMax\":" + String(hallMax) + ",";
    json += "\"clutchDisengageV\":" + String(clutchDisengageV, 3) + ",";
    json += "\"clutchJustEngagedV\":" + String(clutchJustEngagedV, 3) + ",";
    json += "\"clutchServoMin\":" + String(CLUTCH_SERVO_MIN) + ",";
    json += "\"clutchServoMax\":" + String(CLUTCH_SERVO_MAX) + ",";
    json += "\"hallBiteStart\":"  + String(hallSensor.getHallBiteStart())  + ",";
    json += "\"hallBiteEnd\":"    + String(hallSensor.getHallBiteEnd())    + ",";
    json += "\"servoBiteStart\":" + String(hallSensor.getServoBiteStart()) + ",";
    json += "\"servoBiteEnd\":"   + String(hallSensor.getServoBiteEnd())   + ",";
    json += "\"pwBlend\":"        + String(hallSensor.getPwBlend())        + ",";
    json += "\"pin2RawMin\":"     + String(hallSensor.getPin2RawMin())     + ",";
    json += "\"pin2RawMax\":"     + String(hallSensor.getPin2RawMax());
    json += "}";
    
    server->send(200, "application/json", json);
}

void WebInterface::handleShiftStats() {
    server->send(200, "text/plain", shiftLogger.getStatistics());
}

void WebInterface::handleShiftLogs() {
    server->send(200, "application/json", shiftLogger.getRecentLogs(20));
}

// ---------------------------------------------------------------------------
//  NVS CalConfig API handlers
// ---------------------------------------------------------------------------

// Helper: build and send a CalConfig as JSON, optionally with an HTTP code.
static void sendCalJson(WebServer *server, const CalConfig &c, int code = 200) {
    JsonDocument doc;
    calToJson(c, doc);
    String out;
    serializeJson(doc, out);
    server->send(code, "application/json", out);
}

// GET /api/config — return the current CalConfig as JSON.
void WebInterface::handleApiGetConfig() {
    xSemaphoreTake(gCalMutex, portMAX_DELAY);
    CalConfig snap = gCalConfig;
    xSemaphoreGive(gCalMutex);
    sendCalJson(server, snap);
}

// POST /api/config — parse JSON body, validate, commit, read back, echo.
// On success also syncs live globals and calls saveConfig() so the existing
// "gearbox" NVS namespace stays in step.
void WebInterface::handleApiPostConfig() {
    if (!server->hasArg("plain")) {
        server->send(400, "application/json", "{\"error\":\"empty body\"}");
        return;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, server->arg("plain"));
    if (err) {
        server->send(400, "application/json",
                     String("{\"error\":\"bad json: ") + err.c_str() + "\"}");
        return;
    }

    // Work on a copy so the live config is never partially updated.
    CalConfig candidate;
    xSemaphoreTake(gCalMutex, portMAX_DELAY);
    candidate = gCalConfig;
    xSemaphoreGive(gCalMutex);

    calJsonOverlay(doc, candidate);

    String why = calValidate(candidate);
    if (why.length()) {
        server->send(400, "application/json", String("{\"error\":\"") + why + "\"}");
        return;
    }

    calSeal(candidate);
    if (!calNvsWrite(candidate)) {
        server->send(500, "application/json", "{\"error\":\"nvs write failed\"}");
        return;
    }

    // Read back so the response reflects what actually landed on flash.
    CalConfig stored;
    if (!calNvsRead(stored)) {
        server->send(500, "application/json", "{\"error\":\"nvs readback failed\"}");
        return;
    }

    // Promote to live globals and persist in the existing "gearbox" namespace too.
    neutralDownMs   = stored.neutralDownMs;
    neutralUpMs     = stored.neutralUpMs;
    shiftUpMs       = stored.shiftUpMs;
    shiftDownMs     = stored.shiftDownMs;
    clutchIdlePos   = stored.clutchIdlePos;
    clutchFullyPull = stored.clutchFullyPull;
    saveConfig();
    if (!shiftInProgress && shiftSequenceState == 0) {
        clutchServo.write(clutchIdlePos);
    }

    xSemaphoreTake(gCalMutex, portMAX_DELAY);
    gCalConfig = stored;
    xSemaphoreGive(gCalMutex);

    sendCalJson(server, stored);
    Serial.println("CalConfig: saved via /api/config");
}

// POST /api/defaults — restore factory defaults, commit, echo.
void WebInterface::handleApiDefaults() {
    CalConfig def;
    calLoadDefaults(def);
    calSeal(def);
    if (!calNvsWrite(def)) {
        server->send(500, "application/json", "{\"error\":\"nvs write failed\"}");
        return;
    }

    CalConfig stored;
    if (!calNvsRead(stored)) {
        server->send(500, "application/json", "{\"error\":\"nvs readback failed\"}");
        return;
    }

    neutralDownMs   = stored.neutralDownMs;
    neutralUpMs     = stored.neutralUpMs;
    shiftUpMs       = stored.shiftUpMs;
    shiftDownMs     = stored.shiftDownMs;
    clutchIdlePos   = stored.clutchIdlePos;
    clutchFullyPull = stored.clutchFullyPull;
    saveConfig();
    if (!shiftInProgress && shiftSequenceState == 0) {
        clutchServo.write(clutchIdlePos);
    }

    xSemaphoreTake(gCalMutex, portMAX_DELAY);
    gCalConfig = stored;
    xSemaphoreGive(gCalMutex);

    sendCalJson(server, stored);
    Serial.println("CalConfig: defaults restored via /api/defaults");
}

// GET /nvsconfig — serve nvsconfig.html from LittleFS.
// Upload instruction: PlatformIO → "Upload Filesystem Image" (env:main_node)
void WebInterface::handleNvsConfigPage() {
  #ifdef WEBINTERFACE_USE_EMBEDDED
    servePage("/nvsconfig.html", _binary_nvsconfig_html_start, _binary_nvsconfig_html_end);
  #else
    if (LittleFS.exists("/nvsconfig.html")) {
        File file = LittleFS.open("/nvsconfig.html", "r");
        if (file) {
            server->streamFile(file, "text/html");
            file.close();
            return;
        }
    }
    server->send(404, "text/plain",
        "nvsconfig.html not found in LittleFS.\n"
        "Run PlatformIO 'Upload Filesystem Image' (env: main_node) to upload the data/ folder.");
  #endif
}

#endif

// end of code