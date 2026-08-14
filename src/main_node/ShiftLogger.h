#ifndef SHIFT_LOGGER_H
#define SHIFT_LOGGER_H

#include <Arduino.h>
#include <Preferences.h>
#include <LittleFS.h>

// Pin definition for ignition cut
#define PIN_IGNITION_CUT 18

// Shift logging configuration.
//
// Logs live on LittleFS, NOT in NVS. They used to be 100 discrete blob keys in a
// "shiftlogs" NVS namespace, competing with the calibration blob for the same
// 630-entry table in a 20 KB partition - enough shifts and config saves started
// failing with "nvs write failed". A filesystem is the right home for append-only
// records: the spiffs partition is 3.4 MB, 173x the NVS partition, already mounted.
//
// Fixed-size ring file: header plus MAX_SHIFT_LOG_ENTRIES slots, oldest overwritten.
#define MAX_SHIFT_LOG_ENTRIES 5000
#define SHIFT_LOG_PATH   "/shiftlog.bin"
#define SHIFT_LOG_MAGIC  0x54383953UL
#define SHIFT_LOG_VER    1
// Recent entries mirrored in RAM. The web server runs inside the same loop() as the
// shift state machine, so serving /shiftLogs from flash would stall shift timing on
// every browser poll. 20 entries costs a few hundred bytes.
#define SHIFT_LOG_MIRROR 20
#define SHIFT_TIMEOUT_MS 300  // Max time to wait for gear change

// Shift log entry structure
struct ShiftLogEntry {
    uint32_t timestamp;        // millis() when shift started
    uint8_t fromGear;         // Starting gear (0-6)
    uint8_t toGear;           // Ending gear (0-6)  
    uint16_t rpm;             // RPM at shift start
    uint16_t shiftTimeMs;     // Time taken for shift in milliseconds
    uint8_t shiftType;        // 0=upshift, 1=downshift, 2=neutral
    bool successful;          // Whether shift completed successfully
};

// Ring file header. Statistics live here too, so logging never touches NVS.
struct ShiftLogHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t head;
    uint32_t count;
    uint32_t totalShifts;
    uint32_t failedShifts;
    uint16_t avgTime;
};

// Ignition cut relay state
struct IgnitionCutState {
    bool active = false;
    unsigned long startTime = 0;
    int duration = 50; // 50ms default duration
};

class ShiftLogger {
private:
    // Ignition cut management
    IgnitionCutState ignitionCut;
    
    // Shift timing tracking
    bool shiftTimingActive;
    unsigned long shiftStartTime;
    uint8_t expectedFromGear;
    uint8_t expectedToGear;
    uint16_t shiftStartRpm;
    uint8_t currentShiftType;
    
    // Data logging
    // No Preferences member: logging is on LittleFS now. NVS is opened in exactly one
    // place, migrateFromNvs(), to drop the legacy logs that were filling the partition.
    int currentLogIndex;
    int totalLogEntries;
    ShiftLogEntry mirror[SHIFT_LOG_MIRROR];
    int mirrorCount;
    
    // Statistics
    uint32_t totalShifts;
    uint32_t failedShifts;
    uint16_t averageShiftTime;

public:
    ShiftLogger() : shiftTimingActive(false), shiftStartTime(0), 
                    expectedFromGear(0), expectedToGear(0), shiftStartRpm(0),
                    currentShiftType(0), currentLogIndex(0), totalLogEntries(0), mirrorCount(0),
                    totalShifts(0), failedShifts(0), averageShiftTime(0) {}
    
    void begin() {
        // Initialize ignition cut pin
        pinMode(PIN_IGNITION_CUT, OUTPUT);
        digitalWrite(PIN_IGNITION_CUT, LOW); // Start with ignition cut off
        
        // Load logging data from preferences
        loadLogIndex();
        
        Serial.println("Shift Logger initialized");
        Serial.println("Ignition Cut Pin: " + String(PIN_IGNITION_CUT));
        Serial.println("Log entries in memory: " + String(totalLogEntries));
        
        printStatistics();
    }
    
    // Call this regularly in main loop for non-blocking operation
    void update() {
        // Update ignition cut relay
        updateIgnitionCut();
        
        // Check for shift timeout
        checkShiftTimeout();
    }
    
    // Start ignition cut (50ms duration)
    void startIgnitionCut() {
        ignitionCut.active = true;
        ignitionCut.startTime = millis();
        digitalWrite(PIN_IGNITION_CUT, HIGH);
        Serial.println("Ignition cut activated for " + String(ignitionCut.duration) + "ms");
    }
    
    // Start shift timing measurement
    void startShiftTiming(uint8_t fromGear, uint8_t toGear, uint16_t rpm, uint8_t shiftType) {
        if (shiftTimingActive) {
            // Log previous shift as failed if still active
            logFailedShift();
        }
        
        shiftTimingActive = true;
        shiftStartTime = millis();
        expectedFromGear = fromGear;
        expectedToGear = toGear;
        shiftStartRpm = rpm;
        currentShiftType = shiftType;
        
        Serial.print("Shift timing started: ");
        Serial.print(getGearName(fromGear));
        Serial.print(" -> ");
        Serial.print(getGearName(toGear));
        Serial.print(" @ ");
        Serial.print(rpm);
        Serial.println(" RPM");
    }
    
    // Called when gear change is detected
    void onGearChanged(uint8_t newGear, uint8_t oldGear) {
        if (!shiftTimingActive) return;
        
        // Check if this is the expected gear change
        if (oldGear == expectedFromGear && newGear == expectedToGear) {
            // Successful shift completion
            unsigned long shiftTime = millis() - shiftStartTime;
            logSuccessfulShift(shiftTime);
            shiftTimingActive = false;
            
            Serial.print("Shift completed successfully in ");
            Serial.print(shiftTime);
            Serial.println("ms");
        }
    }
    
    // Get shift statistics
    String getStatistics() {
        String stats = "Shift Statistics:\n";
        stats += "Total Shifts: " + String(totalShifts) + "\n";
        stats += "Failed Shifts: " + String(failedShifts) + "\n";
        uint32_t successRate = 0;
        if (totalShifts > 0) {
            successRate = ((totalShifts - failedShifts) * 100) / totalShifts;
        }
        stats += "Success Rate: " + String(successRate) + "%\n";
        stats += "Average Shift Time: " + String(averageShiftTime) + "ms\n";
        stats += "Log Entries: " + String(totalLogEntries) + "/" + String(MAX_SHIFT_LOG_ENTRIES);
        return stats;
    }
    
    // Get recent shift logs (for web interface)
    // Served from the RAM mirror, never from flash. The web server shares loop() with
    // the shift state machine, so a file read here would stall shift timing every time
    // a browser polls. Beyond SHIFT_LOG_MIRROR entries, use the CSV export instead.
    String getRecentLogs(int count = 10) {
        String logs = "[";
        int n = min(count, mirrorCount);
        for (int i = mirrorCount - n; i < mirrorCount; i++) {
            if (i > mirrorCount - n) logs += ",";
            logs += logEntryToJson(mirror[i]);
        }
        logs += "]";
        return logs;
    }
    
    // Clear all shift logs
    void clearLogs() {
        initRing();        // rewrites the file with an empty header and blank slots
        migrateFromNvs();  // and drops any legacy NVS logs still holding config space
        mirrorCount = 0;
        Serial.println("Shift logs cleared");
    }
    
    // Print statistics to serial
    void printStatistics() {
        Serial.println("=== SHIFT LOGGER STATISTICS ===");
        Serial.print(getStatistics());
        Serial.println("===============================");
    }
    
    // Check if shift timing is currently active
    bool isTimingActive() {
        return shiftTimingActive;
    }
    
    // Print all logs to serial (for debugging/analysis)
    void dumpAllLogsToSerial() {
        Serial.println("=== COMPLETE SHIFT LOG DUMP ===");
        Serial.println("Entry | Timestamp | From | To | RPM | Time(ms) | Type | Success");
        Serial.println("------|-----------|------|----|----- |----------|------|--------");
        
        int entriesShown = 0;
        for (int i = 0; i < totalLogEntries; i++) {
            ShiftLogEntry entry;
            int index = (currentLogIndex - totalLogEntries + i + MAX_SHIFT_LOG_ENTRIES) % MAX_SHIFT_LOG_ENTRIES;
            
            if (loadLogEntry(index, entry)) {
                Serial.printf("%5d | %9lu | %4s | %2s | %4d | %8d | %8s | %s\n",
                    entriesShown + 1,
                    entry.timestamp,
                    getGearName(entry.fromGear).c_str(),
                    getGearName(entry.toGear).c_str(),
                    entry.rpm,
                    entry.shiftTimeMs,
                    getShiftTypeName(entry.shiftType).c_str(),
                    entry.successful ? "YES" : "NO"
                );
                entriesShown++;
            }
        }
        
        Serial.println("====================================");
        Serial.println("Total entries: " + String(entriesShown));
        printStatistics();
    }
    
    // Export logs as CSV format to serial
    void exportLogsAsCSV() {
        Serial.println("=== CSV EXPORT ===");
        Serial.println("Timestamp,FromGear,ToGear,RPM,ShiftTimeMs,ShiftType,Successful");
        
        for (int i = 0; i < totalLogEntries; i++) {
            ShiftLogEntry entry;
            int index = (currentLogIndex - totalLogEntries + i + MAX_SHIFT_LOG_ENTRIES) % MAX_SHIFT_LOG_ENTRIES;
            
            if (loadLogEntry(index, entry)) {
                Serial.print(entry.timestamp);
                Serial.print(",");
                Serial.print(getGearName(entry.fromGear));
                Serial.print(",");
                Serial.print(getGearName(entry.toGear));
                Serial.print(",");
                Serial.print(entry.rpm);
                Serial.print(",");
                Serial.print(entry.shiftTimeMs);
                Serial.print(",");
                Serial.print(getShiftTypeName(entry.shiftType));
                Serial.print(",");
                Serial.println(entry.successful ? "1" : "0");
            }
        }
        Serial.println("=== END CSV ===");
    }

private:
    void updateIgnitionCut() {
        if (ignitionCut.active) {
            unsigned long elapsed = millis() - ignitionCut.startTime;
            if (elapsed >= ignitionCut.duration) {
                ignitionCut.active = false;
                digitalWrite(PIN_IGNITION_CUT, LOW);
                Serial.println("Ignition cut deactivated");
            }
        }
    }
    
    void checkShiftTimeout() {
        if (shiftTimingActive) {
            unsigned long elapsed = millis() - shiftStartTime;
            if (elapsed >= SHIFT_TIMEOUT_MS) {
                Serial.println("Shift timeout - logging as failed");
                logFailedShift();
                shiftTimingActive = false;
            }
        }
    }
    
    void logSuccessfulShift(unsigned long shiftTime) {
        ShiftLogEntry entry;
        entry.timestamp = shiftStartTime;
        entry.fromGear = expectedFromGear;
        entry.toGear = expectedToGear;
        entry.rpm = shiftStartRpm;
        entry.shiftTimeMs = (uint16_t)min(shiftTime, 65535UL);
        entry.shiftType = currentShiftType;
        entry.successful = true;
        
        saveLogEntry(entry);
        updateStatistics(entry);
    }
    
    void logFailedShift() {
        ShiftLogEntry entry;
        entry.timestamp = shiftStartTime;
        entry.fromGear = expectedFromGear;
        entry.toGear = expectedToGear;
        entry.rpm = shiftStartRpm;
        entry.shiftTimeMs = SHIFT_TIMEOUT_MS;
        entry.shiftType = currentShiftType;
        entry.successful = false;
        
        saveLogEntry(entry);
        updateStatistics(entry);
    }
    
    void saveLogEntry(const ShiftLogEntry& entry) {
        File f = LittleFS.open(SHIFT_LOG_PATH, "r+");
        if (f) {
            f.seek(slotOffset(currentLogIndex));
            f.write((const uint8_t*)&entry, sizeof(ShiftLogEntry));
            f.close();
        } else {
            Serial.println("ShiftLogger: cannot open ring for write");
        }

        currentLogIndex = (currentLogIndex + 1) % MAX_SHIFT_LOG_ENTRIES;
        if (totalLogEntries < MAX_SHIFT_LOG_ENTRIES) totalLogEntries++;
        writeHeader();

        // Mirror it so the web view never touches flash.
        if (mirrorCount < SHIFT_LOG_MIRROR) {
            mirror[mirrorCount++] = entry;
        } else {
            for (int i = 1; i < SHIFT_LOG_MIRROR; i++) mirror[i - 1] = mirror[i];
            mirror[SHIFT_LOG_MIRROR - 1] = entry;
        }
    }

    bool loadLogEntry(int index, ShiftLogEntry& entry) {
        if (index < 0 || index >= MAX_SHIFT_LOG_ENTRIES) return false;
        File f = LittleFS.open(SHIFT_LOG_PATH, "r");
        if (!f) return false;
        bool ok = f.seek(slotOffset(index)) &&
                  f.read((uint8_t*)&entry, sizeof(ShiftLogEntry)) == sizeof(ShiftLogEntry);
        f.close();
        return ok;
    }

    static uint32_t slotOffset(int index) {
        return sizeof(ShiftLogHeader) + (uint32_t)index * sizeof(ShiftLogEntry);
    }

    void writeHeader() {
        File f = LittleFS.open(SHIFT_LOG_PATH, "r+");
        if (!f) return;
        ShiftLogHeader h = { SHIFT_LOG_MAGIC, SHIFT_LOG_VER,
                             (uint16_t)currentLogIndex, (uint32_t)totalLogEntries,
                             totalShifts, failedShifts, averageShiftTime };
        f.seek(0);
        f.write((const uint8_t*)&h, sizeof(h));
        f.close();
    }

    // Creates the ring on first use and validates it on every boot. A missing, short
    // or foreign file is reinitialised rather than trusted.
    void loadLogIndex() {
        bool needInit = true;
        File f = LittleFS.open(SHIFT_LOG_PATH, "r");
        if (f) {
            ShiftLogHeader h;
            if (f.size() >= slotOffset(MAX_SHIFT_LOG_ENTRIES) &&
                f.read((uint8_t*)&h, sizeof(h)) == sizeof(h) &&
                h.magic == SHIFT_LOG_MAGIC && h.version == SHIFT_LOG_VER &&
                h.head < MAX_SHIFT_LOG_ENTRIES && h.count <= MAX_SHIFT_LOG_ENTRIES) {
                currentLogIndex  = h.head;
                totalLogEntries  = h.count;
                totalShifts      = h.totalShifts;
                failedShifts     = h.failedShifts;
                averageShiftTime = h.avgTime;
                needInit = false;
            }
            f.close();
        }
        if (needInit) initRing();
        fillMirror();
        migrateFromNvs();
    }

    void initRing() {
        File f = LittleFS.open(SHIFT_LOG_PATH, "w");
        if (!f) { Serial.println("ShiftLogger: cannot create ring file"); return; }
        ShiftLogHeader h = { SHIFT_LOG_MAGIC, SHIFT_LOG_VER, 0, 0, 0, 0, 0 };
        f.write((const uint8_t*)&h, sizeof(h));
        ShiftLogEntry blank = {};
        for (int i = 0; i < MAX_SHIFT_LOG_ENTRIES; i++)
            f.write((const uint8_t*)&blank, sizeof(blank));
        f.close();
        currentLogIndex = 0; totalLogEntries = 0;
        totalShifts = 0; failedShifts = 0; averageShiftTime = 0;
        Serial.println("ShiftLogger: ring created, " + String(MAX_SHIFT_LOG_ENTRIES) + " slots");
    }

    void fillMirror() {
        mirrorCount = 0;
        int n = min(totalLogEntries, SHIFT_LOG_MIRROR);
        for (int i = n; i > 0; i--) {
            int idx = (currentLogIndex - i + MAX_SHIFT_LOG_ENTRIES) % MAX_SHIFT_LOG_ENTRIES;
            ShiftLogEntry e;
            if (loadLogEntry(idx, e)) mirror[mirrorCount++] = e;
        }
    }

    // One-time reclaim: the old NVS logs are dead weight, and are exactly what filled
    // the config partition.
    void migrateFromNvs() {
        Preferences p;
        if (!p.begin("shiftlogs", false)) return;
        if (p.isKey("currentIndex") || p.isKey("entry_0")) {
            p.clear();
            Serial.println("ShiftLogger: cleared legacy NVS logs, config space reclaimed");
        }
        p.end();
    }

    void updateStatistics(const ShiftLogEntry& entry) {
        totalShifts++;
        
        if (!entry.successful) {
            failedShifts++;
        } else {
            // Update average shift time
            if (averageShiftTime == 0) {
                averageShiftTime = entry.shiftTimeMs;
            } else {
                // Rolling average calculation
                uint32_t successfulShifts = totalShifts - failedShifts;
                averageShiftTime = (averageShiftTime * (successfulShifts - 1) + entry.shiftTimeMs) / successfulShifts;
            }
        }
        
        // Statistics live in the ring header, not NVS — this ran on every shift, so it
        // was three more NVS writes per gear change on top of the log entry itself.
        writeHeader();
    }
    
    String logEntryToJson(const ShiftLogEntry& entry) {
        String json = "{";
        json += "\"timestamp\":" + String(entry.timestamp) + ",";
        json += "\"from\":\"" + getGearName(entry.fromGear) + "\",";
        json += "\"to\":\"" + getGearName(entry.toGear) + "\",";
        json += "\"rpm\":" + String(entry.rpm) + ",";
        json += "\"time\":" + String(entry.shiftTimeMs) + ",";
        json += "\"type\":\"" + getShiftTypeName(entry.shiftType) + "\",";
        json += "\"success\":" + String(entry.successful ? "true" : "false");
        json += "}";
        return json;
    }
    
    String getGearName(uint8_t gear) {
        if (gear == 0) return "N";
        if (gear >= 1 && gear <= 6) return String(gear);
        return "?";
    }
    
    String getShiftTypeName(uint8_t type) {
        switch (type) {
            case 0: return "upshift";
            case 1: return "downshift"; 
            case 2: return "neutral";
            default: return "unknown";
        }
    }
};

#endif

// end of code