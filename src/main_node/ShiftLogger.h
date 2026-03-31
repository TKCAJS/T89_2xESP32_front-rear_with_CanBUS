#ifndef SHIFT_LOGGER_H
#define SHIFT_LOGGER_H

#include <Arduino.h>
#include <Preferences.h>

// Pin definition for ignition cut
#define PIN_IGNITION_CUT 18

// Shift logging configuration
#define MAX_SHIFT_LOG_ENTRIES 100
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
    Preferences shiftPrefs;
    int currentLogIndex;
    int totalLogEntries;
    
    // Statistics
    uint32_t totalShifts;
    uint32_t failedShifts;
    uint16_t averageShiftTime;

public:
    ShiftLogger() : shiftTimingActive(false), shiftStartTime(0), 
                    expectedFromGear(0), expectedToGear(0), shiftStartRpm(0),
                    currentShiftType(0), currentLogIndex(0), totalLogEntries(0),
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
    String getRecentLogs(int count = 10) {
        String logs = "[";
        int startIndex = max(0, currentLogIndex - count);
        bool first = true;
        
        for (int i = 0; i < min(count, totalLogEntries); i++) {
            ShiftLogEntry entry;
            if (loadLogEntry((startIndex + i) % MAX_SHIFT_LOG_ENTRIES, entry)) {
                if (!first) logs += ",";
                logs += logEntryToJson(entry);
                first = false;
            }
        }
        
        logs += "]";
        return logs;
    }
    
    // Clear all shift logs
    void clearLogs() {
        shiftPrefs.begin("shiftlogs", false);
        shiftPrefs.clear();
        shiftPrefs.end();
        
        currentLogIndex = 0;
        totalLogEntries = 0;
        totalShifts = 0;
        failedShifts = 0;
        averageShiftTime = 0;
        
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
        shiftPrefs.begin("shiftlogs", false);
        
        // Create key for this entry
        String key = "entry_" + String(currentLogIndex);
        
        // Pack the struct into bytes for storage
        uint8_t data[sizeof(ShiftLogEntry)];
        memcpy(data, &entry, sizeof(ShiftLogEntry));
        
        shiftPrefs.putBytes(key.c_str(), data, sizeof(ShiftLogEntry));
        
        // Update counters
        currentLogIndex = (currentLogIndex + 1) % MAX_SHIFT_LOG_ENTRIES;
        if (totalLogEntries < MAX_SHIFT_LOG_ENTRIES) {
            totalLogEntries++;
        }
        
        // Save index
        shiftPrefs.putInt("currentIndex", currentLogIndex);
        shiftPrefs.putInt("totalEntries", totalLogEntries);
        
        shiftPrefs.end();
    }
    
    bool loadLogEntry(int index, ShiftLogEntry& entry) {
        shiftPrefs.begin("shiftlogs", true);
        String key = "entry_" + String(index);
        
        uint8_t data[sizeof(ShiftLogEntry)];
        size_t bytesRead = shiftPrefs.getBytes(key.c_str(), data, sizeof(ShiftLogEntry));
        
        shiftPrefs.end();
        
        if (bytesRead == sizeof(ShiftLogEntry)) {
            memcpy(&entry, data, sizeof(ShiftLogEntry));
            return true;
        }
        return false;
    }
    
    void loadLogIndex() {
        shiftPrefs.begin("shiftlogs", true);
        currentLogIndex = shiftPrefs.getInt("currentIndex", 0);
        totalLogEntries = shiftPrefs.getInt("totalEntries", 0);
        totalShifts = shiftPrefs.getUInt("totalShifts", 0);
        failedShifts = shiftPrefs.getUInt("failedShifts", 0);
        averageShiftTime = shiftPrefs.getUShort("avgTime", 0);
        shiftPrefs.end();
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
        
        // Save statistics
        shiftPrefs.begin("shiftlogs", false);
        shiftPrefs.putUInt("totalShifts", totalShifts);
        shiftPrefs.putUInt("failedShifts", failedShifts);
        shiftPrefs.putUShort("avgTime", averageShiftTime);
        shiftPrefs.end();
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