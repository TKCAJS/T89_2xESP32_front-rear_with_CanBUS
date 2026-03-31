#ifndef WATER_TEMP_CONTROLLER_H
#define WATER_TEMP_CONTROLLER_H
#include <OneWire.h>
#include <DallasTemperature.h>

class WaterTempController {

private:

    // Pin definitions
    int tempSensorPin;
    int pumpPwmPin;
    int engineDetectPin;
  
    // PWM configuration
    static const int pwmFreq = 100;
    static const int pwmResolution = 8;
    // Use a higher channel to avoid servo conflicts
    static const int pwmChannel = 6;  // Changed from 0 to 6

    
    // Temperature sensor objects
    OneWire oneWire;
    DallasTemperature tempSensor;
    
    // Control variables
    float targetTemp;
    float currentTemp;
    bool engineRunning;
    unsigned long lastPumpAction;
    unsigned long engineStoppedTime;
    bool pumpState;
    int currentPwmDuty;
   
    // Timing variables for pulsed operation
    unsigned long onDuration;
    unsigned long offDuration;
    bool pulseMode;
   
    // Non-blocking temperature reading
    unsigned long lastTempRequest;
    bool tempRequested;
    static const unsigned long TEMP_REQUEST_INTERVAL = 500; // Read temp every 1 second

    // Engine not running timer (3 minutes = 180000 ms)
    static const unsigned long ENGINE_OFF_RUNTIME = 180000;

    // Reduce serial output frequency
    unsigned long lastSerialOutput;
    static const unsigned long SERIAL_INTERVAL = 10000; // Print status every 10 seconds

    // Sensor status tracking
    bool sensorConnected;
    int sensorCount;
    unsigned long lastSensorCheck;
    static const unsigned long SENSOR_CHECK_INTERVAL = 5000; // Check sensor every 5 seconds

public:

    WaterTempController(int tempPin, int pwmPin, int enginePin, float target = 80.0) 
        : tempSensorPin(tempPin), pumpPwmPin(pwmPin), engineDetectPin(enginePin),
          oneWire(tempPin), tempSensor(&oneWire), targetTemp(target),
          currentTemp(0), engineRunning(false), lastPumpAction(0), 
          engineStoppedTime(0), pumpState(false), currentPwmDuty(0),
          onDuration(0), offDuration(0), pulseMode(false),
          lastTempRequest(0), tempRequested(false), lastSerialOutput(0),
          sensorConnected(false), sensorCount(0), lastSensorCheck(0) {}

    
    void begin() {
        Serial.println("=== Water Temperature Controller Initialization ===");
        Serial.println("Temp Sensor Pin: " + String(tempSensorPin));
        Serial.println("Pump PWM Pin: " + String(pumpPwmPin));
        Serial.println("Engine Detect Pin: " + String(engineDetectPin));

        // Initialize temperature sensor
        tempSensor.begin();
        tempSensor.setWaitForConversion(false); // Make it non-blocking
        
        // Check sensor connection
        sensorCount = tempSensor.getDeviceCount();
        Serial.println("DS18B20 sensor count: " + String(sensorCount));
        
        if (sensorCount > 0) {
            sensorConnected = true;
            Serial.println("Temperature sensor detected successfully!");
            
            // Print sensor address for debugging
            DeviceAddress tempDeviceAddress;
            if (tempSensor.getAddress(tempDeviceAddress, 0)) {
                Serial.print("Sensor address: ");
                for (uint8_t i = 0; i < 8; i++) {
                    if (tempDeviceAddress[i] < 16) Serial.print("0");
                    Serial.print(tempDeviceAddress[i], HEX);
                }
                Serial.println();
            }
            
            // Set sensor resolution (optional, 12-bit = highest accuracy)
            tempSensor.setResolution(12);
            Serial.println("Sensor resolution set to 12-bit");
        } else {
            sensorConnected = false;
            Serial.println("WARNING: No DS18B20 temperature sensors found!");
            Serial.println("Check wiring and 4.7k pull-up resistor on data line");
        }
        
        // Configure engine detect pin
        pinMode(engineDetectPin, INPUT_PULLUP);
        
        // Configure PWM for pump control using newer API with specific channel
        if (!ledcAttach(pumpPwmPin, pwmFreq, pwmResolution)) {
            Serial.println("ERROR: Failed to attach LEDC to pump PWM pin " + String(pumpPwmPin));
        } else {
            Serial.println("Water pump PWM initialized successfully on pin " + String(pumpPwmPin));
            Serial.println("PWM Frequency: " + String(pwmFreq) + "Hz");
            Serial.println("PWM Resolution: " + String(pwmResolution) + "-bit");
        }
        
        // Initialize PWM to 0
        ledcWrite(pumpPwmPin, 0);
        
        // Request first temperature reading if sensor connected
        if (sensorConnected) {
            tempSensor.requestTemperatures();
            tempRequested = true;
            lastTempRequest = millis();
            Serial.println("First temperature reading requested");
        }
        
        Serial.println("=== Temperature Controller Ready ===");
    }

    
    void setTargetTemperature(float temp) {
        targetTemp = temp;
        Serial.println("Target temperature updated to: " + String(targetTemp, 1) + "°C");
    }

    
    float getTargetTemperature() {
        return targetTemp;
    }

    
    float getCurrentTemperature() {
        return currentTemp;
    }

    
    bool isEngineRunning() {
        return engineRunning;
    }
    
    bool isSensorConnected() {
        return sensorConnected;
    }

    
    // Non-blocking update function
    void update() {
        // Periodically check sensor connection
        checkSensorConnection();
        
        // Non-blocking temperature reading
        updateTemperature();
        
        // Check engine status (assuming LOW = engine running)
        bool previousEngineState = engineRunning;
        engineRunning = !digitalRead(engineDetectPin);
        
        // Track when engine stops
        if (previousEngineState && !engineRunning) {
            engineStoppedTime = millis();
            Serial.println("Engine stopped - starting cooldown timer");
        }
        
        // Only proceed if we have valid temperature and sensor is connected
        if (!sensorConnected || currentTemp == DEVICE_DISCONNECTED_C || currentTemp == 85.0) {
            setPumpPWM(0); // Safety: turn off pump if sensor fails
            return;
        }
        
        // Calculate temperature difference
        float tempDiff = targetTemp - currentTemp;
        
        // Apply appropriate algorithm
        if (engineRunning) {
            runEngineOnAlgorithm(tempDiff);
        } else {
            runEngineOffAlgorithm(tempDiff);
        }
        
        // Handle pulsed operation
        handlePulsedOperation();
        
        // Occasional status output (much less frequent)
        if (millis() - lastSerialOutput >= SERIAL_INTERVAL) {
            printStatus();
            lastSerialOutput = millis();
        }
    }

    
private:
    void checkSensorConnection() {
        unsigned long now = millis();
        if (now - lastSensorCheck >= SENSOR_CHECK_INTERVAL) {
            int currentSensorCount = tempSensor.getDeviceCount();
            if (currentSensorCount != sensorCount) {
                sensorCount = currentSensorCount;
                sensorConnected = (sensorCount > 0);
                Serial.println("Sensor count changed: " + String(sensorCount));
                if (!sensorConnected) {
                    Serial.println("WARNING: Temperature sensor disconnected!");
                } else {
                    Serial.println("Temperature sensor reconnected!");
                }
            }
            lastSensorCheck = now;
        }
    }

    void updateTemperature() {
        if (!sensorConnected) return;
        
        unsigned long now = millis();
        
        if (tempRequested && (now - lastTempRequest >= 750)) {
            // Temperature conversion should be ready (DS18B20 takes ~750ms)
            float newTemp = tempSensor.getTempCByIndex(0);
            
            // Debug output for temperature reading
            if (millis() - lastSerialOutput >= SERIAL_INTERVAL) {
                Serial.println("Raw temperature reading: " + String(newTemp, 2) + "°C");
            }
            
            if (newTemp != DEVICE_DISCONNECTED_C && newTemp != 85.0) {
                // 85.0 is often a sensor error reading
                if (abs(newTemp - currentTemp) > 10.0 && currentTemp != 0) {
                    // Large temperature jump - could be sensor error
                    Serial.println("WARNING: Large temperature change detected: " + 
                                 String(currentTemp, 1) + "°C -> " + String(newTemp, 1) + "°C");
                }
                currentTemp = newTemp;
            } else {
                Serial.println("ERROR: Invalid temperature reading: " + String(newTemp));
                sensorConnected = false; // Mark sensor as disconnected
            }
            tempRequested = false;
        }
        
        if (!tempRequested && (now - lastTempRequest >= TEMP_REQUEST_INTERVAL)) {
            // Request new temperature reading
            tempSensor.requestTemperatures();
            tempRequested = true;
            lastTempRequest = now;
        }
    }

    
    void runEngineOnAlgorithm(float tempDiff) {
        if (tempDiff > 20) {
            // >20°C: PWM=50%, 10s on/30s off
            setPulseMode(50, 10000, 30000);
        } 
        else if (tempDiff > 5) {
            // >5 & <20°C: PWM=50%, 10s on/10s off
            setPulseMode(50, 10000, 10000);
        } 
        else if (tempDiff >= 0) {
            // <5°C: PWM=80%, full on
            setFullOn(80);
        } 
        else {
            // <0°C (overheat): PWM=100%, full on
            setFullOn(100);
        }
    }

    
    void runEngineOffAlgorithm(float tempDiff) {
        // Check if 3-minute timer has expired
        if (millis() - engineStoppedTime > ENGINE_OFF_RUNTIME) {
            setPumpPWM(0);
            pulseMode = false;
            return;
        }
        
        if (tempDiff > 20) {
            // >20°C: PWM=0%
            setFullOn(0);
        } 
        else if (tempDiff > 5) {
            // >5 & <20°C: PWM=30%
            setFullOn(30);
        } 
        else {
            // <5°C: PWM=50%
            setFullOn(50);
        }
    }

    
    void setPulseMode(int pwmPercent, unsigned long onTime, unsigned long offTime) {
        if (!pulseMode || onDuration != onTime || offDuration != offTime) {
            // Only update if mode changes
            pulseMode = true;
            currentPwmDuty = map(pwmPercent, 0, 100, 0, 255);
            onDuration = onTime;
            offDuration = offTime;
        }
    }

    
    void setFullOn(int pwmPercent) {
        if (pulseMode) {
            pulseMode = false;
        }
        int pwmValue = map(pwmPercent, 0, 100, 0, 255);
        setPumpPWM(pwmValue);
    }

    
    void setPumpPWM(int pwmValue) {
        pwmValue = constrain(pwmValue, 0, 255);
        ledcWrite(pumpPwmPin, pwmValue);
    }

    
    void handlePulsedOperation() {
        if (!pulseMode) return;
        
        if (millis() - lastPumpAction >= (pumpState ? onDuration : offDuration)) {
            pumpState = !pumpState;
            lastPumpAction = millis();
            
            if (pumpState) {
                setPumpPWM(currentPwmDuty);
            } else {
                setPumpPWM(0);
            }
        }
    }

    
public:
    void printStatus() {
        Serial.print("TEMP CONTROLLER - ");
        if (!sensorConnected) {
            Serial.println("SENSOR DISCONNECTED!");
            return;
        }
        
        Serial.print("Temp: ");
        Serial.print(currentTemp, 1);
        Serial.print("°C, Target: ");
        Serial.print(targetTemp, 1);
        Serial.print("°C, Diff: ");
        Serial.print(targetTemp - currentTemp, 1);
        Serial.print("°C, Engine: ");
        Serial.print(engineRunning ? "ON" : "OFF");
        Serial.print(", Pump: ");
        Serial.print(getCurrentPwmPercent());
        Serial.print("%");
        
        if (pulseMode) {
            Serial.print(" (PULSE)");
        }
        
        if (!engineRunning && (millis() - engineStoppedTime <= ENGINE_OFF_RUNTIME)) {
            Serial.print(", Timer: ");
            Serial.print((ENGINE_OFF_RUNTIME - (millis() - engineStoppedTime)) / 1000);
            Serial.print("s");
        }
        Serial.println();
    }

    
    // Quick status check without serial output
    bool isPumpActive() {
        return (pulseMode && pumpState) || (!pulseMode && currentPwmDuty > 0);
    }

    
    int getCurrentPwmPercent() {
        return map(currentPwmDuty, 0, 255, 0, 100);
    }
};

#endif

// end of code