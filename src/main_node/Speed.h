#ifndef SPEED_H
#define SPEED_H

#include <Arduino.h>

// Calibration constant: 18 pulses per second = 1 MPH (moved outside class)
const float SPEED_PULSES_PER_SECOND_PER_MPH = 18.0;

// Global variable for ISR access (must be outside class) - EXTERN DECLARATION
extern volatile unsigned long g_speedPulseCount;

// Simple global ISR function - no class methods - EXTERN DECLARATION
void IRAM_ATTR speedISR();

class Speed {
private:
    // Pin definition
    int speedPin;
    
    // Timing variables for non-blocking calculation
    unsigned long lastCalculationTime;
    unsigned long calculationInterval;
    unsigned long lastSpeedPulseTime;
    
    // Current reading
    float currentMph;
    
    // Moving average array for smoothing
    static const int AVERAGE_SAMPLES = 5;
    float speedSamples[AVERAGE_SAMPLES];
    int sampleIndex;
    bool samplesReady;
    
    // Timeout for zero detection (if no pulses received)
    static const unsigned long PULSE_TIMEOUT = 2000; // 2 seconds

public:
    Speed(int speedInputPin)
        : speedPin(speedInputPin), 
          lastCalculationTime(0), calculationInterval(125), // Calculate every 125ms
          lastSpeedPulseTime(0), currentMph(0),
          sampleIndex(0), samplesReady(false) {
        
        // Initialize averaging array
        for (int i = 0; i < AVERAGE_SAMPLES; i++) {
            speedSamples[i] = 0;
        }
    }
    
    void begin() {
        // Configure input pin with pull-up
        pinMode(speedPin, INPUT_PULLDOWN);
        
        // Reset pulse counter
        g_speedPulseCount = 0;
        lastSpeedPulseTime = millis();
        
        // Attach interrupt - using global C function
        attachInterrupt(digitalPinToInterrupt(speedPin), speedISR, RISING);
        
        lastCalculationTime = millis();
        
        Serial.println("Speed sensor initialized");
        Serial.println("Speed Pin: " + String(speedPin));
        Serial.println("Calibration: 18 pulses/second = 1 MPH");
    }
    
    // Non-blocking update function - call this regularly in main loop
    void update() {
        unsigned long currentTime = millis();
        
        // Calculate speed at regular intervals
        if (currentTime - lastCalculationTime >= calculationInterval) {
            calculateSpeed();
            lastCalculationTime = currentTime;
        }
        
        // Check for timeout (no pulses = zero speed)
        checkTimeout();
    }
    
    // Get current MPH value
    float getMph() {
        return currentMph;
    }
    
    // Reset pulse counter (useful for calibration)
    void resetCounter() {
        noInterrupts();
        g_speedPulseCount = 0;
        interrupts();
        
        // Clear averaging array
        for (int i = 0; i < AVERAGE_SAMPLES; i++) {
            speedSamples[i] = 0;
        }
        sampleIndex = 0;
        samplesReady = false;
        
        currentMph = 0;
        lastSpeedPulseTime = millis();
    }
    
    // Get raw pulse count (for debugging/calibration)
    unsigned long getSpeedPulseCount() {
        return g_speedPulseCount;
    }
    
    // Print status for debugging
    void printStatus() {
        Serial.print("MPH: ");
        Serial.print(currentMph, 1);
        Serial.print(" | Speed Pulses: ");
        Serial.println(g_speedPulseCount);
    }

private:
    void calculateSpeed() {
        // Capture pulse count atomically
        noInterrupts();
        unsigned long speedPulses = g_speedPulseCount;
        g_speedPulseCount = 0;  // Reset counter
        interrupts();
        
        // Calculate MPH: 18 pulses per second = 1 MPH
        // So: MPH = (pulses / time_in_seconds) / 18
        float timeSeconds = calculationInterval / 1000.0; // Convert ms to seconds
        float instantMph = 0;
        if (timeSeconds > 0) {
            float pulsesPerSecond = speedPulses / timeSeconds;
            instantMph = pulsesPerSecond / SPEED_PULSES_PER_SECOND_PER_MPH;
        }
        
        // Add to moving average
        speedSamples[sampleIndex] = instantMph;
        
        sampleIndex = (sampleIndex + 1) % AVERAGE_SAMPLES;
        if (sampleIndex == 0) samplesReady = true;
        
        // Calculate moving average
        currentMph = calculateAverage(speedSamples, samplesReady ? AVERAGE_SAMPLES : sampleIndex);
        
        // Apply minimum threshold to avoid tiny values from noise
        if (currentMph < 0.5) currentMph = 0;
        
        // Update last pulse time for timeout detection
        if (speedPulses > 0) lastSpeedPulseTime = millis();
    }
    
    float calculateAverage(float* samples, int count) {
        if (count == 0) return 0;
        
        float sum = 0;
        for (int i = 0; i < count; i++) {
            sum += samples[i];
        }
        return sum / count;
    }
    
    void checkTimeout() {
        unsigned long currentTime = millis();
        
        // If no speed pulses for timeout period, set speed to zero
        if (currentTime - lastSpeedPulseTime > PULSE_TIMEOUT) {
            currentMph = 0;
        }
    }
};

#endif

// end of code