#ifndef SERIAL_COMMANDS_H
#define SERIAL_COMMANDS_H

#include <Arduino.h>

// Forward declarations
class HallSensorControl;
class GearboxStateMachine;
class ShiftLogger;
class GearSensorControl;

class SerialCommands {
private:
    HallSensorControl* hallSensor;
    GearboxStateMachine* gearbox;
    ShiftLogger* shiftLogger;
    GearSensorControl* gearSensor;
    
public:
    SerialCommands() : hallSensor(nullptr), gearbox(nullptr), 
                      shiftLogger(nullptr), gearSensor(nullptr) {}
    
    void begin(HallSensorControl* hall, GearboxStateMachine* stateMachine, 
               ShiftLogger* logger, GearSensorControl* sensor) {
        hallSensor = hall;
        gearbox = stateMachine;
        shiftLogger = logger;
        gearSensor = sensor;
        
        Serial.println("Serial command interface ready");
        Serial.println("Type 'help' for available commands");
    }
    
    // Method declarations only - implementations will be in the main file
    void processCommands();
    void printHelp();
};

#endif

// end of code