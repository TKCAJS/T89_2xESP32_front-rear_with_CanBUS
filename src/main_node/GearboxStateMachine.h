#ifndef GEARBOX_STATE_MACHINE_H
#define GEARBOX_STATE_MACHINE_H

#include <Arduino.h>
#include "SimpleServo.h"

// Forward declarations
class ShiftLogger;
class Speed;
class RPM;

// State definitions
enum GearboxState {
    // Idle states - one for each gear
    IDLE_NEUTRAL = 0,
    IDLE_GEAR_1,
    IDLE_GEAR_2, 
    IDLE_GEAR_3,
    IDLE_GEAR_4,
    IDLE_GEAR_5,
    IDLE_GEAR_6,
    
    // Manual shifting states
    NEUTRAL_DOWN_SHIFTING,
    NEUTRAL_UP_SHIFTING,
    UPSHIFTING,
    
    // Complex downshift sequence states
    DOWNSHIFT_CLUTCH_ENGAGING,
    DOWNSHIFT_CLUTCH_ENGAGED,
    DOWNSHIFT_SHIFTING,
    
    // Hall sensor manual control
    MANUAL_CLUTCH_CONTROL,
    
    // Wait states - FIXED: Added separate waiting states for each button type
    WAITING_FOR_CLUTCH_NEUTRAL_DOWN,
    WAITING_FOR_CLUTCH_NEUTRAL_UP,
    WAITING_FOR_CLUTCH_SHIFT_DOWN,
    WAITING_FOR_CLUTCH_SHIFT_UP,
    
    // Error states
    ERROR_SHIFT_TIMEOUT,
    ERROR_SENSOR_DISCONNECTED,
    
    // Total count
    STATE_COUNT
};

// Events that trigger state transitions
enum GearboxEvent {
    // Button events
    EVENT_NEUTRAL_DOWN_PRESSED,
    EVENT_NEUTRAL_UP_PRESSED,
    EVENT_SHIFT_DOWN_PRESSED,  
    EVENT_SHIFT_UP_PRESSED,
    
    // Hardware events
    EVENT_CLUTCH_PULLED,
    EVENT_CLUTCH_RELEASED,
    EVENT_RELAY_FINISHED,
    EVENT_CLUTCH_ENGAGE_COMPLETE,
    EVENT_GEAR_CHANGED,
    EVENT_TIMEOUT,
    EVENT_HALL_SENSOR_INPUT,
    
    // System events
    EVENT_SENSOR_CONNECTED,
    EVENT_SENSOR_DISCONNECTED,
    
    EVENT_COUNT
};

// State transition structure
struct StateTransition {
    GearboxState fromState;
    GearboxEvent event;
    GearboxState toState;
    bool (*condition)();  // Optional condition function
};

class GearboxStateMachine {
private:
    // Current state
    GearboxState currentState;
    GearboxState previousState;
    
    // Timing for state operations
    unsigned long stateStartTime;
    unsigned long lastStateChange;
    
    // Configuration
    int neutralDownMs;
    int neutralUpMs;
    int shiftDownMs;
    int shiftUpMs;
    int clutchIdlePos;
    int clutchEngagePos;
    
    // Hardware references
    ShiftLogger* shiftLogger;
    Speed* speedSensor;
    RPM* rpmSensor;
    SimpleServo* clutchServo;
    
    // Pin definitions
    int pinDownshiftRelay;
    int pinUpshiftRelay;
    int pinHallSensor;
    
    // Relay control
    bool relayActive;
    unsigned long relayStartTime;
    int relayDuration;
    int activeRelayPin;
    
    // Clutch interlock
    bool clutchInterlockEnabled;
    bool clutchPulled;
    
    // Current gear tracking
    int currentGear;  // 0=N, 1-6=gears
    
    // Timeouts - FIXED: Updated constant names to match cpp file
    static const unsigned long STATE_SHIFT_TIMEOUT_MS = 500;
    static const unsigned long CLUTCH_WAIT_TIMEOUT_MS = 200;
    static const unsigned long CLUTCH_ENGAGE_DELAY_MS = 100;
    
    // State transition table access methods - MOVED FROM CPP
    static const StateTransition* getTransitions();
    static const int getTransitionCount();

public:
    GearboxStateMachine(int downRelayPin, int upRelayPin, int hallPin)
        : currentState(IDLE_NEUTRAL), previousState(IDLE_NEUTRAL),
          stateStartTime(0), lastStateChange(0),
          neutralDownMs(40), neutralUpMs(40), shiftDownMs(150), shiftUpMs(150),
          clutchIdlePos(0), clutchEngagePos(180),
          shiftLogger(nullptr), speedSensor(nullptr), rpmSensor(nullptr), clutchServo(nullptr),
          pinDownshiftRelay(downRelayPin), pinUpshiftRelay(upRelayPin), pinHallSensor(hallPin),
          relayActive(false), relayStartTime(0), relayDuration(0), activeRelayPin(-1),
          clutchInterlockEnabled(true), clutchPulled(false),
          currentGear(0) {}
    
    // Initialization
    void begin(ShiftLogger* logger, Speed* speed, RPM* rpm, SimpleServo* servo);
    void setConfiguration(int nDownMs, int nUpMs, int sDownMs, int sUpMs, 
                         int cIdlePos, int cEngagePos);
    
    // Main state machine execution
    void update();
    bool processEvent(GearboxEvent event);
    
    // State queries
    GearboxState getCurrentState() const { return currentState; }
    String getStateName() const { return getStateName(currentState); }
    String getStateName(GearboxState state) const;
    bool isShifting() const;
    bool isIdle() const;
    bool canAcceptShiftCommand() const;
    
    // Gear information
    void setCurrentGear(int gear);
    int getCurrentGear() const { return currentGear; }
    String getCurrentGearName() const;
    
    // Clutch control
    void setClutchPulled(bool pulled) { clutchPulled = pulled; }
    void setClutchInterlockEnabled(bool enabled) { clutchInterlockEnabled = enabled; }
    
    // Manual clutch control (hall sensor)
    void updateManualClutchControl();
    
    // Status for web interface - FIXED: Updated to handle all waiting states
    bool isWaitingForClutch() const { 
        return currentState == WAITING_FOR_CLUTCH_NEUTRAL_DOWN || 
               currentState == WAITING_FOR_CLUTCH_NEUTRAL_UP ||
               currentState == WAITING_FOR_CLUTCH_SHIFT_DOWN ||
               currentState == WAITING_FOR_CLUTCH_SHIFT_UP; 
    }
    bool isShiftInProgress() const;
    
    // Debug
    void printStateInfo() const;

private:
    // State transition logic
    bool transitionToState(GearboxState newState);
    void executeStateEntry();
    void executeStateUpdate();
    void executeStateExit();
    
    // State handlers
    void enterIdleState();
    void enterShiftingState();
    void enterWaitingState();
    void enterErrorState();
    
    void updateIdleState();
    void updateShiftingState();
    void updateWaitingState();
    void updateErrorState();
    
    void exitShiftingState();
    
    // Hardware control
    void activateRelay(int pin, int duration);
    void deactivateRelay();
    void engageClutch();
    void releaseClutch();
    void updateRelayControl();
    
    // Utility functions
    GearboxState getIdleStateForGear(int gear) const;
    int getGearForIdleState(GearboxState state) const;
    bool isIdleState(GearboxState state) const;
    bool isShiftingState(GearboxState state) const;
    
    // Condition checkers for transitions
    static bool clutchPulledCondition();
    static bool clutchNotRequiredCondition();
    
    // Timeout handling
    void checkTimeouts();
    unsigned long getStateElapsedTime() const { return millis() - stateStartTime; }
};

// Global state machine instance for condition functions - MOVED FROM CPP
extern GearboxStateMachine* g_stateMachine;

#endif

// end of code