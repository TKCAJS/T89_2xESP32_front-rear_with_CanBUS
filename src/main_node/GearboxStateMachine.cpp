// GearboxStateMachine.cpp v19 - Fixed racing downshift sequence with immediate relay activation

#include "GearboxStateMachine.h"
#include "ShiftLogger.h"
#include "RPM.h"

// External pin definitions and functions
extern void displayShiftLetter(char letter);

#define RELAY_ON HIGH
#define RELAY_OFF LOW

// Global state machine instance for condition functions
GearboxStateMachine* g_stateMachine = nullptr;

// State transition table for motorcycle-style sequential gearbox
// Pattern: 1st ↔ N ↔ 2nd → 3rd → 4th → 5th → 6th
// FIXED: Separate waiting states preserve original button press → correct relay action
const StateTransition stateTransitions[] = {
    // FROM NEUTRAL STATE
    // Neutral is between 1st and 2nd - can go to either with full shifts
    {IDLE_NEUTRAL, EVENT_SHIFT_DOWN_PRESSED, WAITING_FOR_CLUTCH_SHIFT_DOWN, nullptr},  // N → 1st (clutch required)
    {IDLE_NEUTRAL, EVENT_SHIFT_UP_PRESSED, WAITING_FOR_CLUTCH_SHIFT_UP, nullptr},      // N → 2nd (clutch required)
    // Neutral Down/Up buttons do nothing when already in neutral
    
    // FROM 1ST GEAR
    // Can only go to neutral via neutral up button (clutch required) or shift up to 2nd
    {IDLE_GEAR_1, EVENT_NEUTRAL_UP_PRESSED, WAITING_FOR_CLUTCH_NEUTRAL_UP, nullptr},   // 1st → N (clutch required)
    {IDLE_GEAR_1, EVENT_SHIFT_UP_PRESSED, UPSHIFTING, nullptr},                       // 1st → 2nd (ignition cut)
    // All other buttons invalid from 1st gear                  
    
    // FROM 2ND GEAR  
    // Can go to neutral (half shift down), or skip neutral to 1st (full shift down), or up to 3rd
    {IDLE_GEAR_2, EVENT_NEUTRAL_DOWN_PRESSED, WAITING_FOR_CLUTCH_NEUTRAL_DOWN, nullptr}, // 2nd → N (clutch required)
    {IDLE_GEAR_2, EVENT_SHIFT_DOWN_PRESSED, DOWNSHIFT_CLUTCH_ENGAGING, nullptr},         // 2nd → 1st (servo, skip N)
    {IDLE_GEAR_2, EVENT_SHIFT_UP_PRESSED, UPSHIFTING, nullptr},                         // 2nd → 3rd (ignition cut)
    // Neutral up invalid from 2nd gear
    
    // FROM 3RD GEAR
    // Can only shift up/down, no neutral access
    {IDLE_GEAR_3, EVENT_SHIFT_DOWN_PRESSED, DOWNSHIFT_CLUTCH_ENGAGING, nullptr}, // 3rd → 2nd (servo)
    {IDLE_GEAR_3, EVENT_SHIFT_UP_PRESSED, UPSHIFTING, nullptr},                 // 3rd → 4th (ignition cut)
    // Neutral buttons invalid from 3rd gear
    
    // FROM 4TH GEAR
    // Can only shift up/down, no neutral access
    {IDLE_GEAR_4, EVENT_SHIFT_DOWN_PRESSED, DOWNSHIFT_CLUTCH_ENGAGING, nullptr}, // 4th → 3rd (servo)
    {IDLE_GEAR_4, EVENT_SHIFT_UP_PRESSED, UPSHIFTING, nullptr},                 // 4th → 5th (ignition cut)
    // Neutral buttons invalid from 4th gear
    
    // FROM 5TH GEAR
    // Can only shift up/down, no neutral access  
    {IDLE_GEAR_5, EVENT_SHIFT_DOWN_PRESSED, DOWNSHIFT_CLUTCH_ENGAGING, nullptr}, // 5th → 4th (servo)
    {IDLE_GEAR_5, EVENT_SHIFT_UP_PRESSED, UPSHIFTING, nullptr},                 // 5th → 6th (ignition cut)
    // Neutral buttons invalid from 5th gear
    
    // FROM 6TH GEAR
    // Can only shift down, no neutral access, can't shift up (top gear)
    {IDLE_GEAR_6, EVENT_SHIFT_DOWN_PRESSED, DOWNSHIFT_CLUTCH_ENGAGING, nullptr}, // 6th → 5th (servo)
    // All other buttons invalid from 6th gear
    
    // CLUTCH INTERLOCK HANDLING - FIXED: Each waiting state goes to correct shifting state
    {WAITING_FOR_CLUTCH_NEUTRAL_DOWN, EVENT_CLUTCH_PULLED, NEUTRAL_DOWN_SHIFTING, nullptr},  // Use downshift relay
    {WAITING_FOR_CLUTCH_NEUTRAL_DOWN, EVENT_TIMEOUT, IDLE_NEUTRAL, nullptr},                  
    {WAITING_FOR_CLUTCH_NEUTRAL_UP, EVENT_CLUTCH_PULLED, NEUTRAL_UP_SHIFTING, nullptr},      // Use upshift relay
    {WAITING_FOR_CLUTCH_NEUTRAL_UP, EVENT_TIMEOUT, IDLE_NEUTRAL, nullptr},                   
    {WAITING_FOR_CLUTCH_SHIFT_DOWN, EVENT_CLUTCH_PULLED, NEUTRAL_DOWN_SHIFTING, nullptr},    // Use downshift relay
    {WAITING_FOR_CLUTCH_SHIFT_DOWN, EVENT_TIMEOUT, IDLE_NEUTRAL, nullptr},                   
    {WAITING_FOR_CLUTCH_SHIFT_UP, EVENT_CLUTCH_PULLED, NEUTRAL_UP_SHIFTING, nullptr},        // Use upshift relay  
    {WAITING_FOR_CLUTCH_SHIFT_UP, EVENT_TIMEOUT, IDLE_NEUTRAL, nullptr},                     
    
    // DOWNSHIFT SEQUENCE STATES - RACING OPTIMIZED
    // Complex downshift with servo clutch control - IMMEDIATE relay activation on clutch pull
    {DOWNSHIFT_CLUTCH_ENGAGING, EVENT_CLUTCH_PULLED, DOWNSHIFT_SHIFTING, nullptr},           // RACING: Immediate relay on clutch pull
    {DOWNSHIFT_SHIFTING, EVENT_RELAY_FINISHED, IDLE_NEUTRAL, nullptr},                       // Will update via gear change
    
    // SIMPLE SHIFT COMPLETIONS
    {NEUTRAL_DOWN_SHIFTING, EVENT_RELAY_FINISHED, IDLE_NEUTRAL, nullptr},        // Will update via gear change
    {NEUTRAL_UP_SHIFTING, EVENT_RELAY_FINISHED, IDLE_NEUTRAL, nullptr},          // Will update via gear change  
    {UPSHIFTING, EVENT_RELAY_FINISHED, IDLE_NEUTRAL, nullptr},                   // Will update via gear change
    
    // GEAR CHANGE EVENT HANDLING
    // This allows the state machine to update when the physical gear actually changes
    {IDLE_NEUTRAL, EVENT_GEAR_CHANGED, IDLE_NEUTRAL, nullptr},                   // Handled specially in code
    
    // ERROR RECOVERY
    {ERROR_SHIFT_TIMEOUT, EVENT_TIMEOUT, IDLE_NEUTRAL, nullptr},
    {ERROR_SENSOR_DISCONNECTED, EVENT_SENSOR_CONNECTED, IDLE_NEUTRAL, nullptr},
};

const int stateTransitionCount = sizeof(stateTransitions) / sizeof(StateTransition);

const StateTransition* GearboxStateMachine::getTransitions() {
    return stateTransitions;
}

const int GearboxStateMachine::getTransitionCount() {
    return stateTransitionCount;
}

void GearboxStateMachine::begin(ShiftLogger* logger, Speed* speed, RPM* rpm, SimpleServo* servo) {
    shiftLogger = logger;
    speedSensor = speed;
    rpmSensor = rpm;
    clutchServo = servo;
    
    // Set global reference for condition functions
    g_stateMachine = this;
    
    // Initialize to idle state based on current gear
    currentState = getIdleStateForGear(currentGear);
    stateStartTime = millis();
    lastStateChange = millis();
    
    Serial.println("GearboxStateMachine initialized");
    Serial.println("Initial state: " + getStateName(currentState));
}

void GearboxStateMachine::setConfiguration(int nDownMs, int nUpMs, int sDownMs, int sUpMs, 
                                         int cIdlePos, int cEngagePos) {
    neutralDownMs = nDownMs;
    neutralUpMs = nUpMs;
    shiftDownMs = sDownMs;
    shiftUpMs = sUpMs;
    clutchIdlePos = cIdlePos;
    clutchEngagePos = cEngagePos;
}

void GearboxStateMachine::update() {
    // Update relay control
    updateRelayControl();
    
    // Check for timeouts
    checkTimeouts();
    
    // Execute current state logic
    executeStateUpdate();
}

bool GearboxStateMachine::processEvent(GearboxEvent event) {
    // Special handling for gear change events
    if (event == EVENT_GEAR_CHANGED) {
        GearboxState newIdleState = getIdleStateForGear(currentGear);
        if (newIdleState != currentState) {
            Serial.println("Gear changed - updating state to: " + getStateName(newIdleState));
            return transitionToState(newIdleState);
        }
        return true;
    }
    
    // Look for valid transition
    const StateTransition* transitions = getTransitions();
    int transitionCount = getTransitionCount();
    
    for (int i = 0; i < transitionCount; i++) {
        const StateTransition& trans = transitions[i];
        
        if (trans.fromState == currentState && trans.event == event) {
            // Check condition if present
            if (trans.condition == nullptr || trans.condition()) {
                return transitionToState(trans.toState);
            }
        }
    }
    
    // No valid transition found
    Serial.println("No transition for event in state " + getStateName(currentState));
    return false;
}

bool GearboxStateMachine::transitionToState(GearboxState newState) {
    if (newState == currentState) return true;
    
    Serial.print("State transition: " + getStateName(currentState));
    Serial.println(" -> " + getStateName(newState));
    
    // Execute exit logic for current state
    executeStateExit();
    
    // Update state
    previousState = currentState;
    currentState = newState;
    stateStartTime = millis();
    lastStateChange = millis();
    
    // Execute entry logic for new state
    executeStateEntry();
    
    return true;
}

void GearboxStateMachine::executeStateEntry() {
    switch (currentState) {
        case IDLE_NEUTRAL:
        case IDLE_GEAR_1:
        case IDLE_GEAR_2:
        case IDLE_GEAR_3:
        case IDLE_GEAR_4:
        case IDLE_GEAR_5:
        case IDLE_GEAR_6:
            enterIdleState();
            break;
            
        case NEUTRAL_DOWN_SHIFTING:
        case NEUTRAL_UP_SHIFTING:
        case UPSHIFTING:
        case DOWNSHIFT_CLUTCH_ENGAGING:
        case DOWNSHIFT_CLUTCH_ENGAGED:
        case DOWNSHIFT_SHIFTING:
            enterShiftingState();
            break;
            
        case WAITING_FOR_CLUTCH_NEUTRAL_DOWN:
        case WAITING_FOR_CLUTCH_NEUTRAL_UP:
        case WAITING_FOR_CLUTCH_SHIFT_DOWN:
        case WAITING_FOR_CLUTCH_SHIFT_UP:
            enterWaitingState();
            break;
            
        case MANUAL_CLUTCH_CONTROL:
            // No special entry action
            break;
            
        case ERROR_SHIFT_TIMEOUT:
        case ERROR_SENSOR_DISCONNECTED:
            enterErrorState();
            break;
            
        default:
            break;
    }
}

void GearboxStateMachine::executeStateUpdate() {
    switch (currentState) {
        case IDLE_NEUTRAL:
        case IDLE_GEAR_1:
        case IDLE_GEAR_2:
        case IDLE_GEAR_3:
        case IDLE_GEAR_4:
        case IDLE_GEAR_5:
        case IDLE_GEAR_6:
            updateIdleState();
            break;
            
        case NEUTRAL_DOWN_SHIFTING:
        case NEUTRAL_UP_SHIFTING:
        case UPSHIFTING:
        case DOWNSHIFT_CLUTCH_ENGAGING:
        case DOWNSHIFT_CLUTCH_ENGAGED:
        case DOWNSHIFT_SHIFTING:
            updateShiftingState();
            break;
            
        case WAITING_FOR_CLUTCH_NEUTRAL_DOWN:
        case WAITING_FOR_CLUTCH_NEUTRAL_UP:
        case WAITING_FOR_CLUTCH_SHIFT_DOWN:
        case WAITING_FOR_CLUTCH_SHIFT_UP:
            updateWaitingState();
            break;
            
        case MANUAL_CLUTCH_CONTROL:
            updateManualClutchControl();
            break;
            
        case ERROR_SHIFT_TIMEOUT:
        case ERROR_SENSOR_DISCONNECTED:
            updateErrorState();
            break;
            
        default:
            break;
    }
}

void GearboxStateMachine::executeStateExit() {
    switch (currentState) {
        case NEUTRAL_DOWN_SHIFTING:
        case NEUTRAL_UP_SHIFTING:
        case UPSHIFTING:
        case DOWNSHIFT_CLUTCH_ENGAGING:
        case DOWNSHIFT_CLUTCH_ENGAGED:
        case DOWNSHIFT_SHIFTING:
            exitShiftingState();
            break;
            
        default:
            break;
    }
}

void GearboxStateMachine::enterIdleState() {
    // Release clutch to idle position
    releaseClutch();
    
    // Ensure relays are off
    deactivateRelay();
    
    Serial.println("Entered idle state for gear: " + getCurrentGearName());
}

void GearboxStateMachine::enterShiftingState() {
    Serial.println("Starting shift operation: " + getStateName(currentState));
    
    // Start shift timing based on state and determine target gear
    if (shiftLogger) {
        uint8_t shiftType = 0; // upshift
        int fromGear = currentGear;
        int toGear = currentGear;
        
        switch (currentState) {
            case NEUTRAL_DOWN_SHIFTING:
                // Triggered by Neutral Down button - always use DOWNSHIFT relay
                if (fromGear == 2) {
                    // 2nd → N (half-shift)
                    shiftType = 2; // neutral
                    toGear = 0;
                } else if (fromGear == 0) {
                    // N → 1st (full shift)
                    shiftType = 1; // downshift
                    toGear = 1;
                }
                shiftLogger->startShiftTiming(fromGear, toGear, rpmSensor->getRpm(), shiftType);
                activateRelay(pinDownshiftRelay, (fromGear == 0) ? shiftDownMs : neutralDownMs);
                displayShiftLetter('D');
                break;
                
            case NEUTRAL_UP_SHIFTING:
                // Triggered by Neutral Up button - always use UPSHIFT relay
                if (fromGear == 1) {
                    // 1st → N (half-shift)
                    shiftType = 2; // neutral
                    toGear = 0;
                } else if (fromGear == 0) {
                    // N → 2nd (full shift)
                    shiftType = 0; // upshift  
                    toGear = 2;
                }
                shiftLogger->startShiftTiming(fromGear, toGear, rpmSensor->getRpm(), shiftType);
                activateRelay(pinUpshiftRelay, (fromGear == 0) ? shiftUpMs : neutralUpMs);
                displayShiftLetter('U');
                break;
                
            case UPSHIFTING:
                // Triggered by Shift Up button - always use UPSHIFT relay
                shiftType = 0; // upshift
                toGear = fromGear + 1;
                shiftLogger->startIgnitionCut();
                shiftLogger->startShiftTiming(fromGear, toGear, rpmSensor->getRpm(), shiftType);
                activateRelay(pinUpshiftRelay, shiftUpMs);
                displayShiftLetter('U');
                break;
                
            case DOWNSHIFT_CLUTCH_ENGAGING:
                // Triggered by Shift Down button - engage clutch, wait for pull detection
                shiftType = 1; // downshift  
                toGear = fromGear - 1;
                shiftLogger->startShiftTiming(fromGear, toGear, rpmSensor->getRpm(), shiftType);
                engageClutch();  // CRITICAL: Engage the clutch servo
                displayShiftLetter('D');
                break;
                
            case DOWNSHIFT_CLUTCH_ENGAGED:
                // Legacy state - should not be used in racing sequence
                Serial.println("WARNING: DOWNSHIFT_CLUTCH_ENGAGED state entered - this should not happen in racing sequence");
                break;
                
            case DOWNSHIFT_SHIFTING:
                // Triggered when clutch is pulled - activate relay immediately
                Serial.println("RACING DOWNSHIFT: Clutch pulled detected - firing relay immediately!");
                activateRelay(pinDownshiftRelay, shiftDownMs);
                break;
                
            default:
                break;
        }
    }
}

void GearboxStateMachine::enterWaitingState() {
    Serial.println("Waiting for clutch to be pulled...");
    displayShiftLetter('C');
}

void GearboxStateMachine::enterErrorState() {
    Serial.println("Entered error state: " + getStateName(currentState));
    
    // Ensure safe state
    deactivateRelay();
    releaseClutch();
}

void GearboxStateMachine::updateIdleState() {
    // In idle state, we don't actively do anything
    // Events are handled by the main loop calling processEvent()
}

void GearboxStateMachine::updateShiftingState() {
    switch (currentState) {
        case DOWNSHIFT_CLUTCH_ENGAGING:
            // RACING: No time-based transitions - only voltage-based clutch detection
            // The clutch pull will be detected by main loop and trigger EVENT_CLUTCH_PULLED
            // No action needed here - just wait for voltage-based clutch detection
            break;
            
        case DOWNSHIFT_CLUTCH_ENGAGED:
            // Legacy state - should not be used in racing sequence
            Serial.println("WARNING: updateShiftingState() - DOWNSHIFT_CLUTCH_ENGAGED should not be active");
            // Failsafe: transition to shifting if we somehow get here
            if (getStateElapsedTime() >= CLUTCH_ENGAGE_DELAY_MS) {
                processEvent(EVENT_TIMEOUT);
            }
            break;
            
        default:
            // Other shifting states are handled by relay completion
            break;
    }
}

void GearboxStateMachine::updateWaitingState() {
    // Handle all clutch waiting states the same way
    if (currentState == WAITING_FOR_CLUTCH_NEUTRAL_DOWN || 
        currentState == WAITING_FOR_CLUTCH_NEUTRAL_UP ||
        currentState == WAITING_FOR_CLUTCH_SHIFT_DOWN ||
        currentState == WAITING_FOR_CLUTCH_SHIFT_UP) {
        
        if (clutchPulled) {
            processEvent(EVENT_CLUTCH_PULLED);
        } else if (getStateElapsedTime() >= CLUTCH_WAIT_TIMEOUT_MS) {
            processEvent(EVENT_TIMEOUT);
        }
    }
}

void GearboxStateMachine::updateErrorState() {
    // RACING: Immediate recovery from error states
    processEvent(EVENT_TIMEOUT);  // Return to idle immediately
}

void GearboxStateMachine::updateManualClutchControl() {
    // Manual clutch control is now handled in the main loop
    // This state is used as a placeholder and will return to idle quickly
    processEvent(EVENT_TIMEOUT);
}

void GearboxStateMachine::exitShiftingState() {
    // Clean up any shifting operations
    Serial.println("Exiting shift state: " + getStateName(currentState));
}

void GearboxStateMachine::activateRelay(int pin, int duration) {
    digitalWrite(pin, RELAY_ON);
    relayActive = true;
    relayStartTime = millis();
    relayDuration = duration;
    activeRelayPin = pin;
    
    Serial.println("Relay activated on pin " + String(pin) + " for " + String(duration) + "ms");
}

void GearboxStateMachine::deactivateRelay() {
    if (relayActive) {
        digitalWrite(activeRelayPin, RELAY_OFF);
        relayActive = false;
        Serial.println("Relay deactivated on pin " + String(activeRelayPin));
    }
}

void GearboxStateMachine::updateRelayControl() {
    if (relayActive) {
        if (millis() - relayStartTime >= relayDuration) {
            deactivateRelay();
            processEvent(EVENT_RELAY_FINISHED);
        }
    }
}

void GearboxStateMachine::engageClutch() {
    clutchServo->write(clutchEngagePos);
    Serial.println("Clutch engaged");
}

void GearboxStateMachine::releaseClutch() {
    clutchServo->write(clutchIdlePos);
    Serial.println("Clutch released");
}

void GearboxStateMachine::checkTimeouts() {
    // Check for shift timeout in any shifting state
    if (isShiftingState(currentState) && getStateElapsedTime() >= STATE_SHIFT_TIMEOUT_MS) {
        Serial.println("Shift timeout detected");
        transitionToState(ERROR_SHIFT_TIMEOUT);
    }
}

String GearboxStateMachine::getStateName(GearboxState state) const {
    switch (state) {
        case IDLE_NEUTRAL: return "IDLE_NEUTRAL";
        case IDLE_GEAR_1: return "IDLE_GEAR_1";
        case IDLE_GEAR_2: return "IDLE_GEAR_2";
        case IDLE_GEAR_3: return "IDLE_GEAR_3";
        case IDLE_GEAR_4: return "IDLE_GEAR_4";
        case IDLE_GEAR_5: return "IDLE_GEAR_5";
        case IDLE_GEAR_6: return "IDLE_GEAR_6";
        case NEUTRAL_DOWN_SHIFTING: return "NEUTRAL_DOWN_SHIFTING";
        case NEUTRAL_UP_SHIFTING: return "NEUTRAL_UP_SHIFTING";
        case UPSHIFTING: return "UPSHIFTING";
        case DOWNSHIFT_CLUTCH_ENGAGING: return "DOWNSHIFT_CLUTCH_ENGAGING";
        case DOWNSHIFT_CLUTCH_ENGAGED: return "DOWNSHIFT_CLUTCH_ENGAGED";
        case DOWNSHIFT_SHIFTING: return "DOWNSHIFT_SHIFTING";
        case MANUAL_CLUTCH_CONTROL: return "MANUAL_CLUTCH_CONTROL";
        case WAITING_FOR_CLUTCH_NEUTRAL_DOWN: return "WAITING_FOR_CLUTCH_NEUTRAL_DOWN";
        case WAITING_FOR_CLUTCH_NEUTRAL_UP: return "WAITING_FOR_CLUTCH_NEUTRAL_UP";
        case WAITING_FOR_CLUTCH_SHIFT_DOWN: return "WAITING_FOR_CLUTCH_SHIFT_DOWN";
        case WAITING_FOR_CLUTCH_SHIFT_UP: return "WAITING_FOR_CLUTCH_SHIFT_UP";
        case ERROR_SHIFT_TIMEOUT: return "ERROR_SHIFT_TIMEOUT";
        case ERROR_SENSOR_DISCONNECTED: return "ERROR_SENSOR_DISCONNECTED";
        default: return "UNKNOWN_STATE";
    }
}

bool GearboxStateMachine::isShifting() const {
    return isShiftingState(currentState) || isWaitingForClutch();
}

bool GearboxStateMachine::isIdle() const {
    return isIdleState(currentState);
}

bool GearboxStateMachine::canAcceptShiftCommand() const {
    return isIdleState(currentState);
}

bool GearboxStateMachine::isShiftInProgress() const {
    return isShifting();
}

void GearboxStateMachine::setCurrentGear(int gear) {
    if (gear >= 0 && gear <= 6 && gear != currentGear) {
        int oldGear = currentGear;
        currentGear = gear;
        
        Serial.println("Gear changed from " + String(oldGear) + " to " + String(gear));
        
        // Process gear change event to update state if needed
        processEvent(EVENT_GEAR_CHANGED);
        
        // Notify shift logger
        if (shiftLogger) {
            shiftLogger->onGearChanged(gear, oldGear);
        }
    }
}

String GearboxStateMachine::getCurrentGearName() const {
    if (currentGear == 0) return "N";
    if (currentGear >= 1 && currentGear <= 6) return String(currentGear);
    return "?";
}

GearboxState GearboxStateMachine::getIdleStateForGear(int gear) const {
    switch (gear) {
        case 0: return IDLE_NEUTRAL;
        case 1: return IDLE_GEAR_1;
        case 2: return IDLE_GEAR_2;
        case 3: return IDLE_GEAR_3;
        case 4: return IDLE_GEAR_4;
        case 5: return IDLE_GEAR_5;
        case 6: return IDLE_GEAR_6;
        default: return IDLE_NEUTRAL;
    }
}

int GearboxStateMachine::getGearForIdleState(GearboxState state) const {
    switch (state) {
        case IDLE_NEUTRAL: return 0;
        case IDLE_GEAR_1: return 1;
        case IDLE_GEAR_2: return 2;
        case IDLE_GEAR_3: return 3;
        case IDLE_GEAR_4: return 4;
        case IDLE_GEAR_5: return 5;
        case IDLE_GEAR_6: return 6;
        default: return 0;
    }
}

bool GearboxStateMachine::isIdleState(GearboxState state) const {
    return state >= IDLE_NEUTRAL && state <= IDLE_GEAR_6;
}

bool GearboxStateMachine::isShiftingState(GearboxState state) const {
    return state >= NEUTRAL_DOWN_SHIFTING && state <= DOWNSHIFT_SHIFTING;
}

void GearboxStateMachine::printStateInfo() const {
    Serial.println("=== GEARBOX STATE MACHINE STATUS ===");
    Serial.println("Current State: " + getStateName(currentState));
    Serial.println("Current Gear: " + getCurrentGearName());
    Serial.println("State Time: " + String(getStateElapsedTime()) + "ms");
    Serial.println("Is Shifting: " + String(isShifting() ? "YES" : "NO"));
    Serial.println("Can Accept Commands: " + String(canAcceptShiftCommand() ? "YES" : "NO"));
    Serial.println("Relay Active: " + String(relayActive ? "YES" : "NO"));
    Serial.println("Clutch Pulled: " + String(clutchPulled ? "YES" : "NO"));
    Serial.println("=====================================");
}

// end of code