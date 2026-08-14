// GearboxStateMachine.cpp - racing sequential gearbox state machine

#include "GearboxStateMachine.h"
#include "MainCan.h"
#include "ShiftLogger.h"
#include "RPM.h"

// Clutch servo feedback, owned by T89_gearbox_206.cpp. clutchVoltage is the live
// position from the servo's pot via the ADS1115; clutchDisengageV is the downshift
// trigger threshold. Feedback is electrically inverted — pulling drives it DOWN.
extern float clutchVoltage;
extern float clutchDisengageV;
extern float clutchJustEngagedV;   // bite point — return target between stacked shifts

// External functions implemented in T89_gearbox_206.cpp
extern void displayShiftLetter(char letter);
extern void canSendShiftUp(uint16_t shiftMs, uint16_t ignCutMs, uint8_t targetGear = GEAR_UNKNOWN);
extern void canSendShiftDown(uint16_t shiftMs, uint8_t targetGear = GEAR_UNKNOWN);
extern void canSendShiftStack(uint8_t targetGear);

// State transition table for motorcycle-style sequential gearbox
// Pattern: 1st ↔ N ↔ 2nd → 3rd → 4th → 5th → 6th
// Shift completion is not in this table: updateRelayControl() transitions
// straight to the idle state for the expected gear when the timing expires.
static const StateTransition stateTransitions[] = {
    // FROM NEUTRAL - between 1st and 2nd, full shift either way (clutch required)
    {IDLE_NEUTRAL, EVENT_SHIFT_DOWN_PRESSED, WAITING_FOR_CLUTCH_SHIFT_DOWN},   // N → 1st
    {IDLE_NEUTRAL, EVENT_SHIFT_UP_PRESSED,   WAITING_FOR_CLUTCH_SHIFT_UP},     // N → 2nd

    // FROM 1ST GEAR
    {IDLE_GEAR_1, EVENT_NEUTRAL_UP_PRESSED, NEUTRAL_UP_SHIFTING},              // 1st → N (no clutch)
    {IDLE_GEAR_1, EVENT_SHIFT_UP_PRESSED,   UPSHIFTING},                       // 1st → 2nd (ignition cut)

    // FROM 2ND GEAR
    {IDLE_GEAR_2, EVENT_NEUTRAL_DOWN_PRESSED, NEUTRAL_DOWN_SHIFTING},          // 2nd → N (no clutch)
    {IDLE_GEAR_2, EVENT_SHIFT_DOWN_PRESSED,   DOWNSHIFT_CLUTCH_ENGAGING},      // 2nd → 1st (servo, skip N)
    {IDLE_GEAR_2, EVENT_SHIFT_UP_PRESSED,     UPSHIFTING},                     // 2nd → 3rd (ignition cut)

    // FROM 3RD-5TH GEAR - shift up/down only, no neutral access
    {IDLE_GEAR_3, EVENT_SHIFT_DOWN_PRESSED, DOWNSHIFT_CLUTCH_ENGAGING},        // 3rd → 2nd (servo)
    {IDLE_GEAR_3, EVENT_SHIFT_UP_PRESSED,   UPSHIFTING},                       // 3rd → 4th (ignition cut)
    {IDLE_GEAR_4, EVENT_SHIFT_DOWN_PRESSED, DOWNSHIFT_CLUTCH_ENGAGING},        // 4th → 3rd (servo)
    {IDLE_GEAR_4, EVENT_SHIFT_UP_PRESSED,   UPSHIFTING},                       // 4th → 5th (ignition cut)
    {IDLE_GEAR_5, EVENT_SHIFT_DOWN_PRESSED, DOWNSHIFT_CLUTCH_ENGAGING},        // 5th → 4th (servo)
    {IDLE_GEAR_5, EVENT_SHIFT_UP_PRESSED,   UPSHIFTING},                       // 5th → 6th (ignition cut)

    // FROM 6TH GEAR - top gear, shift down only
    {IDLE_GEAR_6, EVENT_SHIFT_DOWN_PRESSED, DOWNSHIFT_CLUTCH_ENGAGING},        // 6th → 5th (servo)

    // CLUTCH INTERLOCK - shifts out of neutral wait for the clutch (abort on timeout)
    {WAITING_FOR_CLUTCH_SHIFT_DOWN, EVENT_CLUTCH_PULLED, NEUTRAL_DOWN_SHIFTING},  // Use downshift relay
    {WAITING_FOR_CLUTCH_SHIFT_DOWN, EVENT_TIMEOUT,       IDLE_NEUTRAL},
    {WAITING_FOR_CLUTCH_SHIFT_UP,   EVENT_CLUTCH_PULLED, NEUTRAL_UP_SHIFTING},    // Use upshift relay
    {WAITING_FOR_CLUTCH_SHIFT_UP,   EVENT_TIMEOUT,       IDLE_NEUTRAL},

    // RACING DOWNSHIFT - relay fires the moment the clutch is pulled
    {DOWNSHIFT_CLUTCH_ENGAGING, EVENT_CLUTCH_PULLED, DOWNSHIFT_SHIFTING},
};

static const int stateTransitionCount = sizeof(stateTransitions) / sizeof(StateTransition);

void GearboxStateMachine::begin(ShiftLogger* logger, RPM* rpm, SimpleServo* servo) {
    shiftLogger = logger;
    rpmSensor = rpm;
    clutchServo = servo;

    // Initialize to idle state based on current gear
    currentState = getIdleStateForGear(currentGear);
    stateStartTime = millis();

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
    clutchFullyPull = cEngagePos;
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
    // Stacked downshift: capture paddle press any time a shift is already in progress
    if (event == EVENT_SHIFT_DOWN_PRESSED && !isIdleState(currentState)) {
        if (targetGear == 0) {
            targetGear = max(1, expectedGear - 1);  // first queued press
        } else {
            targetGear = max(1, targetGear - 1);    // additional presses
        }
        canSendShiftStack(targetGear);
        Serial.println("Stacked downshift queued, target: " + String(targetGear));
        return true;
    }

    // Shift up always cancels any pending stack
    if (event == EVENT_SHIFT_UP_PRESSED && targetGear > 0) {
        Serial.println("Stacked downshift cancelled by upshift");
        clearShiftStack();
    }

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
    for (int i = 0; i < stateTransitionCount; i++) {
        const StateTransition& trans = stateTransitions[i];
        if (trans.fromState == currentState && trans.event == event) {
            return transitionToState(trans.toState);
        }
    }

    // No valid transition found
    Serial.println("No transition for event in state " + getStateName(currentState));
    return false;
}

bool GearboxStateMachine::transitionToState(GearboxState newState) {
    if (newState == currentState) return true;

    Serial.println("State transition: " + getStateName(currentState) + " -> " + getStateName(newState));

    // Update state
    currentState = newState;
    stateStartTime = millis();

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
        case DOWNSHIFT_SHIFTING:
            enterShiftingState();
            break;

        // Deliberately NOT routed through enterShiftingState(): that recomputes
        // expectedGear from currentGear, and currentGear still holds the old gear until
        // CAN confirms. Doing so here would rewind expectedGear to the gear we just left.
        case DOWNSHIFT_TO_BITE_POINT:
            enterBitePointState();
            break;

        case WAITING_FOR_CLUTCH_SHIFT_DOWN:
        case WAITING_FOR_CLUTCH_SHIFT_UP:
            enterWaitingState();
            break;

        case ERROR_SHIFT_TIMEOUT:
            enterErrorState();
            break;

        default:
            break;
    }
}

void GearboxStateMachine::executeStateUpdate() {
    switch (currentState) {
        case DOWNSHIFT_CLUTCH_ENGAGING:
            updateDownshiftClutchWait();
            break;

        case DOWNSHIFT_TO_BITE_POINT:
            updateToBitePoint();
            break;

        case WAITING_FOR_CLUTCH_SHIFT_DOWN:
        case WAITING_FOR_CLUTCH_SHIFT_UP:
            updateWaitingState();
            break;

        case ERROR_SHIFT_TIMEOUT:
            updateErrorState();
            break;

        default:
            // Idle states are event-driven; other shifting states complete via updateRelayControl()
            break;
    }
}

void GearboxStateMachine::enterIdleState() {
    // Return the clutch to idle (engaged)
    clutchToIdle();

    // Ensure relays are off
    deactivateShift();

    Serial.println("Entered idle state for gear: " + getCurrentGearName());
}

void GearboxStateMachine::enterShiftingState() {
    Serial.println("Starting shift operation: " + getStateName(currentState));

    // Track expected gear so relay completion lands in the right idle state
    int fromGear = currentGear;
    switch (currentState) {
        case NEUTRAL_DOWN_SHIFTING:       expectedGear = (fromGear == 2) ? 0 : 1; break;
        case NEUTRAL_UP_SHIFTING:         expectedGear = (fromGear == 1) ? 0 : 2; break;
        case UPSHIFTING:                  expectedGear = fromGear + 1; break;
        case DOWNSHIFT_CLUTCH_ENGAGING:
        case DOWNSHIFT_SHIFTING:          expectedGear = fromGear - 1; break;
        default:                          expectedGear = fromGear; break;
    }

    switch (currentState) {
        case NEUTRAL_DOWN_SHIFTING:
            // Neutral Down button - downshift relay: 2nd → N (half-shift) or N → 1st (full shift)
            logShiftStart(fromGear, expectedGear, (fromGear == 2) ? 2 : 1);
            activateShift(false, (fromGear == 0) ? shiftDownMs : neutralDownMs, 0, (uint8_t)expectedGear);
            displayShiftLetter(expectedGear == 0 ? 'N' : 'D');
            break;

        case NEUTRAL_UP_SHIFTING:
            // Neutral Up button - upshift relay, no ignition cut: 1st → N (half-shift) or N → 2nd (full shift)
            logShiftStart(fromGear, expectedGear, (fromGear == 1) ? 2 : 0);
            activateShift(true, (fromGear == 0) ? shiftUpMs : neutralUpMs, 0, (uint8_t)expectedGear);
            displayShiftLetter(expectedGear == 0 ? 'N' : 'U');
            break;

        case UPSHIFTING:
            // Shift Up button - upshift with ignition cut
            if (shiftLogger) shiftLogger->startIgnitionCut();
            logShiftStart(fromGear, expectedGear, 0);
            activateShift(true, shiftUpMs, IGN_CUT_DEFAULT_MS, (uint8_t)expectedGear);
            displayShiftLetter('U');
            break;

        case DOWNSHIFT_CLUTCH_ENGAGING:
            // Shift Down button - engage clutch servo, relay fires on pull detection
            logShiftStart(fromGear, expectedGear, 1);
            clutchToMax();
            displayShiftLetter('D');
            break;

        case DOWNSHIFT_SHIFTING:
            // Clutch pulled (or timeout fallback) - send CAN shift command immediately
            activateShift(false, shiftDownMs, 0, (uint8_t)expectedGear);
            break;

        default:
            break;
    }
}

void GearboxStateMachine::enterWaitingState() {
    Serial.println("Waiting for clutch to be pulled...");
    displayShiftLetter('C');
}

void GearboxStateMachine::enterErrorState() {
    Serial.println("Entered error state: " + getStateName(currentState));

    // Ensure safe state
    deactivateShift();
    clutchToIdle();
    clearShiftStack();  // aborted shift - drop any queued downshifts
}

void GearboxStateMachine::updateDownshiftClutchWait() {
    // The servo is travelling. Only once its feedback confirms the clutch is actually
    // pulled is it safe to send the downshift — clutchDisengaged is a threshold crossing,
    // so it latches on the first sample past the trigger voltage and does not rely on
    // any exact value being sampled.
    if (clutchDisengaged) {
        processEvent(EVENT_CLUTCH_PULLED);
        return;
    }

    // Never send the shift unconfirmed: if the servo has not reached the trigger
    // voltage, the clutch may still be driving and the shift would be against load.
    // Abort instead. enterErrorState() returns the servo to idle and drops any
    // stacked downshifts; updateErrorState() then recovers to the current gear.
    if (getStateElapsedTime() >= DOWNSHIFT_SERVO_TIMEOUT_MS) {
        Serial.println("Downshift ABORTED: clutch not confirmed within " +
                       String(DOWNSHIFT_SERVO_TIMEOUT_MS) + "ms (servo/feedback fault?) - "
                       "live " + String(clutchVoltage, 3) + "V vs trigger " +
                       String(clutchDisengageV, 3) + "V");
        // No explicit failure log needed: no gear change follows, so the logger's own
        // update() records this as a failed shift when its timing window expires.
        transitionToState(ERROR_SHIFT_TIMEOUT);
    }
}

void GearboxStateMachine::enterBitePointState() {
    // Deliberately does NOT command idle. Stopping at the bite point needs an angle to
    // hold, but the bite point is defined as a voltage and there is no volts-to-degrees
    // calibration — so the command is walked out step by step in updateToBitePoint()
    // while feedback is watched, rather than commanding idle and trying to interrupt it
    // mid-travel. Servo stays where the shift left it until the first step.
    lastBiteStepMs = millis();
    displayShiftLetter('D');
    Serial.println("Stacked: releasing to bite point " + String(clutchJustEngagedV, 3) +
                   "V (from " + String(clutchVoltage, 3) + "V)");
}

void GearboxStateMachine::updateToBitePoint() {
    // Bite point reached — hold here. The next shift fires from this position the moment
    // CAN confirms the gear, so the clutch never travels the dead zone beyond it.
    if (clutchVoltage >= clutchJustEngagedV) return;

    // Not there in time: return fully to idle so a mis-set threshold or a slow
    // servo costs speed rather than the shift. Landing in an idle state also hands the
    // stack back to the original path.
    if (getStateElapsedTime() >= STACK_BITE_TIMEOUT_MS) {
        Serial.println("Stacked: bite point not reached in " +
                       String(STACK_BITE_TIMEOUT_MS) + "ms (live " +
                       String(clutchVoltage, 3) + "V vs " + String(clutchJustEngagedV, 3) +
                       "V) - returning fully to idle");
        clutchToIdle();
        transitionToState(getIdleStateForGear(expectedGear));
        return;
    }

    // Step the command toward idle. Rate-limited so the stroke stays close to the servo's
    // own speed instead of being dictated by loop frequency; a slow loop just returns
    // more gently, which errs toward less engagement.
    unsigned long now = millis();
    if (now - lastBiteStepMs < STACK_BITE_STEP_MS) return;
    lastBiteStepMs = now;

    float angle  = clutchServo->getAngle();
    float target = (float)clutchIdlePos;
    if (angle > target)      angle = max(target, angle - STACK_BITE_STEP_DEG);
    else if (angle < target) angle = min(target, angle + STACK_BITE_STEP_DEG);
    clutchServo->writeFloat(angle);
}

void GearboxStateMachine::updateWaitingState() {
    if (clutchDisengaged) {
        processEvent(EVENT_CLUTCH_PULLED);
    } else if (getStateElapsedTime() >= CLUTCH_WAIT_TIMEOUT_MS) {
        clearShiftStack();  // aborting without shifting - drop any queued downshifts
        processEvent(EVENT_TIMEOUT);
    }
}

void GearboxStateMachine::updateErrorState() {
    // RACING: recover immediately, back to the idle state for the actual current gear
    transitionToState(getIdleStateForGear(currentGear));
}

void GearboxStateMachine::activateShift(bool isUpshift, int duration, uint16_t ignCutMs, uint8_t targetGear) {
    if (isUpshift) {
        canSendShiftUp((uint16_t)duration, ignCutMs, targetGear);
    } else {
        canSendShiftDown((uint16_t)duration, targetGear);
    }
    relayActive     = true;
    relayStartTime  = millis();
    relayDuration   = duration;
    activeShiftIsUp = isUpshift;

    Serial.println(String(isUpshift ? "Upshift" : "Downshift") + " CAN sent, duration=" + String(duration) + "ms");
}

void GearboxStateMachine::deactivateShift() {
    if (relayActive) {
        relayActive = false;
        Serial.println(String(activeShiftIsUp ? "Upshift" : "Downshift") + " timing complete");
    }
}

void GearboxStateMachine::updateRelayControl() {
    if (relayActive) {
        if (millis() - relayStartTime >= relayDuration) {
            deactivateShift();

            // With a stacked downshift still pending, hold at the bite point instead of
            // returning to idle: entering an idle state calls clutchToIdle() and the
            // next shift would then have to pull back through the whole dead travel.
            if (!activeShiftIsUp && targetGear > 0 && expectedGear > targetGear) {
                transitionToState(DOWNSHIFT_TO_BITE_POINT);
                return;
            }

            // Go directly to expected gear idle state — don't wait for CAN confirmation,
            // which arrives later and would leave a gap in IDLE_NEUTRAL where the next
            // shift press triggers WAITING_FOR_CLUTCH instead of a direct shift.
            transitionToState(getIdleStateForGear(expectedGear));
        }
    }
}

void GearboxStateMachine::clutchToMax() {
    clutchServo->write(clutchFullyPull);
    Serial.println("Clutch engaged");
}

void GearboxStateMachine::clutchToIdle() {
    clutchServo->write(clutchIdlePos);
    Serial.println("Clutch to idle (engaged)");
}

void GearboxStateMachine::clearShiftStack() {
    if (targetGear > 0) {
        targetGear = 0;
        canSendShiftStack(0);
    }
}

void GearboxStateMachine::logShiftStart(int fromGear, int toGear, uint8_t shiftType) {
    if (shiftLogger) {
        shiftLogger->startShiftTiming(fromGear, toGear, rpmSensor ? rpmSensor->getRpm() : 0, shiftType);
    }
}

void GearboxStateMachine::checkTimeouts() {
    // These two run their own timeouts (updateDownshiftClutchWait / updateToBitePoint)
    // and must not also be caught by the generic shift timeout below.
    if (currentState == DOWNSHIFT_CLUTCH_ENGAGING) return;
    if (currentState == DOWNSHIFT_TO_BITE_POINT) return;

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
        case DOWNSHIFT_SHIFTING: return "DOWNSHIFT_SHIFTING";
        case DOWNSHIFT_TO_BITE_POINT: return "DOWNSHIFT_TO_BITE_POINT";
        case WAITING_FOR_CLUTCH_SHIFT_DOWN: return "WAITING_FOR_CLUTCH_SHIFT_DOWN";
        case WAITING_FOR_CLUTCH_SHIFT_UP: return "WAITING_FOR_CLUTCH_SHIFT_UP";
        case ERROR_SHIFT_TIMEOUT: return "ERROR_SHIFT_TIMEOUT";
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

        // Stacked downshift: CAN confirmed the gear — fire next or clear
        if (targetGear > 0) {
            if (currentGear > targetGear && currentState == DOWNSHIFT_TO_BITE_POINT) {
                // Holding at the bite point. currentGear has just been confirmed, so
                // transition straight in — processEvent() must NOT be used here: outside
                // an idle state it treats a down press as another stack request.
                Serial.println("Stacked: gear " + String(currentGear) + " confirmed at bite point (" +
                               String(clutchVoltage, 3) + "V), firing toward " + String(targetGear));
                transitionToState(DOWNSHIFT_CLUTCH_ENGAGING);
            } else if (currentGear > targetGear && isIdleState(currentState)) {
                Serial.println("Stacked: gear " + String(currentGear) + " confirmed, firing toward " + String(targetGear));
                processEvent(EVENT_SHIFT_DOWN_PRESSED);
            } else {
                // Reached target or shift failed — clear stack
                clearShiftStack();
            }
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

bool GearboxStateMachine::isIdleState(GearboxState state) const {
    return state >= IDLE_NEUTRAL && state <= IDLE_GEAR_6;
}

bool GearboxStateMachine::isShiftingState(GearboxState state) const {
    return state >= NEUTRAL_DOWN_SHIFTING && state <= DOWNSHIFT_TO_BITE_POINT;
}

void GearboxStateMachine::printStateInfo() const {
    Serial.println("=== GEARBOX STATE MACHINE STATUS ===");
    Serial.println("Current State: " + getStateName(currentState));
    Serial.println("Current Gear: " + getCurrentGearName());
    Serial.println("State Time: " + String(getStateElapsedTime()) + "ms");
    Serial.println("Is Shifting: " + String(isShifting() ? "YES" : "NO"));
    Serial.println("Can Accept Commands: " + String(canAcceptShiftCommand() ? "YES" : "NO"));
    Serial.println("Relay Active: " + String(relayActive ? "YES" : "NO"));
    Serial.println("Clutch Pulled: " + String(clutchDisengaged ? "YES" : "NO"));
    Serial.println("=====================================");
}

// end of code
