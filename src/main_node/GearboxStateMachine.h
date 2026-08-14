#ifndef GEARBOX_STATE_MACHINE_H
#define GEARBOX_STATE_MACHINE_H

#include <Arduino.h>
#include "SimpleServo.h"

// Forward declarations
class ShiftLogger;
class RPM;

// State definitions
enum GearboxState {
    // Idle states - one for each gear (must stay contiguous — see isIdleState)
    IDLE_NEUTRAL = 0,
    IDLE_GEAR_1,
    IDLE_GEAR_2,
    IDLE_GEAR_3,
    IDLE_GEAR_4,
    IDLE_GEAR_5,
    IDLE_GEAR_6,

    // Shifting states (must stay contiguous — see isShiftingState)
    NEUTRAL_DOWN_SHIFTING,
    NEUTRAL_UP_SHIFTING,
    UPSHIFTING,
    DOWNSHIFT_CLUTCH_ENGAGING,
    DOWNSHIFT_SHIFTING,
    // Between stacked downshifts the clutch releases only as far as the bite point
    // rather than all the way to idle, skipping the dead travel. Counts as a shifting
    // state so no unrelated shift command is accepted mid-stack.
    DOWNSHIFT_PARTIAL_RELEASE,

    // Wait states - clutch interlock for shifts out of neutral
    WAITING_FOR_CLUTCH_SHIFT_DOWN,
    WAITING_FOR_CLUTCH_SHIFT_UP,

    // Error states
    ERROR_SHIFT_TIMEOUT,

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
    EVENT_GEAR_CHANGED,
    EVENT_TIMEOUT,

    EVENT_COUNT
};

// State transition structure
struct StateTransition {
    GearboxState fromState;
    GearboxEvent event;
    GearboxState toState;
};

class GearboxStateMachine {
private:
    // Current state
    GearboxState currentState;

    // Timing for state operations
    unsigned long stateStartTime;

    // Configuration
    int neutralDownMs;
    int neutralUpMs;
    int shiftDownMs;
    int shiftUpMs;
    int clutchIdlePos;
    int clutchFullyPull;

    // Hardware references
    ShiftLogger* shiftLogger;
    RPM* rpmSensor;
    SimpleServo* clutchServo;

    // Shift timing (mirrors old relay timing — completion returns to idle)
    bool relayActive;
    unsigned long lastReleaseStepMs;   // rate limiter for the stacked partial release
    unsigned long relayStartTime;
    int relayDuration;
    bool activeShiftIsUp;

    // Clutch state (fed from main loop via setClutchPulled)
    bool clutchPulled;

    // Current gear tracking
    int currentGear;     // 0=N, 1-6=gears — confirmed by CAN
    int expectedGear;    // gear we expect after the current shift relay fires
    int targetGear;      // stacked downshift target (0 = no stack pending)

    // Timeouts
    static const unsigned long STATE_SHIFT_TIMEOUT_MS = 500;
    // Driver is being waited on to pull the clutch by hand (neutral paths).
    static const unsigned long CLUTCH_WAIT_TIMEOUT_MS = 200;

    // Servo is being waited on to physically pull the clutch past the trigger voltage
    // (downshift path). Must comfortably exceed real servo travel — a sweep from
    // clutchIdlePos to clutchFullyPull is typically 150-300 ms, so 200 ms would expire
    // on healthy shifts and bypass the gate entirely. On expiry the shift is ABORTED,
    // never sent: an unconfirmed clutch means shifting against drive.
    static const unsigned long DOWNSHIFT_SERVO_TIMEOUT_MS = 500;

    // Cap on the partial release between stacked downshifts. On expiry the clutch
    // completes a normal full release rather than aborting — a missed bite point costs
    // speed, not the shift. Also bounds the wait if CAN never confirms the gear.
    static const unsigned long STACK_RELEASE_TIMEOUT_MS = 300;

    // Release stepping. 2 deg every 4 ms is ~500 deg/s, near the servo's own speed, so
    // the walk-out does not itself become the bottleneck. Tune together.
    static const unsigned long STACK_RELEASE_STEP_MS  = 4;
    static constexpr float     STACK_RELEASE_STEP_DEG = 2.0f;

public:
    GearboxStateMachine()
        : currentState(IDLE_NEUTRAL),
          stateStartTime(0),
          neutralDownMs(40), neutralUpMs(40), shiftDownMs(150), shiftUpMs(150),
          // Placeholders until setConfiguration(); inside CLUTCH_SERVO_MIN/MAX (CalConfig.h)
          clutchIdlePos(42), clutchFullyPull(137),
          shiftLogger(nullptr), rpmSensor(nullptr), clutchServo(nullptr),
          relayActive(false), lastReleaseStepMs(0),
          relayStartTime(0), relayDuration(0), activeShiftIsUp(false),
          clutchPulled(false),
          currentGear(0), expectedGear(0), targetGear(0) {}

    // Initialization
    void begin(ShiftLogger* logger, RPM* rpm, SimpleServo* servo);
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

    // Status for web interface
    bool isWaitingForClutch() const {
        return currentState == WAITING_FOR_CLUTCH_SHIFT_DOWN ||
               currentState == WAITING_FOR_CLUTCH_SHIFT_UP;
    }

    // Debug
    void printStateInfo() const;

private:
    // State transition logic
    bool transitionToState(GearboxState newState);
    void executeStateEntry();
    void executeStateUpdate();

    // State handlers
    void enterIdleState();
    void enterShiftingState();
    void enterWaitingState();
    void enterErrorState();

    void updateDownshiftClutchWait();
    void enterPartialReleaseState();
    void updatePartialRelease();
    void updateWaitingState();
    void updateErrorState();

    // Shift control (sends CAN command + tracks completion timing)
    void activateShift(bool isUpshift, int duration, uint16_t ignCutMs = 0, uint8_t targetGear = 0xFF);
    void deactivateShift();
    void engageClutch();
    void releaseClutch();
    void updateRelayControl();

    // Stacked downshift + logging helpers
    void clearShiftStack();
    void logShiftStart(int fromGear, int toGear, uint8_t shiftType);

    // Utility functions
    GearboxState getIdleStateForGear(int gear) const;
    bool isIdleState(GearboxState state) const;
    bool isShiftingState(GearboxState state) const;

    // Timeout handling
    void checkTimeouts();
    unsigned long getStateElapsedTime() const { return millis() - stateStartTime; }
};

#endif

// end of code
