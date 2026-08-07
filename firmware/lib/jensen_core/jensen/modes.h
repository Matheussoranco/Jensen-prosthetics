// Jensen Hand — operating mode state machine.
//
// Kept free of side effects on purpose: this class decides *what state the
// hand is in*, and the controller decides what that state means for the
// servos. That split is what makes the mode logic testable without a hand
// attached, and it stops the "what happens when the battery dies during
// calibration" class of question from being answered differently in three
// places.
#pragma once

#include <cstdint>

namespace jensen {

enum class ControlMode : uint8_t {
    Boot,          ///< powered, nothing initialised yet
    SelfTest,      ///< checking the servo bus and sensor ranges
    Idle,          ///< no myoelectric activity; outputs relaxed to save power
    Active,        ///< normal proportional control
    Hold,          ///< closure frozen; the user is carrying something
    Calibrating,   ///< running a guided calibration sequence
    Fault,         ///< degraded but still driving (e.g. low battery)
    SafeTorqueOff, ///< servo pulses removed; the hand is limp
};

const char* controlModeName(ControlMode mode);

/// Physical button gestures produced by the HAL.
enum class ButtonEvent : uint8_t {
    None = 0,
    Click,         ///< next grip pattern
    DoubleClick,   ///< toggle hold
    LongPress,     ///< start calibration
    VeryLongPress, ///< toggle safe-torque-off (park the hand)
};

/// Non-button inputs that can also move the machine.
enum class SystemEvent : uint8_t {
    None = 0,
    SelfTestPassed,
    SelfTestFailed,
    CalibrationComplete,
    CalibrationAborted,
    CoContraction, ///< the two-electrode grip-change gesture
};

class ModeMachine {
public:
    void reset();

    ControlMode mode() const { return mode_; }
    ControlMode previous() const { return previous_; }
    /// True for exactly one call after a transition.
    bool changed() const { return changed_; }

    /// Seconds of no myoelectric activity before dropping into Idle.
    void setIdleTimeout(float seconds) { idleTimeout_ = seconds; }

    /// Severe faults force SafeTorqueOff and hold it until they clear;
    /// non-severe faults show as Fault but keep the hand usable.
    void setFaults(bool anyFault, bool severeFault);

    void dispatch(ButtonEvent event);
    void dispatch(SystemEvent event);

    /// `active` is true whenever the user is producing usable myoelectric
    /// activity or the hand is not fully open.
    void update(float dt, bool active);

    /// Convenience predicates used by the controller.
    bool drivesServos() const;
    bool acceptsUserInput() const;

private:
    void transition(ControlMode next);

    ControlMode mode_ = ControlMode::Boot;
    ControlMode previous_ = ControlMode::Boot;
    ControlMode beforeCalibration_ = ControlMode::Active;
    bool changed_ = false;

    bool anyFault_ = false;
    bool severeFault_ = false;
    /// Set when the user parks the hand by hand, so a fault clearing does not
    /// silently re-energise the servos.
    bool userParked_ = false;

    float idleTimeout_ = 20.0f;
    float idleTimer_ = 0.0f;
};

}  // namespace jensen
