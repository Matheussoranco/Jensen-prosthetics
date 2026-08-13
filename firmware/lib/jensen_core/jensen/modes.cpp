#include "jensen/modes.h"

namespace jensen {

const char* controlModeName(ControlMode mode) {
    switch (mode) {
        case ControlMode::Boot:          return "boot";
        case ControlMode::SelfTest:      return "selftest";
        case ControlMode::Idle:          return "idle";
        case ControlMode::Active:        return "active";
        case ControlMode::Hold:          return "hold";
        case ControlMode::Calibrating:   return "calibrating";
        case ControlMode::Fault:         return "fault";
        case ControlMode::SafeTorqueOff: return "safe_torque_off";
        default:                         return "?";
    }
}

void ModeMachine::reset() {
    mode_ = ControlMode::Boot;
    previous_ = ControlMode::Boot;
    beforeCalibration_ = ControlMode::Active;
    changed_ = false;
    anyFault_ = false;
    severeFault_ = false;
    userParked_ = false;
    idleTimer_ = 0.0f;
}

void ModeMachine::transition(ControlMode next) {
    if (next == mode_) {
        changed_ = false;
        return;
    }
    previous_ = mode_;
    mode_ = next;
    changed_ = true;
    if (next == ControlMode::Active || next == ControlMode::Hold) idleTimer_ = 0.0f;
}

void ModeMachine::setFaults(bool anyFault, bool severeFault) {
    anyFault_ = anyFault;
    severeFault_ = severeFault;

    if (severeFault_) {
        transition(ControlMode::SafeTorqueOff);
        return;
    }

    // A severe fault that clears should hand control back — unless the user
    // parked the hand deliberately, in which case it stays parked.
    if (mode_ == ControlMode::SafeTorqueOff && !userParked_) {
        transition(anyFault_ ? ControlMode::Fault : ControlMode::Active);
        return;
    }

    if (mode_ == ControlMode::Fault && !anyFault_) {
        transition(ControlMode::Active);
    } else if (anyFault_ && (mode_ == ControlMode::Active || mode_ == ControlMode::Idle)) {
        transition(ControlMode::Fault);
    }
}

void ModeMachine::dispatch(ButtonEvent event) {
    changed_ = false;
    switch (event) {
        case ButtonEvent::VeryLongPress:
            // The only control that works in every state — the user must
            // always be able to make the hand go limp.
            if (mode_ == ControlMode::SafeTorqueOff) {
                if (severeFault_) return;  // cannot re-energise into a severe fault
                userParked_ = false;
                transition(anyFault_ ? ControlMode::Fault : ControlMode::Active);
            } else {
                userParked_ = true;
                transition(ControlMode::SafeTorqueOff);
            }
            return;

        case ButtonEvent::LongPress:
            if (mode_ == ControlMode::Calibrating) {
                transition(beforeCalibration_);
            } else if (mode_ == ControlMode::Active || mode_ == ControlMode::Idle ||
                       mode_ == ControlMode::Hold || mode_ == ControlMode::Fault) {
                beforeCalibration_ = (mode_ == ControlMode::Hold) ? ControlMode::Active : mode_;
                transition(ControlMode::Calibrating);
            }
            return;

        case ButtonEvent::DoubleClick:
            if (mode_ == ControlMode::Hold) {
                transition(ControlMode::Active);
            } else if (mode_ == ControlMode::Active || mode_ == ControlMode::Idle ||
                       mode_ == ControlMode::Fault) {
                transition(ControlMode::Hold);
            }
            return;

        case ButtonEvent::Click:
            // Grip selection is the controller's business; the click only
            // wakes the hand if it had dozed off.
            if (mode_ == ControlMode::Idle) transition(ControlMode::Active);
            return;

        case ButtonEvent::None:
        default:
            return;
    }
}

void ModeMachine::dispatch(SystemEvent event) {
    changed_ = false;
    switch (event) {
        case SystemEvent::SelfTestPassed:
            if (mode_ == ControlMode::Boot || mode_ == ControlMode::SelfTest) {
                transition(anyFault_ ? ControlMode::Fault : ControlMode::Active);
            }
            return;

        case SystemEvent::SelfTestFailed:
            transition(ControlMode::SafeTorqueOff);
            return;

        case SystemEvent::CalibrationComplete:
        case SystemEvent::CalibrationAborted:
            if (mode_ == ControlMode::Calibrating) {
                transition(severeFault_ ? ControlMode::SafeTorqueOff : beforeCalibration_);
            }
            return;

        case SystemEvent::CoContraction:
            if (mode_ == ControlMode::Idle) transition(ControlMode::Active);
            return;

        case SystemEvent::None:
        default:
            return;
    }
}

void ModeMachine::update(float dt, bool active) {
    changed_ = false;
    if (mode_ == ControlMode::Boot) {
        transition(ControlMode::SelfTest);
        return;
    }

    // Idle only applies to the two states where the user is nominally in
    // control and simply is not doing anything.
    if (mode_ != ControlMode::Active && mode_ != ControlMode::Idle) {
        idleTimer_ = 0.0f;
        return;
    }

    if (active) {
        idleTimer_ = 0.0f;
        if (mode_ == ControlMode::Idle) transition(ControlMode::Active);
        return;
    }

    if (mode_ == ControlMode::Active) {
        idleTimer_ += dt;
        if (idleTimeout_ > 0.0f && idleTimer_ >= idleTimeout_) {
            transition(ControlMode::Idle);
        }
    }
}

bool ModeMachine::drivesServos() const {
    switch (mode_) {
        case ControlMode::Active:
        case ControlMode::Hold:
        case ControlMode::Calibrating:
        case ControlMode::Fault:
        case ControlMode::Idle:
            return true;
        case ControlMode::Boot:
        case ControlMode::SelfTest:
        case ControlMode::SafeTorqueOff:
        default:
            return false;
    }
}

bool ModeMachine::acceptsUserInput() const {
    return mode_ == ControlMode::Active || mode_ == ControlMode::Idle ||
           mode_ == ControlMode::Fault;
}

}  // namespace jensen
