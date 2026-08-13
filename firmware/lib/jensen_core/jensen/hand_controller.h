// Jensen Hand — the control policy.
//
// This is the whole behaviour of the prosthesis: sensors in, servo pulses out,
// with no reference to any peripheral. The ESP32 layer samples ADCs, fills a
// `HandInputs`, calls `update()` once per control tick, and applies the
// resulting `HandOutputs`. That boundary is what lets the entire policy — the
// drive law, the force loop, mode transitions, calibration sequencing — be
// exercised on a host against synthetic inputs.
//
// Signal chain
// ------------
//   EMG counts -> EmgChannel (despike, envelope, calibrated normalise)
//              -> ProportionalDrive (velocity, co-contraction gesture)
//              -> integrate into aperture   [HACKberry's position integrator]
//              -> GripLibrary pose          [which digits move, and how far]
//              -> DigitController per digit (slew limit, endpoint mapping)
//              -> pulse widths
//
//   FSR counts -> ForceSensor (newtons, contact, slip)
//              -> GripForceController -> closure limit applied to the digits
#pragma once

#include <cstdint>

#include "jensen/calibration.h"
#include "jensen/current.h"
#include "jensen/digit.h"
#include "jensen/emg.h"
#include "jensen/force.h"
#include "jensen/grip.h"
#include "jensen/hand_config.h"
#include "jensen/hand_state.h"
#include "jensen/modes.h"
#include "jensen/pattern.h"
#include "jensen/safety.h"

namespace jensen {

class HandController {
public:
    /// Applies a configuration and puts the hand into `SelfTest`.
    void begin(const HandConfig& config);

    /// Applies a configuration to a running hand. Calibration and tuning
    /// changes take effect immediately; motion state is preserved.
    void applyConfig(const HandConfig& config);
    const HandConfig& config() const { return config_; }

    /// One control iteration. `dt` is the measured interval in seconds.
    const HandOutputs& update(const HandInputs& inputs, float dt);

    const HandOutputs& outputs() const { return outputs_; }
    const HandTelemetry& telemetry() const { return telemetry_; }

    // --- Commands ------------------------------------------------------------
    // Issued by the serial CLI and the Nextion display. All are safe to call
    // between control iterations.

    void selectGrip(GripId grip);
    void nextGrip();
    void previousGrip();
    GripId grip() const { return grips_.current(); }

    void setHold(bool hold);
    void park(bool parked);

    void startEmgCalibration();
    void abortCalibration();
    bool calibrating() const { return emgRunner_.running(); }

    /// Direct aperture command, bypassing the drive law. Used by the display's
    /// bench-test page and by `hand open` / `hand close` on the CLI; ignored
    /// while the user is producing myoelectric activity, so it can never fight
    /// the wearer for control.
    void commandAperture(float aperture);
    void clearApertureCommand() { apertureCommandActive_ = false; }

    /// Wrist rotator, when one is fitted. 0.5 is neutral.
    void setWristTarget(float closure);

    bool startPatternTraining(GripId grip);
    void cancelPatternTraining() { pattern_.cancelTraining(); }
    const PatternRecognizer& patternRecognizer() const { return pattern_; }

    /// Captures the current fingertip readings as the unloaded zero and folds
    /// the result back into the configuration. FSR zero drifts with
    /// temperature and with how much the printed fingertip pad has taken a
    /// set, so this is expected to be re-run occasionally. The hand must not
    /// be touching anything when it is called.
    void tareForceSensors();

    /// True once after a calibration completes, telling the application to
    /// persist `config()`.
    bool consumeConfigDirty();

    const ModeMachine& modes() const { return modes_; }
    const SafetySupervisor& safety() const { return safety_; }
    const EmgCalibrationRunner& calibrationRunner() const { return emgRunner_; }
    const BatteryMonitor& battery() const { return battery_; }
    const LoopMonitor& loop() const { return loop_; }

private:
    void configureFromConfig();
    void resetMotion();

    void updateSensors(const HandInputs& inputs, float dt);
    void updateFaults(const HandInputs& inputs, float dt);
    void updateCalibration(const HandInputs& inputs, float dt);
    void updateDrive(float dt);
    void updateGripBlend(float dt);
    void applyPose(float dt);
    void updateOutputs();
    void updateTelemetry(float dt);

    void requestHaptic(HapticCue cue);
    static uint8_t hapticPriority(HapticCue cue);

    HandConfig config_ = HandConfig::defaults(HandSide::Right);

    EmgChannel flexor_{};
    EmgChannel extensor_{};
    EmgChannel aux_[kPatternSiteCount - 2]{};
    ProportionalDrive drive_{};
    PatternRecognizer pattern_{};
    ForceSensor force_[kGraspDigitCount]{};
    GripForceController gripForce_{};
    DigitController digits_[kDigitCount]{};
    MotorCurrentGuard current_[kDigitCount]{};
    GripSelector grips_{};

    ModeMachine modes_{};
    SafetySupervisor safety_{};
    BatteryMonitor battery_{};
    HoldGuard holdGuard_{};
    LoopMonitor loop_{};
    EmgCalibrationRunner emgRunner_{};

    HandOutputs outputs_{};
    HandTelemetry telemetry_{};

    float aperture_ = 0.0f;
    float wristTarget_ = 0.5f;
    float commandedAperture_ = 0.0f;
    bool apertureCommandActive_ = false;

    /// Per-digit closure ceilings from the adaptive-grasp loop. A finger that
    /// reaches its force target stops while the others keep closing, which is
    /// what lets the hand conform to an object that is not symmetric.
    float digitLimit_[kGraspDigitCount] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

    /// Grip morphing state, so a grip change eases the digits across instead
    /// of stepping them.
    GripId blendFrom_ = GripId::Power;
    float blendTimer_ = 0.0f;
    float blendDuration_ = 0.30f;

    /// Latest LM393 comparator readings, carried through to telemetry.
    bool contactDigital_[kGraspDigitCount] = {false, false, false, false, false, false};
    float patternSite_[kPatternSiteCount] = {0.0f, 0.0f, 0.0f, 0.0f};
    float motorCurrentAmps_[kDigitCount] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    bool configDirty_ = false;
    bool previousContact_ = false;
    bool previousWarning_ = false;
    uint32_t previousSlipCount_ = 0;
    /// Mode as of the end of the previous iteration, for entry/exit actions.
    ControlMode lastMode_ = ControlMode::Boot;

    /// Rail-stuck detection for the analogue inputs.
    float emgStuckTimer_ = 0.0f;

    float selfTestTimer_ = 0.0f;
    float uptimeSeconds_ = 0.0f;
    uint32_t loopCount_ = 0;

    HapticCue pendingHaptic_ = HapticCue::None;
};

}  // namespace jensen
