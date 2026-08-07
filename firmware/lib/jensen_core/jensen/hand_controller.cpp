#include "jensen/hand_controller.h"

namespace jensen {

namespace {

/// ESP32 SAR ADC full scale at 12-bit resolution. Used only to recognise an
/// input pinned at a rail, which means a disconnected or shorted electrode.
constexpr float kAdcFullScale = 4095.0f;
constexpr float kRailMarginCounts = 4.0f;
constexpr float kStuckSeconds = 2.0f;

/// Nominal control period. Every time constant in the firmware is expressed in
/// seconds, so this only sets the overrun threshold.
constexpr float kNominalLoopSeconds = 0.010f;

/// Rate used when the aperture is driven by a direct command rather than by
/// the user's muscles.
constexpr float kCommandedApertureRate = 1.5f;
/// Rate at which the hand relaxes open when idle or calibrating.
constexpr float kRelaxRate = 0.6f;

constexpr float kSelfTestSeconds = 0.4f;

LedPattern ledFor(ControlMode mode) {
    switch (mode) {
        case ControlMode::Active:        return LedPattern::Solid;
        case ControlMode::Idle:          return LedPattern::Breathe;
        case ControlMode::Hold:          return LedPattern::SlowBlink;
        case ControlMode::Calibrating:   return LedPattern::FastBlink;
        case ControlMode::Fault:         return LedPattern::DoubleBlink;
        case ControlMode::SafeTorqueOff: return LedPattern::SosBlink;
        case ControlMode::Boot:
        case ControlMode::SelfTest:
        default:                         return LedPattern::FastBlink;
    }
}

}  // namespace

// --- Lifecycle ---------------------------------------------------------------

void HandController::begin(const HandConfig& config) {
    config_ = config;
    configureFromConfig();

    modes_.reset();
    safety_.reset();
    loop_.configure(kNominalLoopSeconds);
    loop_.reset();
    battery_.reset(0.0f);
    holdGuard_.reset();

    resetMotion();

    selfTestTimer_ = 0.0f;
    uptimeSeconds_ = 0.0f;
    loopCount_ = 0;
    configDirty_ = false;
    previousContact_ = false;
    previousWarning_ = false;
    previousSlipCount_ = 0;
    emgStuckTimer_ = 0.0f;
    pendingHaptic_ = HapticCue::None;
    lastMode_ = ControlMode::Boot;
}

void HandController::configureFromConfig() {
    flexor_.configure(config_.emg);
    extensor_.configure(config_.emg);
    flexor_.setCalibration(config_.emgFlexor);
    extensor_.setCalibration(config_.emgExtensor);
    drive_.configure(config_.drive);

    for (uint8_t i = 0; i < kGraspDigitCount; ++i) {
        force_[i].configure(config_.forceSensor);
        force_[i].setCalibration(config_.force[i]);
    }
    gripForce_.configure(config_.gripForce);

    for (uint8_t i = 0; i < kDigitCount; ++i) {
        digits_[i].setCalibration(config_.digits[i]);
        digits_[i].setSpeed(config_.digitSpeed[i]);
        // The wrist rotator is never released mid-task: letting a loaded
        // forearm rotator go limp is far more startling than a finger.
        const bool allowRelease =
            config_.idleReleaseEnabled && (i != index(Digit::Wrist));
        digits_[i].setIdleRelease(allowRelease);
    }

    grips_.setEnabledMask(config_.enabledGrips);
    grips_.select(config_.defaultGrip);
    blendFrom_ = grips_.current();
    blendTimer_ = blendDuration_;

    battery_.configure(config_.battery);
    holdGuard_.configure(config_.holdGuard);
    modes_.setIdleTimeout(config_.idleTimeoutSeconds);
    emgRunner_.configure(CalibrationTiming{});
}

void HandController::applyConfig(const HandConfig& config) {
    const GripId active = grips_.current();
    config_ = config;
    configureFromConfig();
    if (grips_.enabled(active)) grips_.select(active);
    blendFrom_ = grips_.current();
    blendTimer_ = blendDuration_;
}

void HandController::resetMotion() {
    aperture_ = 0.0f;
    commandedAperture_ = 0.0f;
    apertureCommandActive_ = false;
    drive_.reset();
    gripForce_.reset();
    flexor_.reset();
    extensor_.reset();
    for (uint8_t i = 0; i < kGraspDigitCount; ++i) {
        force_[i].reset();
        digitLimit_[i] = 1.0f;
    }
    for (uint8_t i = 0; i < kDigitCount; ++i) digits_[i].snapTo(0.0f);
    digits_[index(Digit::Wrist)].snapTo(wristTarget_);
}

// --- Control loop ------------------------------------------------------------

const HandOutputs& HandController::update(const HandInputs& inputs, float dt) {
    // A zero or negative dt would silently disable every rate limit in the
    // system, so it is clamped rather than trusted.
    if (dt <= 0.0f) dt = kNominalLoopSeconds;
    ++loopCount_;
    uptimeSeconds_ += dt;
    loop_.update(dt);

    updateSensors(inputs, dt);
    updateFaults(inputs, dt);

    // Faults are evaluated before user input: a severe fault must win over a
    // button press in the same iteration.
    modes_.setFaults(safety_.any(), safety_.requiresSafeTorqueOff());

    if (modes_.mode() == ControlMode::Boot) {
        modes_.update(dt, false);  // Boot -> SelfTest
    }

    if (modes_.mode() == ControlMode::SelfTest) {
        selfTestTimer_ += dt;
        if (selfTestTimer_ >= kSelfTestSeconds) {
            const bool ok = inputs.servoBusOk && config_.isValid();
            modes_.dispatch(ok ? SystemEvent::SelfTestPassed : SystemEvent::SelfTestFailed);
        }
    }

    // Button gestures. The mode machine decides the state change; grip
    // selection is this class's business.
    if (inputs.button != ButtonEvent::None) {
        modes_.dispatch(inputs.button);
        if (inputs.button == ButtonEvent::Click && modes_.acceptsUserInput()) nextGrip();
    }

    updateCalibration(inputs, dt);
    updateDrive(dt);

    const bool active = (flexor_.activation() > 0.0f) || (extensor_.activation() > 0.0f) ||
                        (aperture_ > 0.02f) || gripForce_.contact() || apertureCommandActive_;
    modes_.update(dt, active);

    updateGripBlend(dt);
    applyPose(dt);

    // Mode entry/exit effects, evaluated once the new mode is settled.
    const ControlMode mode = modes_.mode();
    if (mode != lastMode_) {
        if (mode == ControlMode::Hold || lastMode_ == ControlMode::Hold) {
            requestHaptic(HapticCue::DoubleTick);
        }
        if (mode == ControlMode::Calibrating) {
            emgRunner_.begin(flexor_, extensor_, config_.drive.mode == DriveMode::DualChannel);
        } else if (lastMode_ == ControlMode::Calibrating && emgRunner_.running()) {
            emgRunner_.abort(flexor_, extensor_);
        }
        lastMode_ = mode;
    }

    updateOutputs();
    updateTelemetry(dt);
    return outputs_;
}

void HandController::updateSensors(const HandInputs& inputs, float dt) {
    // While a calibration is running the runner owns the EMG channels: it is
    // manipulating the very baseline that `update()` reads.
    if (!emgRunner_.running()) {
        flexor_.update(inputs.emgFlexorCounts, dt);
        extensor_.update(inputs.emgExtensorCounts, dt);
    }

    for (uint8_t i = 0; i < kGraspDigitCount; ++i) {
        force_[i].update(inputs.fsrCounts[i], dt);
        // Carried through to telemetry only — see HandTelemetry::contactDigital.
        contactDigital_[i] = inputs.contactDigital[i];
    }

    battery_.update(inputs.batteryVolts, dt);
}

void HandController::updateFaults(const HandInputs& inputs, float dt) {
    safety_.set(kFaultServoBus, !inputs.servoBusOk);
    safety_.set(kFaultBatteryLow, battery_.low());
    safety_.set(kFaultBatteryCritical, battery_.critical());
    safety_.set(kFaultDigitCalibration, !config_.digitsCalibrated());
    safety_.set(kFaultOverForce, gripForce_.overForce());
    safety_.set(kFaultSustainedLoad, holdGuard_.backedOff());

    // An uncalibrated electrode is only a fault once the hand is meant to be
    // driven by it — flagging it during calibration would abort the very
    // procedure that fixes it.
    const bool calibrating = emgRunner_.running();
    safety_.set(kFaultEmgCalibration, !calibrating && !config_.emgCalibrated());

    // An input pinned at either rail for seconds at a time is a wiring
    // problem, not a contraction.
    const float flex = inputs.emgFlexorCounts;
    const bool stuck = (flex <= kRailMarginCounts) ||
                       (flex >= kAdcFullScale - kRailMarginCounts);
    if (stuck && !calibrating) {
        emgStuckTimer_ += dt;
    } else {
        emgStuckTimer_ = 0.0f;
    }
    safety_.set(kFaultSensorRange, emgStuckTimer_ >= kStuckSeconds);

    // Sustained, not instantaneous: one late iteration is noise, a loop that
    // averages twice its budget is a real problem.
    safety_.set(kFaultLoopOverrun, loop_.averageSeconds() > 2.0f * kNominalLoopSeconds);
}

void HandController::updateCalibration(const HandInputs& inputs, float dt) {
    if (!emgRunner_.running()) return;

    const HapticCue cue = emgRunner_.update(flexor_, extensor_, inputs.emgFlexorCounts,
                                            inputs.emgExtensorCounts, dt);
    requestHaptic(cue);

    if (!emgRunner_.finished()) return;

    if (emgRunner_.succeeded()) {
        config_.emgFlexor = emgRunner_.flexorResult();
        flexor_.setCalibration(config_.emgFlexor);
        if (config_.drive.mode == DriveMode::DualChannel) {
            config_.emgExtensor = emgRunner_.extensorResult();
            extensor_.setCalibration(config_.emgExtensor);
        }
        configDirty_ = true;
        modes_.dispatch(SystemEvent::CalibrationComplete);
    } else {
        modes_.dispatch(SystemEvent::CalibrationAborted);
    }
}

void HandController::updateDrive(float dt) {
    const float velocity = drive_.update(flexor_.activation(), extensor_.activation(), dt);

    if (drive_.consumeCoContraction() && modes_.acceptsUserInput()) {
        modes_.dispatch(SystemEvent::CoContraction);
        nextGrip();
    }

    // Any genuine muscle activity revokes a direct aperture command; the
    // wearer always outranks the CLI and the display.
    if (flexor_.activation() > 0.0f || extensor_.activation() > 0.0f) {
        apertureCommandActive_ = false;
    }

    switch (modes_.mode()) {
        case ControlMode::Active:
        case ControlMode::Fault:
            if (apertureCommandActive_) {
                aperture_ = approach(aperture_, commandedAperture_, kCommandedApertureRate * dt);
                if (approxEqual(aperture_, commandedAperture_, 1e-3f)) apertureCommandActive_ = false;
            } else {
                // The integrator HACKberry uses, in seconds rather than in
                // loop iterations.
                aperture_ = clamp01(aperture_ + velocity * dt);
            }
            break;

        case ControlMode::Idle:
        case ControlMode::Calibrating:
            // Relax open. During calibration the hand must be still and out of
            // the way; when idle this is what allows the outputs to release.
            aperture_ = approach(aperture_, 0.0f, kRelaxRate * dt);
            break;

        case ControlMode::Hold:
            // Frozen by definition — the user is carrying something.
            break;

        case ControlMode::Boot:
        case ControlMode::SelfTest:
        case ControlMode::SafeTorqueOff:
        default:
            aperture_ = approach(aperture_, 0.0f, kRelaxRate * dt);
            break;
    }
}

void HandController::updateGripBlend(float dt) {
    if (blendTimer_ < blendDuration_) blendTimer_ += dt;
}

void HandController::applyPose(float dt) {
    // Per-grip force setpoint, further reduced when the hold guard has decided
    // the servos have been working hard for too long.
    const uint8_t gripIdx = static_cast<uint8_t>(grips_.current());
    const float configuredScale =
        (gripIdx < kGripCount && config_.gripForceScale[gripIdx] > 0.0f)
            ? config_.gripForceScale[gripIdx]
            : 1.0f;
    // The guard reads last iteration's peak force and the force loop reads
    // last iteration's guard state. One tick of lag either way is unavoidable
    // in a feedback pair like this, and irrelevant against a 25 s hold timer.
    gripForce_.setTargetScale(configuredScale * holdGuard_.forceScale());
    gripForce_.update(force_, kGraspDigitCount, aperture_, dt);
    holdGuard_.update(gripForce_.peakForce(), dt);

    // The grip-force loop limits how far the *grasp* closes. Applying it in
    // aperture space rather than to each digit matters: patterns like the key
    // grip park fingers at a fixed high closure, and a per-digit clamp would
    // drag those parked digits open the moment the thumb found the object.
    //
    // The limit is written back into the aperture integrator rather than only
    // filtered on the way out. Otherwise the user's command keeps climbing to
    // 1.0 behind a held-back hand, and the instant the object is released the
    // limit relaxes onto a fully wound-up command and the hand slams shut.
    if (aperture_ > gripForce_.closureLimit()) aperture_ = gripForce_.closureLimit();
    const float effectiveAperture = aperture_;

    const GraspPose pose =
        (blendTimer_ < blendDuration_)
            ? GripLibrary::blend(blendFrom_, grips_.current(), blendTimer_ / blendDuration_,
                                 effectiveAperture)
            : GripLibrary::pose(grips_.current(), effectiveAperture);

    const bool hold = (modes_.mode() == ControlMode::Hold);
    const float digitTarget = gripForce_.targetNewtons();

    for (uint8_t i = 0; i < kGraspDigitCount; ++i) {
        // Adaptive grasp: an instrumented digit that has reached the force
        // target stops where it is while the rest keep closing, so the hand
        // conforms to objects that are not symmetric about the palm.
        if (force_[i].present() && force_[i].force() > digitTarget) {
            digitLimit_[i] = approach(digitLimit_[i], digits_[i].closure(), 2.0f * dt);
        } else {
            digitLimit_[i] = approach(digitLimit_[i], 1.0f, config_.gripForce.releasePerSecond * dt);
        }

        DigitController& digit = digits_[i];
        digit.setLocked(hold);
        digit.setClosureLimit(digitLimit_[i]);
        digit.setTarget(pose.closure[i]);
        digit.update(dt);
    }

    DigitController& wrist = digits_[index(Digit::Wrist)];
    wrist.setLocked(hold);
    wrist.setClosureLimit(1.0f);
    wrist.setTarget(wristTarget_);
    wrist.update(dt);

    // Contact and slip cues.
    const bool contact = gripForce_.contact();
    if (contact && !previousContact_) requestHaptic(HapticCue::Contact);
    previousContact_ = contact;

    if (gripForce_.slipCount() != previousSlipCount_) {
        previousSlipCount_ = gripForce_.slipCount();
        requestHaptic(HapticCue::Slip);
    }

    const bool warning = safety_.any();
    if (warning && !previousWarning_) requestHaptic(HapticCue::Warning);
    previousWarning_ = warning;
}

void HandController::updateOutputs() {
    const bool energised = modes_.drivesServos() && !safety_.requiresSafeTorqueOff();

    outputs_.servoRailEnabled = energised;
    outputs_.servoOutputEnabled = energised;
    for (uint8_t i = 0; i < kDigitCount; ++i) {
        outputs_.pulseUs[i] = energised ? digits_[i].pulseUs() : 0;
    }

    outputs_.led = ledFor(modes_.mode());
    outputs_.haptic = config_.hapticsEnabled ? pendingHaptic_ : HapticCue::None;
    pendingHaptic_ = HapticCue::None;
}

void HandController::updateTelemetry(float dt) {
    HandTelemetry& t = telemetry_;
    t.mode = modes_.mode();
    t.grip = grips_.current();
    t.gripName = GripLibrary::name(grips_.current());
    t.aperture = aperture_;
    t.velocity = drive_.velocity();

    t.emgFlexor = flexor_.activation();
    t.emgExtensor = extensor_.activation();
    t.emgFlexorEnvelope = flexor_.envelope();
    t.emgExtensorEnvelope = extensor_.envelope();

    for (uint8_t i = 0; i < kGraspDigitCount; ++i) {
        t.force[i] = force_[i].force();
        t.contactDigital[i] = contactDigital_[i];
    }
    t.peakForce = gripForce_.peakForce();
    t.forceTarget = gripForce_.targetNewtons();
    t.closureLimit = gripForce_.closureLimit();
    t.contact = gripForce_.contact();
    t.slipCount = gripForce_.slipCount();

    t.batteryVolts = battery_.packVolts();
    t.batteryCellVolts = battery_.cellVolts();
    t.stateOfCharge = battery_.stateOfCharge();

    t.faults = safety_.faults();
    t.faultName = safety_.worstFaultName();

    t.calibrationStep = emgRunner_.running() ? emgRunner_.stepName() : nullptr;
    t.calibrationProgress = emgRunner_.progress();

    for (uint8_t i = 0; i < kDigitCount; ++i) {
        t.digits[i].closure = digits_[i].closure();
        t.digits[i].target = digits_[i].target();
        t.digits[i].pulseUs = outputs_.pulseUs[i];
        t.digits[i].locked = digits_[i].locked();
        t.digits[i].released = digits_[i].released();
    }

    t.loopSeconds = dt;
    t.loopCount = loopCount_;
    t.uptimeSeconds = static_cast<uint32_t>(uptimeSeconds_);
}

// --- Commands ----------------------------------------------------------------

void HandController::selectGrip(GripId grip) {
    if (grip == grips_.current() || !grips_.enabled(grip)) return;
    blendFrom_ = grips_.current();
    blendTimer_ = 0.0f;
    grips_.select(grip);
    requestHaptic(HapticCue::Tick);
}

void HandController::nextGrip() {
    if (grips_.enabledCount() <= 1) return;
    blendFrom_ = grips_.current();
    blendTimer_ = 0.0f;
    grips_.next();
    requestHaptic(HapticCue::Tick);
}

void HandController::previousGrip() {
    if (grips_.enabledCount() <= 1) return;
    blendFrom_ = grips_.current();
    blendTimer_ = 0.0f;
    grips_.previous();
    requestHaptic(HapticCue::Tick);
}

void HandController::setHold(bool hold) {
    const bool holding = (modes_.mode() == ControlMode::Hold);
    if (hold != holding) modes_.dispatch(ButtonEvent::DoubleClick);
}

void HandController::park(bool parked) {
    const bool isParked = (modes_.mode() == ControlMode::SafeTorqueOff);
    if (parked != isParked) modes_.dispatch(ButtonEvent::VeryLongPress);
}

void HandController::startEmgCalibration() {
    if (modes_.mode() == ControlMode::Calibrating) return;
    modes_.dispatch(ButtonEvent::LongPress);
}

void HandController::abortCalibration() {
    if (modes_.mode() != ControlMode::Calibrating) return;
    emgRunner_.abort(flexor_, extensor_);
    modes_.dispatch(SystemEvent::CalibrationAborted);
    lastMode_ = modes_.mode();
}

void HandController::commandAperture(float aperture) {
    commandedAperture_ = clamp01(aperture);
    apertureCommandActive_ = true;
}

void HandController::setWristTarget(float closure) {
    wristTarget_ = clamp01(closure);
}

void HandController::tareForceSensors() {
    for (uint8_t i = 0; i < kGraspDigitCount; ++i) {
        if (!force_[i].present()) continue;
        force_[i].tare();
        config_.force[i].zeroCounts = force_[i].calibration().zeroCounts;
    }
    gripForce_.reset();
    configDirty_ = true;
}

bool HandController::consumeConfigDirty() {
    const bool dirty = configDirty_;
    configDirty_ = false;
    return dirty;
}

// --- Haptics -----------------------------------------------------------------

uint8_t HandController::hapticPriority(HapticCue cue) {
    switch (cue) {
        case HapticCue::None:       return 0;
        case HapticCue::Tick:       return 1;
        case HapticCue::DoubleTick: return 2;
        case HapticCue::Contact:    return 3;
        case HapticCue::Confirm:    return 4;
        case HapticCue::Slip:       return 5;
        case HapticCue::Warning:    return 6;
        default:                    return 0;
    }
}

void HandController::requestHaptic(HapticCue cue) {
    // One cue per iteration: the motor cannot play two patterns at once, and
    // queueing them would report events long after they happened.
    if (hapticPriority(cue) > hapticPriority(pendingHaptic_)) pendingHaptic_ = cue;
}

}  // namespace jensen
