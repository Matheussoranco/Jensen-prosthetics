#include "jensen/calibration.h"

namespace jensen {

namespace {
/// Guard limits are placed this far outside the captured endpoints, so a
/// calibrated digit still has a little room before it hits a printed stop.
constexpr uint16_t kGuardMarginUs = 40;
/// Absolute pulse envelope accepted for any hobby servo channel.
constexpr uint16_t kAbsoluteMinUs = 500;
constexpr uint16_t kAbsoluteMaxUs = 2500;
}  // namespace

// --- EmgCalibrationRunner ----------------------------------------------------

void EmgCalibrationRunner::configure(const CalibrationTiming& timing) {
    timing_ = timing;
}

float EmgCalibrationRunner::stepDuration(Step step) const {
    switch (step) {
        case Step::SettleRest:
        case Step::SettleFlexor:
        case Step::SettleExtensor:
            return timing_.settleSeconds;
        case Step::CaptureRest:
            return timing_.restSeconds;
        case Step::CaptureFlexor:
        case Step::CaptureExtensor:
            return timing_.effortSeconds;
        default:
            return 0.0f;
    }
}

void EmgCalibrationRunner::enter(Step step) {
    step_ = step;
    stepTimer_ = 0.0f;
}

void EmgCalibrationRunner::begin(EmgChannel& flexor, EmgChannel& extensor, bool dualChannel) {
    dualChannel_ = dualChannel;
    flexorResult_ = EmgCalibration{};
    extensorResult_ = EmgCalibration{};
    elapsed_ = 0.0f;

    total_ = timing_.settleSeconds * (dualChannel ? 3.0f : 2.0f) + timing_.restSeconds +
             timing_.effortSeconds * (dualChannel ? 2.0f : 1.0f);
    if (total_ <= 0.0f) total_ = 1.0f;

    flexor.beginCalibration();
    if (dualChannel_) extensor.beginCalibration();

    enter(Step::SettleRest);
}

void EmgCalibrationRunner::abort(EmgChannel& flexor, EmgChannel& extensor) {
    // Discard partial captures rather than committing half a calibration.
    if (flexor.calibrating()) flexor.finishCalibration();
    if (extensor.calibrating()) extensor.finishCalibration();
    flexor.reset();
    extensor.reset();
    enter(Step::Failed);
}

bool EmgCalibrationRunner::running() const {
    return step_ != Step::Idle && step_ != Step::Finished && step_ != Step::Failed;
}

HapticCue EmgCalibrationRunner::update(EmgChannel& flexor, EmgChannel& extensor,
                                       float flexorCounts, float extensorCounts, float dt) {
    if (!running() || dt <= 0.0f) return HapticCue::None;

    stepTimer_ += dt;
    elapsed_ += dt;

    switch (step_) {
        case Step::CaptureRest:
            flexor.observeRest(flexorCounts, dt);
            if (dualChannel_) extensor.observeRest(extensorCounts, dt);
            break;
        case Step::CaptureFlexor:
            flexor.observeEffort(flexorCounts, dt);
            break;
        case Step::CaptureExtensor:
            extensor.observeEffort(extensorCounts, dt);
            break;
        default:
            break;
    }

    if (stepTimer_ < stepDuration(step_)) return HapticCue::None;

    // Phase boundary.
    switch (step_) {
        case Step::SettleRest:
            enter(Step::CaptureRest);
            return HapticCue::Tick;

        case Step::CaptureRest:
            enter(Step::SettleFlexor);
            return HapticCue::Confirm;

        case Step::SettleFlexor:
            enter(Step::CaptureFlexor);
            return HapticCue::Tick;

        case Step::CaptureFlexor:
            flexorResult_ = flexor.finishCalibration();
            if (!flexorResult_.isUsable()) {
                // Stop here: a flexor channel that cannot separate contraction
                // from rest cannot drive the hand, and continuing to the
                // extensor phase would only waste the user's time.
                if (extensor.calibrating()) extensor.finishCalibration();
                enter(Step::Failed);
                return HapticCue::Warning;
            }
            if (!dualChannel_) {
                enter(Step::Finished);
                return HapticCue::Confirm;
            }
            enter(Step::SettleExtensor);
            return HapticCue::Confirm;

        case Step::SettleExtensor:
            enter(Step::CaptureExtensor);
            return HapticCue::Tick;

        case Step::CaptureExtensor:
            extensorResult_ = extensor.finishCalibration();
            if (!extensorResult_.isUsable()) {
                enter(Step::Failed);
                return HapticCue::Warning;
            }
            enter(Step::Finished);
            return HapticCue::Confirm;

        default:
            return HapticCue::None;
    }
}

const char* EmgCalibrationRunner::stepName() const {
    switch (step_) {
        case Step::Idle:            return nullptr;
        case Step::SettleRest:      return "relax your arm";
        case Step::CaptureRest:     return "hold still";
        case Step::SettleFlexor:    return "ready to close";
        case Step::CaptureFlexor:   return "close hard";
        case Step::SettleExtensor:  return "ready to open";
        case Step::CaptureExtensor: return "open hard";
        case Step::Finished:        return "calibration saved";
        case Step::Failed:          return "calibration failed";
        default:                    return nullptr;
    }
}

float EmgCalibrationRunner::progress() const {
    if (step_ == Step::Idle) return 0.0f;
    if (finished()) return 1.0f;
    return clamp01(elapsed_ / total_);
}

// --- DigitCalibrationRunner --------------------------------------------------

void DigitCalibrationRunner::begin(Digit digit, const DigitCalibration& current) {
    active_ = true;
    digit_ = digit;
    base_ = current;
    haveOpen_ = false;
    haveClosed_ = false;
    // Start from the digit's current open position: it is the pose the hand is
    // already in, so beginning there does not jerk the tendon.
    pulseUs_ = current.isValid() ? current.openUs : 1500;
    open_ = pulseUs_;
    closed_ = pulseUs_;
}

void DigitCalibrationRunner::setPulseUs(uint16_t us) {
    pulseUs_ = clampU16(static_cast<int32_t>(us), base_.guardLowUs(), base_.guardHighUs());
}

uint16_t DigitCalibrationRunner::jog(int16_t deltaUs) {
    setPulseUs(static_cast<uint16_t>(
        clampU16(static_cast<int32_t>(pulseUs_) + deltaUs, kAbsoluteMinUs, kAbsoluteMaxUs)));
    return pulseUs_;
}

bool DigitCalibrationRunner::result(DigitCalibration& out) const {
    if (!haveOpen_ || !haveClosed_) return false;

    DigitCalibration cal = base_;
    cal.openUs = open_;
    cal.closedUs = closed_;

    const uint16_t lo = open_ < closed_ ? open_ : closed_;
    const uint16_t hi = open_ < closed_ ? closed_ : open_;
    cal.minUs = clampU16(static_cast<int32_t>(lo) - kGuardMarginUs, kAbsoluteMinUs, kAbsoluteMaxUs);
    cal.maxUs = clampU16(static_cast<int32_t>(hi) + kGuardMarginUs, kAbsoluteMinUs, kAbsoluteMaxUs);
    cal.enabled = true;

    if (!cal.isValid()) return false;
    out = cal;
    return true;
}

}  // namespace jensen
