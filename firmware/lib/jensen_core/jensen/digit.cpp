#include "jensen/digit.h"

namespace jensen {

namespace {
/// Below this much travel the endpoints are indistinguishable and the digit
/// would be commanded into a stall rather than moved.
constexpr uint16_t kMinTravelUs = 60;
constexpr float kSettleTolerance = 0.005f;
}  // namespace

const char* digitName(Digit d) {
    switch (d) {
        case Digit::ThumbFlex: return "thumb";
        case Digit::ThumbRot:  return "thumb_rot";
        case Digit::Index:     return "index";
        case Digit::Middle:    return "middle";
        case Digit::Ring:      return "ring";
        case Digit::Pinky:     return "pinky";
        case Digit::Wrist:     return "wrist";
        default:               return "?";
    }
}

// --- DigitCalibration --------------------------------------------------------

bool DigitCalibration::isValid() const {
    const uint16_t lo = guardLowUs();
    const uint16_t hi = guardHighUs();
    if (hi <= lo) return false;
    const int32_t travel = static_cast<int32_t>(closedUs) - static_cast<int32_t>(openUs);
    if ((travel < 0 ? -travel : travel) < kMinTravelUs) return false;
    if (openUs < lo || openUs > hi) return false;
    if (closedUs < lo || closedUs > hi) return false;
    return true;
}

uint16_t DigitCalibration::pulseFor(float closure) const {
    const float t = clamp01(closure);
    const float us = lerp(static_cast<float>(openUs), static_cast<float>(closedUs), t);
    return clampU16(static_cast<int32_t>(us + 0.5f), guardLowUs(), guardHighUs());
}

// --- DigitController ---------------------------------------------------------

void DigitController::setCalibration(const DigitCalibration& calibration) {
    calibration_ = calibration;
}

void DigitController::setSpeed(float closurePerSecond) {
    speed_ = closurePerSecond > 0.0f ? closurePerSecond : 0.0f;
}

void DigitController::setTarget(float closure) {
    target_ = clamp01(closure);
}

void DigitController::setClosureLimit(float limit) {
    closureLimit_ = clamp01(limit);
}

void DigitController::snapTo(float closure) {
    closure_ = clamp01(closure);
    target_ = closure_;
    idleTimer_ = 0.0f;
    released_ = false;
}

float DigitController::effectiveTarget() const {
    // The force loop only limits *closing*; it must never drag a digit further
    // closed than it was told to go.
    return target_ < closureLimit_ ? target_ : closureLimit_;
}

void DigitController::update(float dt) {
    if (dt <= 0.0f) return;

    if (!locked_) {
        const float goal = effectiveTarget();
        const float previous = closure_;
        closure_ = approach(closure_, goal, speed_ * dt);
        if (!approxEqual(closure_, previous, 1e-6f)) {
            // Any motion cancels a pending idle release.
            idleTimer_ = 0.0f;
            released_ = false;
        }
    }

    if (idleReleaseEnabled_ && !locked_ && closure_ <= idleReleaseBelow_ &&
        effectiveTarget() <= idleReleaseBelow_) {
        idleTimer_ += dt;
        if (idleTimer_ >= idleReleaseAfter_) released_ = true;
    } else if (!released_) {
        idleTimer_ = 0.0f;
    }
}

bool DigitController::settled() const {
    return approxEqual(closure_, effectiveTarget(), kSettleTolerance);
}

uint16_t DigitController::pulseUs() const {
    if (!calibration_.enabled || !calibration_.isValid()) return 0;
    if (released_) return 0;
    return calibration_.pulseFor(closure_);
}

void DigitController::setIdleRelease(bool enabled, float belowClosure, float afterSeconds) {
    idleReleaseEnabled_ = enabled;
    idleReleaseBelow_ = clamp01(belowClosure);
    idleReleaseAfter_ = afterSeconds > 0.0f ? afterSeconds : 0.0f;
    if (!enabled) {
        released_ = false;
        idleTimer_ = 0.0f;
    }
}

}  // namespace jensen
