#include "jensen/force.h"

#include <cmath>

namespace jensen {

// --- ForceCalibration --------------------------------------------------------

bool ForceCalibration::isValid() const {
    return present && spanCounts > zeroCounts && referenceNewtons > 0.0f && exponent > 0.0f;
}

float ForceCalibration::newtons(float rawCounts) const {
    if (!isValid()) return 0.0f;
    if (!std::isfinite(rawCounts)) return 0.0f;
    const float span = static_cast<float>(spanCounts) - static_cast<float>(zeroCounts);
    const float normalized = (rawCounts - static_cast<float>(zeroCounts)) / span;
    if (normalized <= 0.0f) return 0.0f;
    const float shaped = (exponent == 1.0f) ? normalized : std::pow(normalized, exponent);
    float newtons = shaped * referenceNewtons;
    // Upper clamp: an FSR saturates, it never reports 10x the reference
    // load. Without this a shorted rail (4095 counts) feeds an unbounded
    // peakForce into the grip loop and the hold guard. 2x reference keeps
    // genuine overloads visible while bounding the fault path.
    const float cap = referenceNewtons * 2.0f;
    if (!std::isfinite(newtons)) return cap;
    if (newtons > cap) newtons = cap;
    if (newtons < 0.0f) newtons = 0.0f;
    return newtons;
}

// --- ForceSensor -------------------------------------------------------------

void ForceSensor::configure(const ForceSensorConfig& config) {
    config_ = config;
    smooth_.setTimeConstant(config_.smoothingSeconds);
    derivative_.configure(config_.derivativeSeconds);
    contact_.configure(config_.contactOnNewtons, config_.contactOffNewtons);
}

void ForceSensor::setCalibration(const ForceCalibration& calibration) {
    calibration_ = calibration;
}

void ForceSensor::reset() {
    smooth_.reset(0.0f);
    derivative_.reset(0.0f);
    contact_.reset(false);
    raw_ = 0.0f;
    force_ = 0.0f;
    rate_ = 0.0f;
    slipEvent_ = false;
    for (uint8_t i = 0; i < kTareSamples; ++i) rawHistory_[i] = 0.0f;
    rawHistoryCount_ = 0;
    rawHistoryHead_ = 0;
}

float ForceSensor::update(float rawCounts, float dt) {
    raw_ = rawCounts;
    rawHistory_[rawHistoryHead_] = rawCounts;
    rawHistoryHead_ = static_cast<uint8_t>((rawHistoryHead_ + 1) % kTareSamples);
    if (rawHistoryCount_ < kTareSamples) ++rawHistoryCount_;
    if (!calibration_.present) {
        force_ = 0.0f;
        rate_ = 0.0f;
        contact_.reset(false);
        return 0.0f;
    }

    force_ = smooth_.update(calibration_.newtons(rawCounts), dt);
    rate_ = derivative_.update(force_, dt);

    const bool wasContact = contact_.state();
    const bool isContact = contact_.update(force_);

    // Slip is only meaningful while we believe we are holding something. A
    // force collapse that coincides with losing contact altogether is a
    // deliberate release, not a slip, so require contact to still be asserted.
    if (isContact && wasContact && rate_ < config_.slipRateNewtonsPerSec) {
        slipEvent_ = true;
    }
    return force_;
}

bool ForceSensor::consumeSlip() {
    const bool event = slipEvent_;
    slipEvent_ = false;
    return event;
}

void ForceSensor::tare() {
    // Mean of recent history, not the single latest sample: a lone ADC spike
    // (servo switching noise is the usual suspect) would otherwise bias the
    // zero — and therefore every newton — until the next tare.
    float zero = raw_;
    if (rawHistoryCount_ > 0) {
        float sum = 0.0f;
        for (uint8_t i = 0; i < rawHistoryCount_; ++i) sum += rawHistory_[i];
        zero = sum / static_cast<float>(rawHistoryCount_);
    }
    if (zero > 0.0f) {
        calibration_.zeroCounts = static_cast<uint16_t>(zero < 65535.0f ? zero : 65535.0f);
    }
    // A pending slip report belongs to the grasp in progress, not to the
    // zeroing procedure: reset() clears it, so save and restore it here.
    const bool pendingSlip = slipEvent_;
    reset();
    slipEvent_ = pendingSlip;
}

// --- GripForceController -----------------------------------------------------

void GripForceController::configure(const GripForceConfig& config) {
    config_ = config;
    if (config_.maxNewtons < config_.targetNewtons) config_.maxNewtons = config_.targetNewtons;
    config_.minLimit = clamp01(config_.minLimit);
    if (config_.proportionalGain < 0.0f) config_.proportionalGain = 0.0f;
    if (config_.integralGain < 0.0f) config_.integralGain = 0.0f;
}

void GripForceController::reset() {
    limit_ = 1.0f;
    peakForce_ = 0.0f;
    contact_ = false;
    overForce_ = false;
    slipCount_ = 0;
    prevError_ = 0.0f;
}

void GripForceController::setTargetScale(float scale) {
    targetScale_ = scale > 0.0f ? scale : 1.0f;
}

void GripForceController::update(ForceSensor* sensors, uint8_t count, float currentClosure,
                                 float dt) {
    if (dt <= 0.0f) return;

    peakForce_ = 0.0f;
    contact_ = false;
    bool slipped = false;

    for (uint8_t i = 0; i < count; ++i) {
        ForceSensor& sensor = sensors[i];
        if (!sensor.present()) continue;
        if (sensor.force() > peakForce_) peakForce_ = sensor.force();
        if (sensor.contact()) contact_ = true;
        if (sensor.consumeSlip()) slipped = true;
    }

    const float target = targetNewtons();
    overForce_ = peakForce_ > config_.maxNewtons * targetScale_;

    if (!contact_) {
        // Nothing in the hand: give the user the full range back.
        limit_ = approach(limit_, 1.0f, config_.releasePerSecond * dt);
        return;
    }

    if (slipped) {
        ++slipCount_;
        // Respond to a slip before the integrator does — by the time an
        // integral term has wound in far enough the object is already gone.
        limit_ = clamp01(limit_ + config_.slipTightenStep);
    }

    // Incremental (velocity-form) PI regulation. The output `limit_` is
    // itself the integrator state, so the proportional action applies to the
    // *change* in error and the integral to the error itself:
    //
    //   limit += Kp * (e - ePrev) + Ki * e * dt
    //
    // Pure integral-only control reacted a full error-second late to a
    // sudden overload; the Kp term bites on the first tick the force jumps.
    // Because P acts on (e - ePrev), a steady-state error does not keep
    // pushing — no proportional windup, and the anti-windup clamp below
    // still bounds the single remaining integrator.
    const float error = target - peakForce_;
    limit_ = clamp01(limit_ + config_.proportionalGain * (error - prevError_) +
                      config_.integralGain * error * dt);
    prevError_ = error;

    // Anti-windup: while in contact the limit must stay close to where the
    // hand actually is, otherwise releasing an object leaves a large stale
    // headroom that the digits would sprint through.
    const float ceiling = clamp01(currentClosure + config_.windupMargin);
    if (limit_ > ceiling) limit_ = ceiling;
    if (limit_ < config_.minLimit) limit_ = config_.minLimit;
}

}  // namespace jensen
