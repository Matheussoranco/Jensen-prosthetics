#include "jensen/force.h"

#include <cmath>

namespace jensen {

// --- ForceCalibration --------------------------------------------------------

bool ForceCalibration::isValid() const {
    return present && spanCounts > zeroCounts && referenceNewtons > 0.0f && exponent > 0.0f;
}

float ForceCalibration::newtons(float rawCounts) const {
    if (!isValid()) return 0.0f;
    const float span = static_cast<float>(spanCounts) - static_cast<float>(zeroCounts);
    const float normalized = (rawCounts - static_cast<float>(zeroCounts)) / span;
    if (normalized <= 0.0f) return 0.0f;
    const float shaped = (exponent == 1.0f) ? normalized : std::pow(normalized, exponent);
    return shaped * referenceNewtons;
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
}

float ForceSensor::update(float rawCounts, float dt) {
    raw_ = rawCounts;
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
    if (raw_ > 0.0f) {
        calibration_.zeroCounts = static_cast<uint16_t>(raw_ < 65535.0f ? raw_ : 65535.0f);
    }
    reset();
}

// --- GripForceController -----------------------------------------------------

void GripForceController::configure(const GripForceConfig& config) {
    config_ = config;
    if (config_.maxNewtons < config_.targetNewtons) config_.maxNewtons = config_.targetNewtons;
    config_.minLimit = clamp01(config_.minLimit);
}

void GripForceController::reset() {
    limit_ = 1.0f;
    peakForce_ = 0.0f;
    contact_ = false;
    overForce_ = false;
    slipCount_ = 0;
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

    // Integral-only regulation. There is no position feedback from the servos,
    // so a proportional term would act on a quantity we cannot observe; the
    // integrator simply walks the allowed closure until measured force sits at
    // the setpoint.
    const float error = target - peakForce_;
    limit_ = clamp01(limit_ + config_.integralGain * error * dt);

    // Anti-windup: while in contact the limit must stay close to where the
    // hand actually is, otherwise releasing an object leaves a large stale
    // headroom that the digits would sprint through.
    const float ceiling = clamp01(currentClosure + config_.windupMargin);
    if (limit_ > ceiling) limit_ = ceiling;
    if (limit_ < config_.minLimit) limit_ = config_.minLimit;
}

}  // namespace jensen
