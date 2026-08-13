#include "jensen/emg.h"

#include <cmath>

namespace jensen {

namespace {
/// A contraction peak must clear the resting noise by this factor before the
/// calibration is considered trustworthy.
constexpr float kMinContrastRatio = 2.5f;
/// ...and by this many absolute ADC counts, so a near-zero noise reading (a
/// disconnected input reading a hard 0) cannot satisfy the ratio test alone.
constexpr float kMinContrastCounts = 120.0f;
}  // namespace

bool EmgCalibration::isUsable() const {
    if (!valid) return false;
    if (mvcCounts <= noiseCounts) return false;
    if ((mvcCounts - noiseCounts) < kMinContrastCounts) return false;
    if (noiseCounts > 0.0f && (mvcCounts / noiseCounts) < kMinContrastRatio) return false;
    return true;
}

// --- EmgChannel --------------------------------------------------------------

void EmgChannel::configure(const EmgConfig& config) {
    config_ = config;
    if (config_.gamma <= 0.0f) config_.gamma = 1.0f;
    config_.deadband = clamp(config_.deadband, 0.0f, 0.9f);
    envelope_.configure(config_.attackSeconds, config_.releaseSeconds);
}

void EmgChannel::setCalibration(const EmgCalibration& calibration) {
    calibration_ = calibration;
    envelope_.setBaseline(calibration_.baselineCounts);
}

void EmgChannel::reset() {
    despike_.reset();
    envelope_.reset(0.0f);
    raw_ = 0.0f;
    normalized_ = 0.0f;
    activation_ = 0.0f;
}

float EmgChannel::floorCounts() const {
    return calibration_.noiseCounts * config_.noiseMargin;
}

void EmgChannel::beginCalibration() {
    calibrating_ = true;
    restSum_ = 0.0;
    restCount_ = 0;
    restPeakEnvelope_ = 0.0f;
    effortPeakEnvelope_ = 0.0f;
    // Rest capture measures deviation about a baseline we do not know yet, so
    // start from zero and let the running mean converge.
    envelope_.setBaseline(0.0f);
    envelope_.reset(0.0f);
    despike_.reset();
}

void EmgChannel::observeRest(float rawCounts, float dt) {
    if (!calibrating_) return;
    const float sample = despike_.update(rawCounts);
    restSum_ += static_cast<double>(sample);
    ++restCount_;

    // Track the envelope about the running mean so the noise figure reflects
    // fluctuation, not the DC level of the electrode.
    const float runningMean = static_cast<float>(restSum_ / static_cast<double>(restCount_));
    envelope_.setBaseline(runningMean);
    const float env = envelope_.update(sample, dt);
    // Ignore the first samples, while the mean is still settling.
    if (restCount_ > 8 && env > restPeakEnvelope_) restPeakEnvelope_ = env;
}

void EmgChannel::observeEffort(float rawCounts, float dt) {
    if (!calibrating_) return;
    const float sample = despike_.update(rawCounts);
    const float env = envelope_.update(sample, dt);
    if (env > effortPeakEnvelope_) effortPeakEnvelope_ = env;
}

EmgCalibration EmgChannel::finishCalibration() {
    calibrating_ = false;
    EmgCalibration result{};
    result.baselineCounts =
        restCount_ > 0 ? static_cast<float>(restSum_ / static_cast<double>(restCount_)) : 0.0f;
    result.noiseCounts = restPeakEnvelope_;
    result.mvcCounts = effortPeakEnvelope_;
    result.valid = true;
    result.valid = result.isUsable();
    setCalibration(result);
    reset();
    return result;
}

float EmgChannel::update(float rawCounts, float dt) {
    raw_ = rawCounts;
    const float sample = despike_.update(rawCounts);
    const float env = envelope_.update(sample, dt);

    const float floorC = floorCounts();
    const float span = calibration_.mvcCounts - floorC;
    if (!calibration_.valid || span <= 0.0f) {
        // Without a trustworthy calibration we refuse to synthesise intent.
        // The application surfaces this as a fault rather than driving servos
        // from an arbitrary scaling.
        normalized_ = 0.0f;
        activation_ = 0.0f;
        return activation_;
    }

    normalized_ = clamp01((env - floorC) / span);

    if (normalized_ <= config_.deadband) {
        activation_ = 0.0f;
    } else {
        const float shaped = (normalized_ - config_.deadband) / (1.0f - config_.deadband);
        activation_ = (config_.gamma == 1.0f) ? shaped : std::pow(shaped, config_.gamma);
        activation_ = clamp01(activation_);
    }
    return activation_;
}

// --- ProportionalDrive -------------------------------------------------------

void ProportionalDrive::configure(const DriveConfig& config) {
    config_ = config;
    if (config_.singleHoldBelow < config_.singleOpenBelow) {
        config_.singleHoldBelow = config_.singleOpenBelow;
    }
    if (config_.coContractionReleaseLevel > config_.coContractionLevel) {
        config_.coContractionReleaseLevel = config_.coContractionLevel;
    }
}

void ProportionalDrive::reset() {
    velocity_ = 0.0f;
    coTimer_ = 0.0f;
    coActive_ = false;
    coArmed_ = true;
    coEvent_ = false;
}

float ProportionalDrive::dualChannelVelocity(float flex, float ext) const {
    // Opposing channels subtract, so light background activity on the
    // antagonist bleeds off drive instead of fighting it at the servo.
    return flex * config_.closeSpeed - ext * config_.openSpeed;
}

float ProportionalDrive::singleChannelVelocity(float flex) const {
    if (flex < config_.singleOpenBelow) {
        // Relaxed: return to the open pose at a fixed rate.
        return -config_.openSpeed;
    }
    if (flex < config_.singleHoldBelow) {
        // True deadband — no creep in either direction.
        return 0.0f;
    }
    // Proportional close over the remaining span. Clamped, unlike the original.
    return mapClamped(flex, config_.singleHoldBelow, 1.0f, 0.0f, config_.closeSpeed);
}

void ProportionalDrive::updateCoContraction(float flex, float ext, float dt) {
    const bool bothHigh = (flex >= config_.coContractionLevel) && (ext >= config_.coContractionLevel);
    const bool bothLow = (flex <= config_.coContractionReleaseLevel) &&
                         (ext <= config_.coContractionReleaseLevel);

    if (bothLow) coArmed_ = true;

    if (bothHigh) {
        coTimer_ += dt;
        if (coTimer_ >= config_.coContractionSeconds) {
            if (coArmed_) {
                coEvent_ = true;
                coArmed_ = false;
            }
            coActive_ = true;
        }
    } else {
        coTimer_ = 0.0f;
        coActive_ = false;
    }
}

float ProportionalDrive::update(float flex, float ext, float dt) {
    flex = clamp01(flex);
    ext = clamp01(ext);

    if (config_.mode == DriveMode::DualChannel) {
        updateCoContraction(flex, ext, dt);
        // Freeze the hand while the gesture is being made; without this the
        // stronger channel wins and the hand lurches on every grip change.
        velocity_ = coActive_ ? 0.0f : dualChannelVelocity(flex, ext);
    } else {
        coActive_ = false;
        coTimer_ = 0.0f;
        velocity_ = singleChannelVelocity(flex);
    }
    return velocity_;
}

bool ProportionalDrive::consumeCoContraction() {
    const bool event = coEvent_;
    coEvent_ = false;
    return event;
}

}  // namespace jensen
