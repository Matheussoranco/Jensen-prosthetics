// Jensen Hand — myoelectric input conditioning and the velocity control law.
//
// Relationship to HACKberry
// -------------------------
// HACKberry samples one sensor, rescales it between a calibrated min/max, and
// converts the result into a *velocity* that integrates into a hand position.
// That velocity-integration idea is sound and is kept here. Three things are
// changed:
//
//   1. Calibration stores a resting baseline and a noise floor separately from
//      the maximum, so a channel that drifts does not silently re-scale the
//      whole usable range.
//   2. The speed mapping is clamped. HACKberry's `map(v, 40, 100, 0, 6)` is
//      evaluated for inputs below 40 as well, so the region between its "zero"
//      threshold (30) and 40 produces a small *negative* speed — the hand
//      creeps open inside what reads like a deadband. Here the deadband is
//      exactly a deadband.
//   3. Timing is expressed in seconds rather than in loop iterations, so
//      changing the control rate does not change the feel of the hand.
#pragma once

#include <cstdint>

#include "jensen/filter.h"
#include "jensen/util.h"

namespace jensen {

/// Per-channel calibration, persisted to NVS between sessions.
struct EmgCalibration {
    float baselineCounts = 0.0f;  ///< mean ADC reading with the muscle at rest
    float noiseCounts = 0.0f;     ///< peak envelope observed at rest
    float mvcCounts = 0.0f;       ///< peak envelope at maximum voluntary contraction
    bool valid = false;

    /// A calibration is only usable if the contraction peak is clearly above
    /// the resting noise; otherwise the electrode is loose or misplaced and we
    /// would be amplifying noise straight into servo motion.
    bool isUsable() const;
};

struct EmgConfig {
    float attackSeconds = 0.030f;   ///< envelope rise constant
    float releaseSeconds = 0.150f;  ///< envelope fall constant
    float noiseMargin = 1.6f;       ///< floor = noiseCounts * noiseMargin
    float deadband = 0.08f;         ///< normalized activation ignored as noise
    float gamma = 1.35f;            ///< >1 gives finer control at low effort
};

/// One surface-EMG (or FSR/flex-sensor) input channel: raw counts in,
/// normalized 0..1 activation out.
class EmgChannel {
public:
    void configure(const EmgConfig& config);
    const EmgConfig& config() const { return config_; }

    void setCalibration(const EmgCalibration& calibration);
    const EmgCalibration& calibration() const { return calibration_; }

    /// Clears filter state but keeps calibration.
    void reset();

    // --- Calibration capture -------------------------------------------------
    // The application drives these: `beginCalibration()`, then a rest phase of
    // `observeRest()` calls, then a contraction phase of `observeEffort()`
    // calls, then `finishCalibration()`.

    void beginCalibration();
    void observeRest(float rawCounts, float dt);
    void observeEffort(float rawCounts, float dt);
    EmgCalibration finishCalibration();
    bool calibrating() const { return calibrating_; }

    // --- Runtime -------------------------------------------------------------

    /// Feed one sample; returns activation in [0, 1].
    float update(float rawCounts, float dt);

    float activation() const { return activation_; }
    float envelope() const { return envelope_.value(); }
    float raw() const { return raw_; }

    /// Normalized activation before deadband and gamma shaping — useful for
    /// tuning displays and for judging electrode placement.
    float normalized() const { return normalized_; }

private:
    float floorCounts() const;

    EmgConfig config_{};
    EmgCalibration calibration_{};
    Median3 despike_{};
    Envelope envelope_{};

    float raw_ = 0.0f;
    float normalized_ = 0.0f;
    float activation_ = 0.0f;

    bool calibrating_ = false;
    // Accumulators used only during calibration capture.
    double restSum_ = 0.0;
    uint32_t restCount_ = 0;
    float restPeakEnvelope_ = 0.0f;
    float effortPeakEnvelope_ = 0.0f;
};

/// How activation is turned into hand motion.
enum class DriveMode : uint8_t {
    /// Two electrodes: flexor closes, extensor opens, simultaneous strong
    /// contraction of both ("co-contraction") is a discrete command.
    DualChannel,
    /// One electrode, HACKberry-style: relaxed opens, light effort holds,
    /// firm effort closes proportionally.
    SingleChannel,
};

struct DriveConfig {
    DriveMode mode = DriveMode::DualChannel;

    /// Closure units per second at full activation. 1.0 == full travel/second.
    float closeSpeed = 2.2f;
    float openSpeed = 2.8f;

    /// Single-channel band edges, on activation in [0, 1].
    /// Below `singleOpenBelow` the hand opens at `openSpeed`; between that and
    /// `singleHoldBelow` it holds; above, it closes proportionally.
    float singleOpenBelow = 0.12f;
    float singleHoldBelow = 0.32f;

    /// Co-contraction gesture (dual-channel only).
    float coContractionLevel = 0.55f;
    float coContractionSeconds = 0.20f;
    /// Both channels must fall below this before the gesture can re-arm.
    float coContractionReleaseLevel = 0.30f;
};

/// Converts channel activations into an aperture velocity, and detects the
/// co-contraction gesture used to change grip pattern.
class ProportionalDrive {
public:
    void configure(const DriveConfig& config);
    const DriveConfig& config() const { return config_; }

    void reset();

    /// `flex` closes the hand, `ext` opens it. In single-channel mode only
    /// `flex` is used. Returns closure velocity in units per second, positive
    /// when closing.
    float update(float flex, float ext, float dt);

    float velocity() const { return velocity_; }

    /// True while both channels are held above the co-contraction level.
    bool coContracting() const { return coActive_; }

    /// Reads and clears the one-shot co-contraction event.
    bool consumeCoContraction();

private:
    float dualChannelVelocity(float flex, float ext) const;
    float singleChannelVelocity(float flex) const;
    void updateCoContraction(float flex, float ext, float dt);

    DriveConfig config_{};
    float velocity_ = 0.0f;
    float coTimer_ = 0.0f;
    bool coActive_ = false;
    bool coArmed_ = true;
    bool coEvent_ = false;
};

}  // namespace jensen
