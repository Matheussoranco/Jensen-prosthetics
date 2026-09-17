// Jensen Hand — fingertip force sensing and the grip-force closed loop.
//
// HACKberry is open loop: the sensor commands a position and the servo drives
// to it regardless of what the hand is holding. That works for a hand with one
// tendon and a compliant printed finger, but it means grip force is whatever
// the mechanics happen to produce, and a slipping object is never noticed.
//
// The Jensen hardware carries FSR400 fingertip sensors, so the closure command
// can be closed around measured load instead:
//
//   * once fingertip force reaches the target, closure is held there rather
//     than driven further into a stall — this is what keeps an MG996R from
//     cooking itself against an object it cannot crush;
//   * a sudden drop in force while in contact means the object is escaping,
//     and the hand tightens a step in response.
#pragma once

#include <cstdint>

#include "jensen/filter.h"
#include "jensen/util.h"

namespace jensen {

/// FSR400 conditioning constants.
///
/// An FSR is not a load cell — its conductance is roughly proportional to
/// applied force over the mid-range and it is markedly non-linear at both
/// ends. The two-point fit below (zero reading plus one known reference load)
/// with an adjustable exponent is deliberately modest: it gives a repeatable,
/// monotonic number in newtons that is good enough to regulate against, and it
/// is honest about not being a calibrated force measurement.
struct ForceCalibration {
    uint16_t zeroCounts = 60;         ///< reading with no load applied
    uint16_t spanCounts = 2600;       ///< reading at `referenceNewtons`
    float referenceNewtons = 20.0f;   ///< load used to capture `spanCounts`
    float exponent = 0.85f;           ///< 1.0 = linear; FSR400 sits near 0.8–0.9
    bool present = false;             ///< sensor fitted on this digit

    /// Newtons for a raw ADC reading. Returns 0 below the zero point.
    float newtons(float rawCounts) const;
    bool isValid() const;
};

struct ForceSensorConfig {
    float smoothingSeconds = 0.020f;    ///< low-pass on the force estimate
    float derivativeSeconds = 0.030f;   ///< smoothing on dF/dt
    float contactOnNewtons = 1.0f;      ///< Schmitt rising edge
    float contactOffNewtons = 0.5f;     ///< Schmitt falling edge
    float slipRateNewtonsPerSec = -30.0f;  ///< dF/dt below this while in contact
};

/// One fingertip force channel.
class ForceSensor {
public:
    /// Samples averaged by tare() when capturing the unloaded zero.
    static constexpr uint8_t kTareSamples = 8;

    void configure(const ForceSensorConfig& config);
    void setCalibration(const ForceCalibration& calibration);
    const ForceCalibration& calibration() const { return calibration_; }

    void reset();

    /// Feed one raw ADC sample; returns the filtered force in newtons.
    float update(float rawCounts, float dt);

    float force() const { return force_; }
    float rate() const { return rate_; }
    float raw() const { return raw_; }
    bool present() const { return calibration_.present; }
    bool contact() const { return contact_.state(); }

    /// Reads and clears the one-shot slip event.
    bool consumeSlip();

    /// Capture the unloaded zero as the MEAN of the last kTareSamples raw
    /// readings (not a single sample: one ADC spike would bias every later
    /// newton). Called at boot and from the CLI; FSR zero drifts with
    /// temperature and with how the fingertip pad has been compressed.
    /// Unlike reset(), a pending slip event survives tare() — re-zeroing
    /// must not swallow an in-flight slip report.
    void tare();

private:
    ForceSensorConfig config_{};
    ForceCalibration calibration_{};
    Ema smooth_{};
    Derivative derivative_{};
    Schmitt contact_{};

    float raw_ = 0.0f;
    float force_ = 0.0f;
    float rate_ = 0.0f;
    bool slipEvent_ = false;

    /// Ring of recent raw readings feeding tare(). Written by update().
    float rawHistory_[kTareSamples] = {};
    uint8_t rawHistoryCount_ = 0;
    uint8_t rawHistoryHead_ = 0;
};

struct GripForceConfig {
    float targetNewtons = 12.0f;   ///< regulation setpoint
    float maxNewtons = 22.0f;      ///< above this the supervisor raises a fault
    float integralGain = 0.05f;    ///< closure units per newton-second
    float proportionalGain = 0.01f;///< closure units per newton of error *change*
    float slipTightenStep = 0.05f; ///< closure added immediately on a slip
    float releasePerSecond = 1.5f; ///< how fast the limit relaxes with no contact
    float minLimit = 0.05f;        ///< never clamp the hand fully open
    float windupMargin = 0.08f;    ///< limit is never left far above actual closure
};

/// Regulates how far the hand is allowed to close, given fingertip load.
///
/// The output is an upper bound on closure rather than a closure command, so
/// the user always retains authority to open, and releasing the object simply
/// lets the bound relax back to 1.0.
class GripForceController {
public:
    void configure(const GripForceConfig& config);
    const GripForceConfig& config() const { return config_; }

    void reset();

    /// Per-grip force scaling — a pinch should not squeeze as hard as a power
    /// grasp. 1.0 leaves `targetNewtons` unchanged.
    void setTargetScale(float scale);
    float targetNewtons() const { return config_.targetNewtons * targetScale_; }

    /// `sensors` is the fingertip array; `currentClosure` is where the hand
    /// actually is, used for anti-windup.
    void update(ForceSensor* sensors, uint8_t count, float currentClosure, float dt);

    float closureLimit() const { return limit_; }
    float peakForce() const { return peakForce_; }
    bool contact() const { return contact_; }
    bool overForce() const { return overForce_; }
    uint32_t slipCount() const { return slipCount_; }

private:
    GripForceConfig config_{};
    float targetScale_ = 1.0f;
    float limit_ = 1.0f;
    float peakForce_ = 0.0f;
    bool contact_ = false;
    bool overForce_ = false;
    uint32_t slipCount_ = 0;
    /// Previous error for the incremental (velocity-form) PI law.
    float prevError_ = 0.0f;
};

}  // namespace jensen
