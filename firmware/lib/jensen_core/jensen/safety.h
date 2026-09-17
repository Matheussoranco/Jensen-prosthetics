// Jensen Hand — battery supervision, load protection and fault aggregation.
//
// A prosthesis is worn. Two failure modes matter more than anything else in
// this file: the hand losing power while it is holding something, and a stalled
// MG996R quietly heating up inside a closed forearm shell. Both are handled by
// degrading predictably rather than by cutting out.
#pragma once

#include <cstdint>
#include <limits>

#include "jensen/filter.h"
#include "jensen/util.h"

namespace jensen {

/// Fault bits. Multiple faults can be asserted at once; the supervisor decides
/// which of them are severe enough to remove servo torque.
enum FaultFlag : uint16_t {
    kFaultNone = 0,
    kFaultBatteryLow = 1u << 0,      ///< warn the user, keep operating
    kFaultBatteryCritical = 1u << 1, ///< park and remove torque
    kFaultServoBus = 1u << 2,        ///< PCA9685 not acknowledging on I2C
    kFaultEmgCalibration = 1u << 3,  ///< no usable EMG calibration stored
    kFaultDigitCalibration = 1u << 4,///< no usable servo endpoint calibration
    kFaultOverForce = 1u << 5,       ///< fingertip load above the hard cap
    kFaultSustainedLoad = 1u << 6,   ///< held at high force long enough to cook a servo
    kFaultLoopOverrun = 1u << 7,     ///< control loop missed its deadline
    kFaultSensorRange = 1u << 8,     ///< an input sat at rail — open or shorted
    kFaultMotorStall = 1u << 9,      ///< directly measured servo stall current
};

struct BatteryConfig {
    uint8_t cellCount = 2;              ///< 2S pack
    float warnCellVolts = 3.45f;
    float criticalCellVolts = 3.20f;
    float hysteresisVolts = 0.06f;      ///< per cell, to stop threshold chatter
    float smoothingSeconds = 1.5f;      ///< servo current draw makes the rail sag
};

/// Pack voltage monitoring with a Li-ion state-of-charge estimate.
class BatteryMonitor {
public:
    void configure(const BatteryConfig& config);
    void reset(float packVolts = 0.0f);

    void update(float packVolts, float dt);

    /// False until the first real sample seeds the filter. reset(0) —
    /// "no known voltage" — deliberately leaves the monitor unprimed
    /// instead of reporting a flat pack (which used to park the hand for
    /// seconds after every boot).
    bool valid() const { return smoothed_.primed(); }

    /// Volts, or NaN while !valid(). Check valid() before displaying.
    float packVolts() const {
        return smoothed_.primed() ? smoothed_.value()
                                  : std::numeric_limits<float>::quiet_NaN();
    }
    float cellVolts() const;
    /// Rough state of charge in [0, 1] from a resting Li-ion discharge curve.
    /// Under load this reads low; it is a fuel gauge for the user, not a
    /// control input.
    float stateOfCharge() const;

    bool low() const { return low_; }
    bool critical() const { return critical_; }

private:
    BatteryConfig config_{};
    Ema smoothed_{};
    bool low_ = false;
    bool critical_ = false;
};

struct HoldGuardConfig {
    float engageNewtons = 8.0f;    ///< load considered "working hard"
    float holdSeconds = 25.0f;     ///< how long that may be sustained
    float coolSeconds = 12.0f;     ///< recovery time once load drops
    float backoffScale = 0.65f;    ///< force target multiplier while backed off
};

/// Duty-cycle protection for the tendon servos.
///
/// Per-servo current IS sampled when fitted: main.cpp fills
/// `HandInputs::motorCurrentAmps[]` from `board::kPinMotorCurrent[]`
/// (see board_config.h; -1 = channel not fitted) and feeds
/// `MotorCurrentConfig`/kFaultMotorStall for direct stall detection.
/// HoldGuard is the complementary thermal/duty fallback: after a sustained
/// hold it reduces the force target rather than releasing the object
/// outright — a prosthesis that drops what it is carrying to protect
/// itself is worse than one that grips a little softer.
class HoldGuard {
public:
    void configure(const HoldGuardConfig& config);
    void reset();

    void update(float peakForceNewtons, float dt);

    bool backedOff() const { return backedOff_; }
    /// Multiplier to apply to the grip force setpoint (1.0 when not engaged).
    float forceScale() const { return backedOff_ ? config_.backoffScale : 1.0f; }
    float loadSeconds() const { return loadTimer_; }

private:
    HoldGuardConfig config_{};
    float loadTimer_ = 0.0f;
    float coolTimer_ = 0.0f;
    bool backedOff_ = false;
};

/// Watches control-loop timing. A prosthesis whose loop rate wanders has a
/// wandering feel, and every time constant in this firmware assumes the loop
/// actually runs when it says it does.
class LoopMonitor {
public:
    void configure(float nominalSeconds, float overrunFactor = 2.5f);
    void reset();

    /// Feed the measured interval between control iterations.
    void update(float dt);

    bool overrun() const { return overrun_; }
    float worstSeconds() const { return worst_; }
    float averageSeconds() const { return average_.value(); }
    uint32_t overrunCount() const { return overruns_; }

private:
    float nominal_ = 0.010f;
    float threshold_ = 0.025f;
    float worst_ = 0.0f;
    Ema average_{0.5f};
    bool overrun_ = false;
    uint32_t overruns_ = 0;
};

/// Aggregates fault sources and decides on the safe response.
class SafetySupervisor {
public:
    void reset();

    void set(FaultFlag flag, bool asserted);
    bool isSet(FaultFlag flag) const { return (faults_ & flag) != 0; }
    uint16_t faults() const { return faults_; }
    bool any() const { return faults_ != kFaultNone; }

    /// True when the hand must be parked and servo pulses removed.
    /// Immediate (no debounce): severe faults must cut torque at once.
    bool requiresSafeTorqueOff() const;
    /// Time-debounced variant: the severe condition must persist for
    /// `confirmMs` before reporting true (guards against a single bad
    /// sample parking the hand). `nowMs` comes from millis().
    bool requiresSafeTorqueOffDebounced(uint32_t nowMs, uint32_t confirmMs = 50) const;
    /// True when the hand may keep operating but the user should be told.
    bool requiresWarning() const;

    /// Human-readable name of the most severe asserted fault, or nullptr.
    const char* worstFaultName() const;

private:
    uint16_t faults_ = kFaultNone;
    // Debounce state for the time-guarded overload (mutable so the
    // immediate const query stays unchanged).
    mutable uint32_t severeSinceMs_ = 0;
    mutable bool severeTiming_ = false;
};

const char* faultName(FaultFlag flag);

}  // namespace jensen
