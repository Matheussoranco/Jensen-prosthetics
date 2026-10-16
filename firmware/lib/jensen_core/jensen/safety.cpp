#include "jensen/safety.h"

#include <limits>

namespace jensen {

// --- BatteryMonitor ----------------------------------------------------------

namespace {

/// Resting Li-ion discharge curve, cell volts -> state of charge.
/// Coarse on purpose: the pack sags under servo current and a finer curve
/// would imply an accuracy the measurement does not have.
struct SocPoint {
    float volts;
    float soc;
};

constexpr SocPoint kSocCurve[] = {
    {3.00f, 0.00f}, {3.30f, 0.05f}, {3.50f, 0.12f}, {3.60f, 0.22f},
    {3.70f, 0.38f}, {3.80f, 0.55f}, {3.90f, 0.70f}, {4.00f, 0.82f},
    {4.10f, 0.93f}, {4.20f, 1.00f},
};
constexpr uint8_t kSocPoints = sizeof(kSocCurve) / sizeof(kSocCurve[0]);

}  // namespace

void BatteryMonitor::configure(const BatteryConfig& config) {
    config_ = config;
    if (config_.cellCount == 0) config_.cellCount = 1;
    smoothed_.setTimeConstant(config_.smoothingSeconds);
}

void BatteryMonitor::reset(float packVolts) {
    // With no known starting voltage, leave the filter unprimed so the first
    // real sample seeds it. Priming a 1.5 s filter with 0 V would report a
    // critically flat pack — and park the hand — for the first seconds after
    // every boot.
    if (packVolts > 0.0f) {
        smoothed_.reset(packVolts);
    } else {
        smoothed_.clear();
    }
    low_ = false;
    critical_ = false;
}

void BatteryMonitor::update(float packVolts, float dt) {
    smoothed_.update(packVolts, dt);
    const float cell = cellVolts();

    // Thresholds latch with hysteresis so a servo inrush dip does not toggle
    // the hand in and out of a low-battery state several times a second.
    if (critical_) {
        if (cell > config_.criticalCellVolts + config_.hysteresisVolts) critical_ = false;
    } else if (cell < config_.criticalCellVolts) {
        critical_ = true;
    }

    if (low_) {
        if (cell > config_.warnCellVolts + config_.hysteresisVolts) low_ = false;
    } else if (cell < config_.warnCellVolts) {
        low_ = true;
    }
}

float BatteryMonitor::cellVolts() const {
    if (!smoothed_.primed()) return std::numeric_limits<float>::quiet_NaN();
    return smoothed_.value() / static_cast<float>(config_.cellCount);
}

float BatteryMonitor::stateOfCharge() const {
    if (!smoothed_.primed()) return std::numeric_limits<float>::quiet_NaN();
    const float v = cellVolts();
    if (v <= kSocCurve[0].volts) return 0.0f;
    if (v >= kSocCurve[kSocPoints - 1].volts) return 1.0f;
    for (uint8_t i = 1; i < kSocPoints; ++i) {
        if (v <= kSocCurve[i].volts) {
            return mapRange(v, kSocCurve[i - 1].volts, kSocCurve[i].volts, kSocCurve[i - 1].soc,
                            kSocCurve[i].soc);
        }
    }
    return 1.0f;
}

// --- HoldGuard ---------------------------------------------------------------

void HoldGuard::configure(const HoldGuardConfig& config) {
    config_ = config;
    config_.backoffScale = clamp(config_.backoffScale, 0.1f, 1.0f);
}

void HoldGuard::reset() {
    loadTimer_ = 0.0f;
    coolTimer_ = 0.0f;
    backedOff_ = false;
}

void HoldGuard::update(float peakForceNewtons, float dt) {
    if (dt <= 0.0f) return;

    if (peakForceNewtons >= config_.engageNewtons) {
        loadTimer_ += dt;
        coolTimer_ = 0.0f;
        if (loadTimer_ >= config_.holdSeconds) backedOff_ = true;
    } else {
        coolTimer_ += dt;
        // Bleed the accumulated load time off at real time, so alternating
        // grips does not let the servos accumulate heat indefinitely.
        loadTimer_ -= dt;
        if (loadTimer_ < 0.0f) loadTimer_ = 0.0f;
        if (backedOff_ && coolTimer_ >= config_.coolSeconds) {
            backedOff_ = false;
            loadTimer_ = 0.0f;
        }
    }
}

// --- LoopMonitor -------------------------------------------------------------

void LoopMonitor::configure(float nominalSeconds, float overrunFactor) {
    nominal_ = nominalSeconds > 0.0f ? nominalSeconds : 0.010f;
    if (overrunFactor < 1.1f) overrunFactor = 1.1f;
    threshold_ = nominal_ * overrunFactor;
    average_.setTimeConstant(0.5f);
}

void LoopMonitor::reset() {
    worst_ = 0.0f;
    overruns_ = 0;
    overrun_ = false;
    average_.reset(nominal_);
}

void LoopMonitor::update(float dt) {
    if (dt <= 0.0f) return;
    average_.update(dt, dt);
    if (dt > worst_) worst_ = dt;
    overrun_ = dt > threshold_;
    if (overrun_) ++overruns_;
}

// --- SafetySupervisor --------------------------------------------------------

void SafetySupervisor::reset() { faults_ = kFaultNone; severeSinceMs_ = 0; severeTiming_ = false; }

void SafetySupervisor::set(FaultFlag flag, bool asserted) {
    if (asserted) {
        faults_ = static_cast<uint16_t>(faults_ | flag);
    } else {
        faults_ = static_cast<uint16_t>(faults_ & ~static_cast<uint16_t>(flag));
    }
}

bool SafetySupervisor::requiresSafeTorqueOff() const {
    // Faults where continuing to drive servos is either useless or unsafe.
    // Over-force removes torque from the whole hand. A directly measured
    // motor stall is isolated by MotorCurrentGuard and updateOutputs per
    // digit; powering down the other digits would release a held object.
    constexpr uint16_t kSevere = kFaultBatteryCritical | kFaultServoBus | kFaultDigitCalibration |
                                 kFaultOverForce;
    return (faults_ & kSevere) != 0;
}

bool SafetySupervisor::requiresSafeTorqueOffDebounced(uint32_t nowMs, uint32_t confirmMs) const {
    if (!requiresSafeTorqueOff()) {
        severeTiming_ = false;
        severeSinceMs_ = nowMs;
        return false;
    }
    if (!severeTiming_) {
        severeTiming_ = true;
        severeSinceMs_ = nowMs;
    }
    return (nowMs - severeSinceMs_) >= confirmMs;
}

bool SafetySupervisor::requiresWarning() const {
    return any() && !requiresSafeTorqueOff();
}

const char* faultName(FaultFlag flag) {
    switch (flag) {
        case kFaultBatteryLow:       return "battery_low";
        case kFaultBatteryCritical:  return "battery_critical";
        case kFaultServoBus:         return "servo_bus";
        case kFaultEmgCalibration:   return "emg_uncalibrated";
        case kFaultDigitCalibration: return "digits_uncalibrated";
        case kFaultOverForce:        return "over_force";
        case kFaultSustainedLoad:    return "sustained_load";
        case kFaultLoopOverrun:      return "loop_overrun";
        case kFaultSensorRange:      return "sensor_range";
        case kFaultMotorStall:       return "motor_stall";
        case kFaultNone:             return "none";
        default:                     return "unknown";
    }
}

const char* SafetySupervisor::worstFaultName() const {
    // Ordered most to least severe.
    constexpr FaultFlag kOrder[] = {
        kFaultBatteryCritical, kFaultServoBus,       kFaultDigitCalibration,
        kFaultEmgCalibration,  kFaultOverForce,      kFaultSustainedLoad,
        kFaultMotorStall,      kFaultBatteryLow,      kFaultSensorRange,
        kFaultLoopOverrun,
    };
    for (FaultFlag flag : kOrder) {
        if (isSet(flag)) return faultName(flag);
    }
    return nullptr;
}

}  // namespace jensen
