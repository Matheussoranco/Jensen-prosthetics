// Jensen Hand - per-motor current supervision.
#pragma once

#include <cstdint>

#include "jensen/digit.h"

namespace jensen {

struct MotorCurrentConfig {
    /// Bit i is set when digit i has a current channel fitted and calibrated.
    uint8_t presentMask = 0;
    /// Current above this level while position error persists is a stall.
    float tripAmps = 2.2f;
    /// Current must fall below this level before automatic recovery begins.
    float clearAmps = 0.45f;
    float tripSeconds = 0.25f;
    float clearSeconds = 1.0f;
};

/// Latches a single servo off when directly measured current indicates a stall.
/// The other digits remain available, and the channel automatically recovers
/// only after current has stayed low for the configured cooldown.
class MotorCurrentGuard {
public:
    void configure(const MotorCurrentConfig& config, bool present);
    void reset();
    void update(float amps, bool energised, float dt);

    float amps() const { return amps_; }
    bool present() const { return present_; }
    bool stalled() const { return stalled_; }
    float highCurrentSeconds() const { return highTimer_; }
    uint32_t stallCount() const { return stallCount_; }

private:
    MotorCurrentConfig config_{};
    bool present_ = false;
    bool stalled_ = false;
    float amps_ = 0.0f;
    float highTimer_ = 0.0f;
    float clearTimer_ = 0.0f;
    uint32_t stallCount_ = 0;
};

}  // namespace jensen
