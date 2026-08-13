// Jensen Hand — per-digit motion control.
//
// Every digit is described in a single normalized coordinate, "closure":
//
//     0.0 = fully open / extended        1.0 = fully closed / flexed
//
// Servo geometry lives entirely in `DigitCalibration`, so the rest of the
// firmware never has to know which way a given servo horn turns, whether the
// hand is a left or a right, or where the mechanical hard stops are. This is
// the main structural difference from HACKberry, which carries left/right
// handedness and open/close endpoint swapping through the whole control path.
#pragma once

#include <cstdint>

#include "jensen/util.h"

namespace jensen {

enum class Digit : uint8_t {
    ThumbFlex = 0,  ///< thumb MCP/IP flexion tendon
    ThumbRot,       ///< thumb CMC opposition (rotates the thumb across the palm)
    Index,
    Middle,
    Ring,
    Pinky,
    Wrist,  ///< optional forearm/wrist rotator, not part of a grasp pose
    Count
};

constexpr uint8_t kDigitCount = static_cast<uint8_t>(Digit::Count);
/// Digits that participate in a grasp pose (everything except the wrist).
constexpr uint8_t kGraspDigitCount = 6;

constexpr uint8_t index(Digit d) { return static_cast<uint8_t>(d); }

const char* digitName(Digit d);

/// Maps normalized closure onto a servo pulse width.
///
/// `openUs` and `closedUs` may be in either order — that ordering *is* the
/// direction of travel. `minUs`/`maxUs` are absolute mechanical guards that
/// are enforced after interpolation, so a bad calibration cannot drive a
/// tendon past its hard stop.
struct DigitCalibration {
    uint16_t openUs = 1500;
    uint16_t closedUs = 1500;
    uint16_t minUs = 700;
    uint16_t maxUs = 2300;
    bool enabled = false;

    /// Travel must be meaningful, and both endpoints inside the guard band.
    bool isValid() const;

    /// Pulse width for a closure value, guard-band clamped.
    uint16_t pulseFor(float closure) const;

    uint16_t guardLowUs() const { return minUs < maxUs ? minUs : maxUs; }
    uint16_t guardHighUs() const { return minUs < maxUs ? maxUs : minUs; }
};

/// Slew-rate limited closure controller for one digit.
class DigitController {
public:
    void setCalibration(const DigitCalibration& calibration);
    const DigitCalibration& calibration() const { return calibration_; }

    /// Full-travel rate limit, in closure units per second.
    /// 2.0 means the digit sweeps open-to-closed in 500 ms.
    void setSpeed(float closurePerSecond);
    float speed() const { return speed_; }

    /// Commanded closure. Clamped to [0, 1] and to the active travel limit.
    void setTarget(float closure);
    float target() const { return target_; }

    /// Upper bound on closure imposed by the grip-force loop. Applied to the
    /// *command*, so releasing the limit lets the digit resume closing.
    void setClosureLimit(float limit);
    float closureLimit() const { return closureLimit_; }

    /// A locked digit holds its current closure and ignores new targets.
    void setLocked(bool locked) { locked_ = locked; }
    bool locked() const { return locked_; }

    /// Jump to a closure without rate limiting. Only for homing and bench
    /// calibration, never from the control loop.
    void snapTo(float closure);

    void update(float dt);

    float closure() const { return closure_; }
    bool settled() const;

    /// Servo pulse in microseconds, or 0 when the output is released
    /// (no pulse — the servo is unpowered and free).
    uint16_t pulseUs() const;

    /// Release the output once the digit has been open and idle for a while.
    /// Saves current and stops MG996R hunting/buzzing at the open stop; never
    /// engages while the hand is closed on an object.
    void setIdleRelease(bool enabled, float belowClosure = 0.03f, float afterSeconds = 1.5f);
    bool released() const { return released_; }

private:
    float effectiveTarget() const;

    DigitCalibration calibration_{};
    float speed_ = 2.0f;
    float target_ = 0.0f;
    float closure_ = 0.0f;
    float closureLimit_ = 1.0f;
    bool locked_ = false;

    bool idleReleaseEnabled_ = false;
    float idleReleaseBelow_ = 0.03f;
    float idleReleaseAfter_ = 1.5f;
    float idleTimer_ = 0.0f;
    bool released_ = false;
};

}  // namespace jensen
