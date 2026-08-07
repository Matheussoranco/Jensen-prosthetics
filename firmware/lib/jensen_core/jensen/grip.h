// Jensen Hand — grasp pattern library.
//
// A single scalar "aperture" command (0 = open, 1 = closed) drives the whole
// hand. What each digit *does* with that command is what distinguishes one
// grip from another: in a power grasp all four fingers sweep the full range,
// in a key grip they stay curled and only the thumb moves, when pointing the
// index never leaves full extension.
//
// Each pattern is therefore stored as two poses — the shape at aperture 0 and
// the shape at aperture 1 — and the commanded pose is the interpolation
// between them. Digits with identical open and closed values simply hold a
// posture, which is how "out of the way" and "pre-positioned" digits are
// expressed without any special-casing in the control loop.
#pragma once

#include <cstdint>

#include "jensen/digit.h"

namespace jensen {

enum class GripId : uint8_t {
    Rest = 0,    ///< relaxed cosmetic posture; does not respond to aperture
    Power,       ///< cylindrical / whole-hand wrap
    Pinch,       ///< thumb-to-index tip pinch
    Tripod,      ///< thumb + index + middle
    LateralKey,  ///< thumb adducted against the side of a closed index
    Point,       ///< index extended, remaining digits closed
    Hook,        ///< four-finger hook, thumb parked open
    Count
};

constexpr uint8_t kGripCount = static_cast<uint8_t>(GripId::Count);

/// Closure values for the six grasping digits, indexed by `Digit`.
/// (`Digit::Wrist` is deliberately excluded — it is commanded separately.)
struct GraspPose {
    float closure[kGraspDigitCount] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    float operator[](Digit d) const { return closure[index(d)]; }
    float& operator[](Digit d) { return closure[index(d)]; }
};

struct GripPattern {
    const char* name;
    const char* description;
    GraspPose open;    ///< shape at aperture 0
    GraspPose closed;  ///< shape at aperture 1
};

/// Static, ROM-resident table of grasp patterns.
class GripLibrary {
public:
    static const GripPattern& pattern(GripId grip);
    static const char* name(GripId grip);

    /// Pose for `grip` at the given aperture, per-digit interpolated.
    static GraspPose pose(GripId grip, float aperture);

    /// Blend between two grips — used to morph smoothly when the grip changes
    /// mid-motion instead of snapping the servos to a new shape.
    static GraspPose blend(GripId from, GripId to, float t, float aperture);
};

/// Tracks the active grip and cycles through the subset the user has enabled.
///
/// Users typically settle on two or three patterns; cycling through all seven
/// on every co-contraction makes the hand tedious to operate, so unwanted
/// patterns can be masked out and are then skipped entirely.
class GripSelector {
public:
    GripSelector();

    void setEnabled(GripId grip, bool enabled);
    bool enabled(GripId grip) const;

    /// Bitmask of enabled grips, for persistence. `Rest` is always forced on
    /// so there is a guaranteed fallback pose.
    void setEnabledMask(uint16_t mask);
    uint16_t enabledMask() const { return mask_; }

    void select(GripId grip);
    GripId current() const { return current_; }

    /// Advance to the next/previous enabled grip. No-ops if only one is
    /// enabled; never returns a disabled grip.
    GripId next();
    GripId previous();

    uint8_t enabledCount() const;

private:
    GripId step(int direction);

    uint16_t mask_ = 0;
    GripId current_ = GripId::Power;
};

}  // namespace jensen
