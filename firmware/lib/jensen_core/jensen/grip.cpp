#include "jensen/grip.h"

namespace jensen {

namespace {

// Pose literals are written in Digit order:
//   { thumbFlex, thumbRot, index, middle, ring, pinky }
//
// thumbRot is opposition: 0.0 places the thumb alongside the index (adducted,
// the "key grip" plane), ~0.85 places it across the palm facing the fingertips.
// Values were chosen to be mechanically conservative — none of them command a
// digit to its hard stop, leaving the per-hand endpoint calibration room to
// take up build variation.

constexpr GripPattern kPatterns[kGripCount] = {
    // Rest — open == closed, so aperture has no effect. This is the posture
    // the hand parks in when idle or faulted.
    {"rest", "Relaxed neutral posture",
     {{0.08f, 0.35f, 0.12f, 0.14f, 0.16f, 0.18f}},
     {{0.08f, 0.35f, 0.12f, 0.14f, 0.16f, 0.18f}}},

    // Power — the default. Thumb stays opposed while all four fingers wrap.
    {"power", "Cylindrical whole-hand wrap",
     {{0.10f, 0.55f, 0.00f, 0.00f, 0.00f, 0.00f}},
     {{0.90f, 0.58f, 1.00f, 1.00f, 1.00f, 1.00f}}},

    // Pinch — thumb fully opposed to the index; the ulnar three digits hold a
    // light curl so they stay clear of the pinch axis.
    {"pinch", "Thumb-to-index tip pinch",
     {{0.15f, 0.85f, 0.00f, 0.18f, 0.18f, 0.18f}},
     {{0.70f, 0.85f, 0.78f, 0.22f, 0.22f, 0.22f}}},

    // Tripod — pinch plus the middle finger, for rounder objects.
    {"tripod", "Thumb, index and middle",
     {{0.15f, 0.80f, 0.00f, 0.00f, 0.22f, 0.22f}},
     {{0.72f, 0.80f, 0.75f, 0.75f, 0.26f, 0.26f}}},

    // Lateral / key — fingers park closed, thumb rotates into the index plane
    // and flexes down onto it. Only thumbFlex responds to aperture.
    {"key", "Thumb against the side of a closed index",
     {{0.10f, 0.05f, 0.85f, 0.90f, 0.90f, 0.90f}},
     {{0.80f, 0.05f, 0.85f, 0.90f, 0.90f, 0.90f}}},

    // Point — index pinned at full extension across the whole aperture range.
    {"point", "Index extended for buttons and touchscreens",
     {{0.20f, 0.30f, 0.00f, 0.60f, 0.60f, 0.60f}},
     {{0.85f, 0.30f, 0.00f, 1.00f, 1.00f, 1.00f}}},

    // Hook — carrying handles. Thumb is parked fully open and out of the load
    // path so it is not back-driven by the weight.
    {"hook", "Four-finger carrying hook",
     {{0.00f, 0.00f, 0.15f, 0.15f, 0.15f, 0.15f}},
     {{0.00f, 0.00f, 1.00f, 1.00f, 1.00f, 1.00f}}},
};

constexpr uint8_t gripIndex(GripId grip) {
    return static_cast<uint8_t>(grip) < kGripCount ? static_cast<uint8_t>(grip) : 0;
}

}  // namespace

const GripPattern& GripLibrary::pattern(GripId grip) {
    return kPatterns[gripIndex(grip)];
}

const char* GripLibrary::name(GripId grip) {
    return kPatterns[gripIndex(grip)].name;
}

GraspPose GripLibrary::pose(GripId grip, float aperture) {
    const GripPattern& p = pattern(grip);
    const float t = clamp01(aperture);
    GraspPose out{};
    for (uint8_t i = 0; i < kGraspDigitCount; ++i) {
        out.closure[i] = lerp(p.open.closure[i], p.closed.closure[i], t);
    }
    return out;
}

GraspPose GripLibrary::blend(GripId from, GripId to, float t, float aperture) {
    const GraspPose a = pose(from, aperture);
    const GraspPose b = pose(to, aperture);
    const float mix = clamp01(t);
    GraspPose out{};
    for (uint8_t i = 0; i < kGraspDigitCount; ++i) {
        out.closure[i] = lerp(a.closure[i], b.closure[i], mix);
    }
    return out;
}

// --- GripSelector ------------------------------------------------------------

namespace {
constexpr uint16_t bitFor(GripId grip) {
    return static_cast<uint16_t>(1u << static_cast<uint8_t>(grip));
}
constexpr uint16_t kRestBit = bitFor(GripId::Rest);
}  // namespace

GripSelector::GripSelector() {
    // A sensible starting set: everything except the two most situational
    // patterns, which the user can enable from the display or the CLI.
    setEnabledMask(bitFor(GripId::Rest) | bitFor(GripId::Power) | bitFor(GripId::Pinch) |
                   bitFor(GripId::Tripod) | bitFor(GripId::LateralKey));
    current_ = GripId::Power;
}

void GripSelector::setEnabled(GripId grip, bool enabled) {
    if (static_cast<uint8_t>(grip) >= kGripCount) return;
    if (enabled) {
        mask_ = static_cast<uint16_t>(mask_ | bitFor(grip));
    } else {
        mask_ = static_cast<uint16_t>(mask_ & ~bitFor(grip));
    }
    setEnabledMask(mask_);
}

bool GripSelector::enabled(GripId grip) const {
    if (static_cast<uint8_t>(grip) >= kGripCount) return false;
    return (mask_ & bitFor(grip)) != 0;
}

void GripSelector::setEnabledMask(uint16_t mask) {
    // Rest is always available: it is the pose the hand falls back to on a
    // fault, and a fully empty mask would leave the selector with nothing to
    // point at.
    mask_ = static_cast<uint16_t>((mask & ((1u << kGripCount) - 1u)) | kRestBit);
    if (!enabled(current_)) current_ = GripId::Rest;
}

void GripSelector::select(GripId grip) {
    if (enabled(grip)) current_ = grip;
}

GripId GripSelector::step(int direction) {
    const int count = static_cast<int>(kGripCount);
    int i = static_cast<int>(current_);
    for (int attempt = 0; attempt < count; ++attempt) {
        i = (i + direction + count) % count;
        const GripId candidate = static_cast<GripId>(i);
        if (enabled(candidate)) {
            current_ = candidate;
            break;
        }
    }
    return current_;
}

GripId GripSelector::next() { return step(1); }
GripId GripSelector::previous() { return step(-1); }

uint8_t GripSelector::enabledCount() const {
    uint8_t n = 0;
    for (uint8_t i = 0; i < kGripCount; ++i) {
        if (mask_ & (1u << i)) ++n;
    }
    return n;
}

}  // namespace jensen
