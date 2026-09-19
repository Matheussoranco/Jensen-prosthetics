// Jensen Hand — small numeric helpers shared by the control core.
//
// Header-only, freestanding: no Arduino, no dynamic allocation, no exceptions.
// Everything in `lib/jensen_core` must stay host-compilable so the control
// logic can be unit-tested without hardware (see `test/native`).
#pragma once

#include <cstdint>

namespace jensen {

/// Clamp `v` into the inclusive range [lo, hi].
///
/// NaN-safe: NaN can never satisfy a comparison, so the naive
/// `v < lo ? lo : (v > hi ? hi : v)` propagates NaN (clamp01(NaN) == NaN,
/// which then poisons digit closure / EMG activation / aperture). Here any
/// unordered input falls back to `lo`.
template <typename T>
constexpr T clamp(T v, T lo, T hi) {
    if (!(v >= lo)) return lo;  // covers v < lo and NaN
    if (!(v <= hi)) return hi;  // covers v > hi (+inf); NaN already returned
    return v;
}

/// Clamp to the unit interval — the range used for every normalized quantity
/// in this firmware (digit closure, EMG activation, grip aperture).
constexpr float clamp01(float v) {
    return clamp(v, 0.0f, 1.0f);
}

/// Linear interpolation. `t` is not clamped; use `lerpClamped` when it may
/// fall outside [0, 1].
constexpr float lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

constexpr float lerpClamped(float a, float b, float t) {
    return lerp(a, b, clamp01(t));
}

/// Arduino-style `map()` in floating point, without clamping.
///
/// Returns `outLo` for a degenerate input range rather than dividing by zero.
constexpr float mapRange(float x, float inLo, float inHi, float outLo, float outHi) {
    return (inHi == inLo) ? outLo
                          : outLo + (x - inLo) * (outHi - outLo) / (inHi - inLo);
}

/// `mapRange` with the result constrained to [outLo, outHi] regardless of
/// which endpoint is larger. This is the variant to reach for by default:
/// unclamped extrapolation is the source of the small "creep past the
/// deadband" quirk present in the original HACKberry sensor mapping.
constexpr float mapClamped(float x, float inLo, float inHi, float outLo, float outHi) {
    const float y = mapRange(x, inLo, inHi, outLo, outHi);
    return (outLo <= outHi) ? clamp(y, outLo, outHi) : clamp(y, outHi, outLo);
}

/// Move `current` toward `target` by at most `maxDelta`.
constexpr float approach(float current, float target, float maxDelta) {
    if (maxDelta <= 0.0f) return current;
    const float diff = target - current;
    if (diff > maxDelta) return current + maxDelta;
    if (diff < -maxDelta) return current - maxDelta;
    return target;
}

constexpr float absf(float v) {
    return v < 0.0f ? -v : v;
}

constexpr bool approxEqual(float a, float b, float eps = 1e-4f) {
    return absf(a - b) <= eps;
}

/// Integer-domain helper for servo pulse widths.
constexpr uint16_t clampU16(int32_t v, uint16_t lo, uint16_t hi) {
    if (v < static_cast<int32_t>(lo)) return lo;
    if (v > static_cast<int32_t>(hi)) return hi;
    return static_cast<uint16_t>(v);
}

}  // namespace jensen
