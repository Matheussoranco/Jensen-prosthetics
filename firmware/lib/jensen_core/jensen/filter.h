// Jensen Hand — signal conditioning primitives.
//
// The original HACKberry firmware conditioned its single sensor channel by
// averaging ten consecutive `analogRead()` calls inside the control loop.
// That is a box filter with an undefined cutoff whose behaviour changes with
// loop timing. Here the primitives are explicit and time-parameterised, so a
// tuning value means the same thing regardless of how fast the loop runs.
#pragma once

#include <cstdint>

#include "jensen/util.h"

namespace jensen {

/// First-order exponential moving average with a time-constant expressed in
/// seconds, so the response is independent of the sample rate.
class Ema {
public:
    Ema() = default;
    explicit Ema(float timeConstantSeconds) { setTimeConstant(timeConstantSeconds); }

    void setTimeConstant(float seconds) { tau_ = seconds > 0.0f ? seconds : 0.0f; }
    float timeConstant() const { return tau_; }

    /// Seed the filter so the first sample does not produce a ramp from zero.
    void reset(float value = 0.0f) {
        value_ = value;
        primed_ = true;
    }

    /// Forget the seed, so the *next* sample becomes the starting value.
    /// Use this when the correct initial value is not known yet — priming a
    /// slow filter with a placeholder means the reading is wrong for several
    /// time constants, which for the battery monitor reads as a flat pack.
    void clear() {
        value_ = 0.0f;
        primed_ = false;
    }

    float update(float sample, float dt) {
        if (!primed_) {
            value_ = sample;
            primed_ = true;
            return value_;
        }
        if (tau_ <= 0.0f || dt <= 0.0f) {
            value_ = sample;
            return value_;
        }
        // alpha = dt / (tau + dt) is the exact zero-order-hold discretisation
        // of a one-pole low pass and is unconditionally stable for any dt.
        const float alpha = dt / (tau_ + dt);
        value_ += alpha * (sample - value_);
        return value_;
    }

    float value() const { return value_; }
    bool primed() const { return primed_; }

private:
    float tau_ = 0.05f;
    float value_ = 0.0f;
    bool primed_ = false;
};

/// Three-tap median, used to knock out single-sample ADC spikes (a common
/// artefact when servos switch and drag the rail) before low-pass filtering.
class Median3 {
public:
    float update(float sample) {
        a_ = b_;
        b_ = c_;
        c_ = sample;
        if (++filled_ > 3) filled_ = 3;
        if (filled_ < 3) return sample;
        // median of three without branching on all six orderings
        const float lo = a_ < b_ ? a_ : b_;
        const float hi = a_ < b_ ? b_ : a_;
        return c_ < lo ? lo : (c_ > hi ? hi : c_);
    }

    void reset() {
        a_ = b_ = c_ = 0.0f;
        filled_ = 0;
    }

private:
    float a_ = 0.0f, b_ = 0.0f, c_ = 0.0f;
    uint8_t filled_ = 0;
};

/// Asymmetric envelope follower: fast attack, slow release.
///
/// This is the standard way to turn a raw surface-EMG signal into a usable
/// intent signal — rectify about the resting baseline, then track the
/// rectified amplitude with a short attack constant (so the user's intent is
/// picked up quickly) and a longer release constant (so the hand does not
/// chatter when the burst decays).
class Envelope {
public:
    void configure(float attackSeconds, float releaseSeconds) {
        attack_ = attackSeconds > 0.0f ? attackSeconds : 0.0f;
        release_ = releaseSeconds > 0.0f ? releaseSeconds : 0.0f;
    }

    void setBaseline(float baseline) { baseline_ = baseline; }
    float baseline() const { return baseline_; }

    void reset(float value = 0.0f) { value_ = value; }

    float update(float sample, float dt) {
        const float rectified = absf(sample - baseline_);
        const float tau = (rectified > value_) ? attack_ : release_;
        if (tau <= 0.0f || dt <= 0.0f) {
            value_ = rectified;
        } else {
            value_ += (dt / (tau + dt)) * (rectified - value_);
        }
        return value_;
    }

    float value() const { return value_; }

private:
    float attack_ = 0.020f;
    float release_ = 0.120f;
    float baseline_ = 0.0f;
    float value_ = 0.0f;
};

/// Slew-rate limiter in engineering units per second.
class RateLimiter {
public:
    RateLimiter() = default;
    explicit RateLimiter(float unitsPerSecond) : rate_(unitsPerSecond) {}

    void setRate(float unitsPerSecond) { rate_ = unitsPerSecond > 0.0f ? unitsPerSecond : 0.0f; }
    float rate() const { return rate_; }

    void reset(float value) { value_ = value; }

    float update(float target, float dt) {
        value_ = approach(value_, target, rate_ * (dt > 0.0f ? dt : 0.0f));
        return value_;
    }

    float value() const { return value_; }

private:
    float rate_ = 1.0f;
    float value_ = 0.0f;
};

/// Comparator with hysteresis. Used for contact detection and for every
/// threshold in the mode logic, so a signal hovering at a threshold cannot
/// produce a burst of state transitions.
class Schmitt {
public:
    Schmitt() = default;
    Schmitt(float rising, float falling) { configure(rising, falling); }

    /// `rising` should be greater than `falling`; the arguments are ordered
    /// defensively so a swapped pair degrades to a plain comparator rather
    /// than latching.
    void configure(float rising, float falling) {
        if (falling > rising) {
            const float t = rising;
            rising = falling;
            falling = t;
        }
        rising_ = rising;
        falling_ = falling;
    }

    bool update(float value) {
        if (state_) {
            if (value < falling_) state_ = false;
        } else {
            if (value > rising_) state_ = true;
        }
        return state_;
    }

    bool state() const { return state_; }
    void reset(bool state = false) { state_ = state; }

private:
    float rising_ = 0.6f;
    float falling_ = 0.4f;
    bool state_ = false;
};

/// Filtered numerical derivative (units per second).
class Derivative {
public:
    void configure(float smoothingSeconds) { smooth_.setTimeConstant(smoothingSeconds); }

    void reset(float value = 0.0f) {
        previous_ = value;
        primed_ = false;
        smooth_.reset(0.0f);
    }

    float update(float value, float dt) {
        if (!primed_) {
            previous_ = value;
            primed_ = true;
            return 0.0f;
        }
        const float raw = (dt > 0.0f) ? (value - previous_) / dt : 0.0f;
        previous_ = value;
        return smooth_.update(raw, dt);
    }

    float value() const { return smooth_.value(); }

private:
    Ema smooth_{0.030f};
    float previous_ = 0.0f;
    bool primed_ = false;
};

}  // namespace jensen
