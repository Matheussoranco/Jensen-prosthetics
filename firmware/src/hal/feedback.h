// Jensen Hand — status LED and haptic feedback.
//
// Both are non-blocking pattern players stepped from the control loop. Nothing
// in the firmware is allowed to `delay()`: a hand that stops reading its
// electrodes to blink an LED is a hand that stutters under the user's control.
#pragma once

#include <cstdint>

#include "jensen/hand_state.h"

namespace jensen {

/// Eccentric-mass motor driven through a LEDC PWM channel.
///
/// A cheap vibration module has a stall threshold well above zero duty, so
/// "intensity" here is a duty cycle mapped into the range where the motor
/// actually turns rather than a linear 0–100%.
class Haptics {
public:
    void begin(int pin, uint8_t ledcChannel);

    void setEnabled(bool enabled);
    bool enabled() const { return enabled_; }

    /// Starts a cue, replacing whatever is playing. Cues are short and the
    /// user cannot distinguish an interrupted one from a queued one, so the
    /// most recent event always wins.
    void play(HapticCue cue);

    /// Steps the active pattern. Call every control iteration.
    void update(uint32_t nowMillis);

    bool busy() const { return step_ < stepCount_; }

private:
    void writeIntensity(uint8_t percent);

    int pin_ = -1;
    uint8_t channel_ = 0;
    bool enabled_ = true;

    struct Step {
        uint16_t millis;
        uint8_t intensity;  // 0–100
    };
    static constexpr uint8_t kMaxSteps = 8;
    Step steps_[kMaxSteps] = {};
    uint8_t stepCount_ = 0;
    uint8_t step_ = 0;
    uint32_t stepStartedMillis_ = 0;
};

/// Status LED. Carries the operating mode at a glance, which matters because
/// the display is usually facing away from the wearer.
class StatusLed {
public:
    void begin(int pin, uint8_t ledcChannel);

    void set(LedPattern pattern);
    LedPattern pattern() const { return pattern_; }

    void update(uint32_t nowMillis);

private:
    void writeBrightness(uint8_t percent);

    int pin_ = -1;
    uint8_t channel_ = 0;
    LedPattern pattern_ = LedPattern::Off;
    uint32_t patternStartedMillis_ = 0;
};

}  // namespace jensen
