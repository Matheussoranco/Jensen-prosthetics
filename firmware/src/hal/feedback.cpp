#include "hal/feedback.h"

#include <Arduino.h>

#include "board_config.h"

namespace jensen {

namespace {

constexpr uint16_t kLedcMaxDuty = (1u << board::kLedcResolutionBits) - 1u;

/// A 5 V eccentric-mass module needs roughly a third of full duty before it
/// turns at all, so "1%" of a linear scale would be silence.
constexpr uint8_t kHapticStallPercent = 35;

uint32_t hapticDuty(uint8_t percent) {
    if (percent == 0) return 0;
    const uint32_t span = 100u - kHapticStallPercent;
    const uint32_t scaled = kHapticStallPercent + (static_cast<uint32_t>(percent) * span) / 100u;
    return (scaled * kLedcMaxDuty) / 100u;
}

}  // namespace

// --- Haptics -----------------------------------------------------------------

void Haptics::begin(int pin, uint8_t ledcChannel) {
    pin_ = pin;
    channel_ = ledcChannel;
    ledcSetup(channel_, board::kLedcFrequencyHz, board::kLedcResolutionBits);
    ledcAttachPin(pin_, channel_);
    writeIntensity(0);
    stepCount_ = 0;
    step_ = 0;
}

void Haptics::setEnabled(bool enabled) {
    enabled_ = enabled;
    if (!enabled_) {
        stepCount_ = 0;
        step_ = 0;
        writeIntensity(0);
    }
}

void Haptics::writeIntensity(uint8_t percent) {
    if (pin_ < 0) return;
    ledcWrite(channel_, hapticDuty(percent));
}

void Haptics::play(HapticCue cue) {
    if (!enabled_ || pin_ < 0 || cue == HapticCue::None) return;

    stepCount_ = 0;
    auto push = [this](uint16_t ms, uint8_t intensity) {
        if (stepCount_ < kMaxSteps) steps_[stepCount_++] = Step{ms, intensity};
    };

    switch (cue) {
        case HapticCue::Tick:
            push(40, 70);
            break;
        case HapticCue::DoubleTick:
            push(40, 70);
            push(60, 0);
            push(40, 70);
            break;
        case HapticCue::Contact:
            // Short and soft: this fires on every grasp, so it has to be
            // informative without becoming irritating.
            push(25, 45);
            break;
        case HapticCue::Slip:
            // Sharp triple burst — the one cue the user must react to.
            push(35, 100);
            push(35, 0);
            push(35, 100);
            push(35, 0);
            push(35, 100);
            break;
        case HapticCue::Warning:
            push(400, 85);
            break;
        case HapticCue::Confirm:
            push(120, 60);
            break;
        case HapticCue::None:
        default:
            return;
    }

    step_ = 0;
    stepStartedMillis_ = millis();
    writeIntensity(steps_[0].intensity);
}

void Haptics::update(uint32_t nowMillis) {
    if (pin_ < 0 || step_ >= stepCount_) return;

    if ((nowMillis - stepStartedMillis_) < steps_[step_].millis) return;

    ++step_;
    stepStartedMillis_ = nowMillis;
    if (step_ >= stepCount_) {
        stepCount_ = 0;
        writeIntensity(0);
        return;
    }
    writeIntensity(steps_[step_].intensity);
}

// --- StatusLed ---------------------------------------------------------------

void StatusLed::begin(int pin, uint8_t ledcChannel) {
    pin_ = pin;
    channel_ = ledcChannel;
    ledcSetup(channel_, board::kLedcFrequencyHz, board::kLedcResolutionBits);
    ledcAttachPin(pin_, channel_);
    writeBrightness(0);
    patternStartedMillis_ = millis();
}

void StatusLed::set(LedPattern pattern) {
    if (pattern == pattern_) return;
    pattern_ = pattern;
    patternStartedMillis_ = millis();
}

void StatusLed::writeBrightness(uint8_t percent) {
    if (pin_ < 0) return;
    ledcWrite(channel_, (static_cast<uint32_t>(percent) * kLedcMaxDuty) / 100u);
}

void StatusLed::update(uint32_t nowMillis) {
    if (pin_ < 0) return;
    const uint32_t elapsed = nowMillis - patternStartedMillis_;

    switch (pattern_) {
        case LedPattern::Off:
            writeBrightness(0);
            break;

        case LedPattern::Solid:
            writeBrightness(100);
            break;

        case LedPattern::Breathe: {
            // Triangle ramp over 3 s. Cheaper than a sine and, on a 5 mm LED
            // through a diffuser, indistinguishable.
            const uint32_t phase = elapsed % 3000;
            const uint32_t level = phase < 1500 ? phase : (3000 - phase);
            writeBrightness(static_cast<uint8_t>(5 + (level * 45) / 1500));
            break;
        }

        case LedPattern::SlowBlink:
            writeBrightness((elapsed % 1600) < 800 ? 100 : 0);
            break;

        case LedPattern::FastBlink:
            writeBrightness((elapsed % 300) < 150 ? 100 : 0);
            break;

        case LedPattern::DoubleBlink: {
            const uint32_t phase = elapsed % 1600;
            const bool on = (phase < 120) || (phase >= 260 && phase < 380);
            writeBrightness(on ? 100 : 0);
            break;
        }

        case LedPattern::SosBlink: {
            // Three short, three long, three short, then a pause.
            const uint32_t phase = elapsed % 4200;
            bool on = false;
            if (phase < 900) {
                on = (phase % 300) < 150;
            } else if (phase < 2400) {
                on = ((phase - 900) % 500) < 350;
            } else if (phase < 3300) {
                on = ((phase - 2400) % 300) < 150;
            }
            writeBrightness(on ? 100 : 0);
            break;
        }

        default:
            writeBrightness(0);
            break;
    }
}

}  // namespace jensen
