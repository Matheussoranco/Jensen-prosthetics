#include "jensen/current.h"

#include "jensen/util.h"

namespace jensen {

void MotorCurrentGuard::configure(const MotorCurrentConfig& config, bool present) {
    config_ = config;
    config_.tripAmps = clamp(config_.tripAmps, 0.05f, 20.0f);
    config_.clearAmps = clamp(config_.clearAmps, 0.0f, config_.tripAmps);
    config_.tripSeconds = clamp(config_.tripSeconds, 0.01f, 10.0f);
    config_.clearSeconds = clamp(config_.clearSeconds, 0.0f, 60.0f);
    present_ = present;
    if (!present_) reset();
}

void MotorCurrentGuard::reset() {
    stalled_ = false;
    amps_ = 0.0f;
    highTimer_ = 0.0f;
    clearTimer_ = 0.0f;
    stallCount_ = 0;
}

void MotorCurrentGuard::update(float amps, bool energised, float dt) {
    amps_ = amps > 0.0f ? amps : 0.0f;
    if (!present_ || !energised || dt <= 0.0f) {
        highTimer_ = 0.0f;
        clearTimer_ = 0.0f;
        return;
    }

    if (stalled_) {
        if (amps_ <= config_.clearAmps) {
            clearTimer_ += dt;
            if (clearTimer_ >= config_.clearSeconds) {
                stalled_ = false;
                highTimer_ = 0.0f;
                clearTimer_ = 0.0f;
            }
        } else {
            clearTimer_ = 0.0f;
        }
        return;
    }

    // A calibrated current channel is direct evidence of overload. Position
    // here is commanded position (ordinary hobby servos expose no encoder),
    // so using command error would let a physically jammed servo escape once
    // its software trajectory reached the target.
    if (amps_ >= config_.tripAmps) {
        highTimer_ += dt;
        if (highTimer_ >= config_.tripSeconds) {
            stalled_ = true;
            clearTimer_ = 0.0f;
            ++stallCount_;
        }
    } else {
        highTimer_ = 0.0f;
    }
}

}  // namespace jensen
