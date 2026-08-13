#include "hal/servo_bus.h"

#include <Arduino.h>
#include <Wire.h>

#include "board_config.h"

namespace jensen {

namespace {
/// PCA9685 "full off" is bit 12 of the OFF register, which suppresses the
/// pulse entirely rather than emitting a zero-width one.
constexpr uint16_t kFullOff = 4096;
}  // namespace

bool ServoBus::begin() {
    pinMode(board::kPinServoRailEnable, OUTPUT);
    pinMode(board::kPinServoOutputEnable, OUTPUT);
    // Assert safe torque off *before* the bus comes up, so a PCA9685 holding
    // stale registers from a watchdog reset cannot twitch the hand.
    setRailEnabled(false);
    setOutputEnabled(false);

    Wire.begin(board::kPinI2cSda, board::kPinI2cScl, board::kI2cFrequencyHz);

    driver_ = Adafruit_PWMServoDriver(board::kPca9685Address, Wire);
    if (!driver_.begin()) {
        healthy_ = false;
        return false;
    }
    driver_.setOscillatorFrequency(board::kPca9685OscillatorHz);
    driver_.setPWMFreq(board::kServoFrequencyHz);

    for (uint8_t channel = 0; channel < 16; ++channel) {
        driver_.setPWM(channel, 0, kFullOff);
        lastMicros_[channel] = 0;
    }

    healthy_ = poll();
    return healthy_;
}

bool ServoBus::poll() {
    Wire.beginTransmission(board::kPca9685Address);
    healthy_ = (Wire.endTransmission() == 0);
    return healthy_;
}

void ServoBus::setRailEnabled(bool enabled) {
    railEnabled_ = enabled;
    digitalWrite(board::kPinServoRailEnable, enabled ? HIGH : LOW);
}

void ServoBus::setOutputEnabled(bool enabled) {
    outputEnabled_ = enabled;
    // /OE is active low.
    digitalWrite(board::kPinServoOutputEnable, enabled ? LOW : HIGH);
}

void ServoBus::write(uint8_t channel, uint16_t microseconds) {
    if (channel >= 16 || !healthy_) return;
    if (microseconds == lastMicros_[channel]) return;
    lastMicros_[channel] = microseconds;

    if (microseconds == 0) {
        driver_.setPWM(channel, 0, kFullOff);
    } else {
        driver_.writeMicroseconds(channel, microseconds);
    }
}

void ServoBus::releaseAll() {
    for (uint8_t channel = 0; channel < 16; ++channel) write(channel, 0);
    setOutputEnabled(false);
    setRailEnabled(false);
}

}  // namespace jensen
