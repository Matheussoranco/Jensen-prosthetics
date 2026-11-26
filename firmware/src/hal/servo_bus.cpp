#include "hal/servo_bus.h"

#include <Arduino.h>
#include <Wire.h>

#include "board_config.h"
#include "jensen/pca9685.h"

namespace jensen {

namespace {
/// PCA9685 "full off" is bit 12 of the OFF register, which suppresses the
/// pulse entirely rather than emitting a zero-width one.
constexpr uint16_t kFullOff = 4096;
/// Never a pulse width, so the next write() of any value reaches the chip.
constexpr uint16_t kUnknown = 0xFFFF;

constexpr uint8_t kPrescale =
    pca9685Prescale(board::kPca9685OscillatorHz, board::kServoFrequencyHz);
}  // namespace

bool ServoBus::begin() {
    pinMode(board::kPinServoRailEnable, OUTPUT);
    pinMode(board::kPinServoOutputEnable, OUTPUT);
    // Assert safe torque off *before* the bus comes up, so a PCA9685 holding
    // stale registers from a watchdog reset cannot twitch the hand.
    setRailEnabled(false);
    setOutputEnabled(false);

    Wire.begin(board::kPinI2cSda, board::kPinI2cScl, board::kI2cFrequencyHz);

    healthy_ = false;
    configured_ = false;
    resetPending_ = false;

    driver_ = Adafruit_PWMServoDriver(board::kPca9685Address, Wire);
    if (!driver_.begin()) return false;

    healthy_ = configure();
    return healthy_;
}

bool ServoBus::poll() {
    bool holds = false;
    if (!readConfiguration(holds)) {
        healthy_ = false;  // off the bus: nothing is known about its state
        return false;
    }

    // A reset is persistent, a corrupted read is not: look twice before
    // leaving the wearer without a hand.
    if (configured_ && !holds && !readConfiguration(holds)) {
        healthy_ = false;
        return false;
    }

    if (configured_ && holds) {
        if (!healthy_) {
            // Writes made while the bus was away may never have arrived.
            for (uint16_t& value : lastMicros_) value = kUnknown;
        }
        healthy_ = true;
        return true;
    }

    if (configured_) {
        // The chip has forgotten what it was told, so it was reset.
        configured_ = false;
        resetPending_ = true;
    }

    // Never write to an unconfigured chip with torque available.
    setOutputEnabled(false);
    setRailEnabled(false);
    healthy_ = configure();
    return healthy_;
}

bool ServoBus::takeReset() {
    const bool pending = resetPending_;
    resetPending_ = false;
    return pending;
}

bool ServoBus::readRegister(uint8_t reg, uint8_t& value) {
    Wire.beginTransmission(board::kPca9685Address);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom(board::kPca9685Address, static_cast<uint8_t>(1)) != 1) return false;
    value = static_cast<uint8_t>(Wire.read());
    return true;
}

bool ServoBus::readConfiguration(bool& holds) {
    uint8_t mode1 = 0;
    uint8_t prescale = 0;
    if (!readRegister(kPca9685RegMode1, mode1) || !readRegister(kPca9685RegPrescale, prescale)) {
        return false;
    }
    holds = pca9685Configured(mode1, prescale, kPrescale);
    return true;
}

bool ServoBus::configure() {
    // reset() wakes the chip. It must come first: setPWMFreq() restores the
    // MODE1 it found, so on a freshly powered, sleeping PCA9685 it would leave
    // the chip asleep.
    driver_.reset();
    driver_.setOscillatorFrequency(board::kPca9685OscillatorHz);
    driver_.setPWMFreq(board::kServoFrequencyHz);

    for (uint8_t channel = 0; channel < 16; ++channel) {
        driver_.setPWM(channel, 0, kFullOff);
        lastMicros_[channel] = 0;
    }

    bool holds = false;
    configured_ = readConfiguration(holds) && holds;
    return configured_;
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
