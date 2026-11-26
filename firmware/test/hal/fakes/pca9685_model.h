// A register-level model of the parts of the PCA9685 this firmware depends on,
// written from the NXP data sheet. It is a model, not the chip: it shows that
// the driver does the right thing for the documented behaviour, nothing more.
//
//   * power-on: MODE1 = 0x11 (asleep, auto-increment off), PRE_SCALE = 0x1E,
//     every channel full off
//   * without auto-increment, every byte of a burst lands in one register
//   * PRE_SCALE can only be written while SLEEP is set
//   * writing 1 to MODE1.RESTART does not leave the bit set
//   * no output while the oscillator is off
#pragma once

#include <cstddef>
#include <cstdint>

namespace fake {

class Pca9685Model {
public:
    static constexpr int kRailPin = 26;          // board::kPinServoRailEnable
    static constexpr int kOutputEnablePin = 27;  // board::kPinServoOutputEnable

    bool present = true;           ///< acknowledges on the bus
    bool ignoresPrescale = false;  ///< a part that will not take its setup
    int corruptReads = 0;          ///< this many reads return 0xFF

    /// Register writes made while the servo rail or outputs were enabled.
    int writesWithTorqueAvailable = 0;

    Pca9685Model() { powerCycle(); }

    void powerCycle() {
        for (uint8_t& r : reg_) r = 0;
        reg_[0x00] = 0x11;
        reg_[0x01] = 0x04;
        reg_[0xFE] = 0x1E;
        for (int channel = 0; channel < 16; ++channel) reg_[0x09 + 4 * channel] = 0x10;
    }

    void setPin(int pin, int level) {
        if (pin == kRailPin) rail_ = level != 0;
        if (pin == kOutputEnablePin) outputsEnabled_ = level == 0;  // active low
    }
    bool rail() const { return rail_; }
    bool outputsEnabled() const { return outputsEnabled_; }

    void write(const uint8_t* bytes, size_t length) {
        if (length == 0) return;
        pointer_ = bytes[0];
        for (size_t i = 1; i < length; ++i) {
            if (rail_ || outputsEnabled_) ++writesWithTorqueAvailable;
            store(pointer_, bytes[i]);
            if (autoIncrement()) ++pointer_;
        }
    }

    uint8_t readNext() {
        if (corruptReads > 0) {
            --corruptReads;
            return 0xFF;
        }
        const uint8_t value = reg_[pointer_];
        if (autoIncrement()) ++pointer_;
        return value;
    }

    uint8_t mode1() const { return reg_[0x00]; }
    uint8_t prescale() const { return reg_[0xFE]; }
    bool asleep() const { return (reg_[0x00] & 0x10) != 0; }
    bool autoIncrement() const { return (reg_[0x00] & 0x20) != 0; }

    bool channelFullOff(int channel) const { return (reg_[0x09 + 4 * channel] & 0x10) != 0; }
    uint16_t channelOffCount(int channel) const {
        return static_cast<uint16_t>(reg_[0x08 + 4 * channel] |
                                     ((reg_[0x09 + 4 * channel] & 0x0F) << 8));
    }
    bool channelPulsing(int channel) const {
        return !asleep() && !channelFullOff(channel) && channelOffCount(channel) > 0;
    }

private:
    void store(uint8_t address, uint8_t value) {
        if (address == 0x00) {
            reg_[0x00] = value & 0x7F;
        } else if (address == 0xFE) {
            if (asleep() && !ignoresPrescale) reg_[0xFE] = value;
        } else {
            reg_[address] = value;
        }
    }

    uint8_t reg_[256] = {};
    uint8_t pointer_ = 0;
    bool rail_ = false;
    bool outputsEnabled_ = false;
};

inline Pca9685Model& chip() {
    static Pca9685Model instance;
    return instance;
}

}  // namespace fake
