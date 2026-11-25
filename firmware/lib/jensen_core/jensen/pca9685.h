// Jensen Hand — recognising a PCA9685 that has forgotten its configuration.
//
// Pure functions, here rather than beside the I2C code so the host tests can
// check them.  Values are from the NXP PCA9685 data sheet.
//
// After power-on the chip is asleep (oscillator off, no output), has
// auto-increment disabled and runs a 200 Hz prescale.  It still acknowledges
// its address, so "it answers" says nothing about whether it is driving servos.
#pragma once

#include <cstdint>

namespace jensen {

constexpr uint8_t kPca9685RegMode1 = 0x00;
constexpr uint8_t kPca9685RegPrescale = 0xFE;

constexpr uint8_t kPca9685Mode1Sleep = 0x10;          ///< set at power-on
constexpr uint8_t kPca9685Mode1AutoIncrement = 0x20;  ///< clear at power-on

/// PRE_SCALE for a frame rate.  Rounds as the Adafruit driver library does, so
/// the value read back from the chip matches exactly.
constexpr uint8_t pca9685Prescale(uint32_t oscillatorHz, float frameHz) {
    return static_cast<uint8_t>(
        static_cast<float>(oscillatorHz) / (frameHz * 4096.0f) + 0.5f - 1.0f);
}

/// True when the registers still hold the running configuration this firmware
/// wrote: awake, auto-incrementing and at the expected frame rate.
constexpr bool pca9685Configured(uint8_t mode1, uint8_t prescale, uint8_t expectedPrescale) {
    return (mode1 & kPca9685Mode1Sleep) == 0 && (mode1 & kPca9685Mode1AutoIncrement) != 0 &&
           prescale == expectedPrescale;
}

}  // namespace jensen
