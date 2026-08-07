// Jensen Hand — PCA9685 servo bank.
//
// Two independent ways to remove torque, because they fail differently:
//
//   /OE   cuts the PWM outputs in microseconds but leaves the servos powered,
//         so they go limp instantly and can be back-driven.
//   RAIL  drops the servo supply entirely. Slower (the bulk capacitance takes
//         time to bleed down) but it also covers a PCA9685 that has stopped
//         responding on I2C and is still emitting its last frame.
//
// Safe torque off asserts both.
#pragma once

#include <Adafruit_PWMServoDriver.h>

#include <cstdint>

namespace jensen {

class ServoBus {
public:
    /// Brings up I2C and the PCA9685. Returns false if the device does not
    /// acknowledge, which the controller treats as a severe fault.
    bool begin();

    /// Re-checks that the PCA9685 is still on the bus. Cheap enough to call
    /// at a few hertz; a servo harness that works loose is a real failure
    /// mode in a worn device.
    bool poll();
    bool healthy() const { return healthy_; }

    void setRailEnabled(bool enabled);
    void setOutputEnabled(bool enabled);
    bool railEnabled() const { return railEnabled_; }
    bool outputEnabled() const { return outputEnabled_; }

    /// Writes one channel. `microseconds == 0` emits no pulse at all, leaving
    /// that servo unpowered and back-drivable.
    void write(uint8_t channel, uint16_t microseconds);

    /// Stops every channel. Used on fault paths and at shutdown.
    void releaseAll();

private:
    Adafruit_PWMServoDriver driver_{};
    bool healthy_ = false;
    bool railEnabled_ = false;
    bool outputEnabled_ = false;
    /// Last value written per channel, so unchanged channels are not re-sent
    /// over I2C every control tick.
    uint16_t lastMicros_[16] = {0};
};

}  // namespace jensen
