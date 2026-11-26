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
//
// A PCA9685 whose logic supply drops out comes back asleep at its power-on
// defaults and still acknowledges its address. poll() therefore reads the
// registers back; on finding them lost it cuts torque, configures the chip
// again with every output off, and reports a reset once. Whether the hand may
// then be energised is the control core's decision, not this class's.
#pragma once

#include <Adafruit_PWMServoDriver.h>

#include <cstdint>

namespace jensen {

class ServoBus {
public:
    /// Brings up I2C and the PCA9685. Returns false if it does not acknowledge
    /// or will not hold its configuration; the controller treats that as a
    /// severe fault.
    bool begin();

    /// Probes the chip, checks its configuration and restores it if lost.
    /// Cheap enough to call at a few hertz when nothing is wrong; a servo
    /// harness that works loose is a real failure mode in a worn device.
    /// Restoring takes about 15 ms of library delays. Returns healthy().
    bool poll();

    /// On the bus and holding the configuration written by this class.
    bool healthy() const { return healthy_; }

    /// A reset has been found and not yet handed to the control core.
    bool resetPending() const { return resetPending_; }
    /// True once per detected reset (however many polls it took to recover).
    bool takeReset();

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
    bool readRegister(uint8_t reg, uint8_t& value);
    /// False when the chip cannot be read; `holds` is meaningful only if true.
    bool readConfiguration(bool& holds);
    /// Wakes the chip, sets the frame rate and turns every output off.
    bool configure();

    Adafruit_PWMServoDriver driver_{};
    bool healthy_ = false;
    /// The chip has held a verified configuration since the last (re)write.
    bool configured_ = false;
    bool resetPending_ = false;
    bool railEnabled_ = false;
    bool outputEnabled_ = false;
    /// Last value written per channel, so unchanged channels are not re-sent
    /// over I2C every control tick.
    uint16_t lastMicros_[16] = {0};
};

}  // namespace jensen
