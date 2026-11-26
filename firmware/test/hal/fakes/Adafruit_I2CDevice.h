// Host fake of the Adafruit BusIO device: the only I2C interface the PWM driver
// library uses. Transfers go to the chip model.
#pragma once

#include <cstddef>
#include <cstdint>

#include "Wire.h"
#include "pca9685_model.h"

class Adafruit_I2CDevice {
public:
    Adafruit_I2CDevice(uint8_t, TwoWire*) {}

    bool begin(bool = true) { return fake::chip().present; }

    bool write(const uint8_t* buffer, size_t length) {
        if (!fake::chip().present) return false;
        fake::chip().write(buffer, length);
        return true;
    }

    bool write_then_read(const uint8_t* out, size_t outLength, uint8_t* in, size_t inLength) {
        if (!fake::chip().present) return false;
        fake::chip().write(out, outLength);
        for (size_t i = 0; i < inLength; ++i) in[i] = fake::chip().readNext();
        return true;
    }
};
