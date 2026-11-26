// Host fake: TwoWire talking to the chip model. See Arduino.h here.
#pragma once

#include <cstddef>
#include <cstdint>

#include "pca9685_model.h"

class TwoWire {
public:
    bool begin(int, int, uint32_t = 0) { return true; }

    void beginTransmission(uint8_t) { length_ = 0; }
    size_t write(uint8_t value) {
        if (length_ >= sizeof(buffer_)) return 0;
        buffer_[length_++] = value;
        return 1;
    }
    uint8_t endTransmission() {
        if (!fake::chip().present) return 2;  // address NACK
        fake::chip().write(buffer_, length_);
        return 0;
    }

    uint8_t requestFrom(uint8_t, uint8_t quantity) {
        available_ = fake::chip().present ? quantity : 0;
        return available_;
    }
    int read() {
        if (available_ == 0) return -1;
        --available_;
        return fake::chip().readNext();
    }

private:
    uint8_t buffer_[32] = {};
    size_t length_ = 0;
    uint8_t available_ = 0;
};

inline TwoWire Wire;
