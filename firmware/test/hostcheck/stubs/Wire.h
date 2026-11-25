// Compile-only stub — see Arduino.h in this directory.
#pragma once

#include <cstddef>
#include <cstdint>

class TwoWire {
public:
    bool begin(int sda, int scl, uint32_t frequency = 0);
    void beginTransmission(uint8_t address);
    uint8_t endTransmission();
    uint8_t endTransmission(bool sendStop);
    size_t write(uint8_t value);
    uint8_t requestFrom(uint8_t address, uint8_t quantity);
    int read();
    void setClock(uint32_t frequency);
};

extern TwoWire Wire;
