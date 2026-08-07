// Compile-only stub — see Arduino.h in this directory.
#pragma once

#include <cstdint>

class TwoWire {
public:
    bool begin(int sda, int scl, uint32_t frequency = 0);
    void beginTransmission(uint8_t address);
    uint8_t endTransmission();
    uint8_t endTransmission(bool sendStop);
    void setClock(uint32_t frequency);
};

extern TwoWire Wire;
