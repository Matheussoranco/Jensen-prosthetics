// Host fake for the HAL tests: just enough Arduino to run servo_bus.cpp and the
// real Adafruit PWM driver against the chip model in pca9685_model.h.
#pragma once

#include <cstddef>
#include <cstdint>

#include "pca9685_model.h"

#define OUTPUT 0x03
#define HIGH 0x1
#define LOW 0x0

template <typename T>
constexpr T min(T a, T b) {
    return b < a ? b : a;
}

inline void pinMode(int, int) {}
inline void digitalWrite(int pin, int level) { fake::chip().setPin(pin, level); }
inline void delay(unsigned long) {}
