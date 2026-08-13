// Compile-only stub — see Arduino.h in this directory.
//
// Mirrors the subset of the Adafruit PWM Servo Driver 3.x API that
// hal/servo_bus.cpp uses.
#pragma once

#include <Wire.h>

#include <cstdint>

class Adafruit_PWMServoDriver {
public:
    Adafruit_PWMServoDriver();
    explicit Adafruit_PWMServoDriver(uint8_t address);
    Adafruit_PWMServoDriver(uint8_t address, TwoWire& bus);

    bool begin(uint8_t prescale = 0);
    void reset();
    void sleep();
    void wakeup();

    void setOscillatorFrequency(uint32_t frequency);
    void setPWMFreq(float frequency);

    void setPWM(uint8_t channel, uint16_t on, uint16_t off);
    uint8_t writeMicroseconds(uint8_t channel, uint16_t microseconds);
};
