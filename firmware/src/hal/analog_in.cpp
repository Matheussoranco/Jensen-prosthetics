#include "hal/analog_in.h"

#include <Arduino.h>

#include "board_config.h"

namespace jensen {
namespace analog {

namespace {

/// Every analogue input on this board is on ADC1 and swings across most of
/// the 0–3.3 V range, so all of them want the widest attenuation.
constexpr int kInputPins[] = {
    board::kPinEmgFlexor,   board::kPinEmgExtensor, board::kPinBatterySense,
    board::kPinFsr[0],      board::kPinFsr[2],      board::kPinFsr[3],
    board::kPinEmgAux[0],   board::kPinEmgAux[1],
    board::kPinMotorCurrent[0], board::kPinMotorCurrent[1], board::kPinMotorCurrent[2],
    board::kPinMotorCurrent[3], board::kPinMotorCurrent[4], board::kPinMotorCurrent[5],
    board::kPinMotorCurrent[6],
};

}  // namespace

void begin() {
    analogReadResolution(12);
    for (int pin : kInputPins) {
        if (pin < 0) continue;
        pinMode(pin, INPUT);
        // 11 dB gives a nominal full scale near 3.1 V. The top of the range is
        // non-linear; the battery divider is sized to stay well below it.
        analogSetPinAttenuation(pin, ADC_11db);
    }
}

uint16_t readCounts(int pin, uint8_t samples) {
    if (pin < 0) return 0;
    if (samples == 0) samples = 1;
    uint32_t total = 0;
    for (uint8_t i = 0; i < samples; ++i) total += static_cast<uint32_t>(analogRead(pin));
    return static_cast<uint16_t>(total / samples);
}

uint32_t readMillivolts(int pin, uint8_t samples) {
    if (pin < 0) return 0;
    if (samples == 0) samples = 1;
    uint32_t total = 0;
    for (uint8_t i = 0; i < samples; ++i) total += analogReadMilliVolts(pin);
    return total / samples;
}

float readBatteryVolts(int pin, float dividerRatio, uint8_t samples) {
    if (pin < 0 || dividerRatio <= 0.0f) return 0.0f;
    return (static_cast<float>(readMillivolts(pin, samples)) / 1000.0f) * dividerRatio;
}

float readCurrentAmps(int pin, float zeroVolts, float voltsPerAmp, uint8_t samples) {
    if (pin < 0 || voltsPerAmp <= 0.0f) return 0.0f;
    const float volts = static_cast<float>(readMillivolts(pin, samples)) / 1000.0f;
    const float amps = (volts - zeroVolts) / voltsPerAmp;
    return amps > 0.0f ? amps : 0.0f;
}

}  // namespace analog
}  // namespace jensen
