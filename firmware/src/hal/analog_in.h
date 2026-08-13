// Jensen Hand — analogue front end.
//
// Thin wrapper over the ESP32 ADC that does two things the control core
// cannot: oversampling (the SAR ADC is noisy, and there is a switching servo
// rail a few centimetres away) and the divider maths for pack voltage.
//
// Deliberately *not* filtering here. Every filter in `jensen_core` is
// expressed as a time constant and needs to see the sample rate it actually
// runs at; a hidden filter in the HAL would change the feel of the hand
// depending on how the loop happened to be scheduled.
#pragma once

#include <cstdint>

namespace jensen {
namespace analog {

/// Configures ADC width and per-pin attenuation. Call once from setup().
void begin();

/// Averaged raw counts, 0..4095. Returns 0 for a pin of -1 (not fitted).
uint16_t readCounts(int pin, uint8_t samples);

/// Calibrated millivolts at the pin, using the ESP32's factory eFuse curve
/// where present. More accurate than scaling raw counts, which is why the
/// battery path uses it.
uint32_t readMillivolts(int pin, uint8_t samples);

/// Pack voltage, corrected for the sense divider.
float readBatteryVolts(int pin, float dividerRatio, uint8_t samples);

/// Current transducer result in amperes. `zeroVolts` is its zero-current
/// output and `voltsPerAmp` is the fitted sensor's sensitivity.
float readCurrentAmps(int pin, float zeroVolts, float voltsPerAmp, uint8_t samples);

}  // namespace analog
}  // namespace jensen
