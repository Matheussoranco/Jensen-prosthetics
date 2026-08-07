// Jensen Hand — board pin map and hardware constants.
//
// Target: ESP32 DevKit (30-pin) on the forearm expansion board, driving a
// PCA9685 servo bank. Every analogue input is on ADC1: ADC2 shares hardware
// with the Wi-Fi radio and stops returning readings the moment the radio is
// enabled, which would silently kill myoelectric control the first time
// someone adds a BLE feature.
//
//  GPIO  Direction  Function
//  ----  ---------  -----------------------------------------------------
//    21  I2C SDA    PCA9685 (0x40)
//    22  I2C SCL    PCA9685
//    27  out        PCA9685 /OE — low enables outputs (safe torque off)
//    26  out        Servo rail enable (buck converter / high-side MOSFET)
//    16  UART2 RX   Nextion 2.4" HMI
//    17  UART2 TX   Nextion 2.4" HMI
//    18  UART1 RX   Voice Recognition V3 module (optional)
//     5  UART1 TX   Voice Recognition V3 module (optional)
//     4  in         Mode button, active low, internal pull-up
//    13  out        Status LED (green, 5 mm)
//    14  out        Vibration motor, LEDC PWM
//     2  out        On-board LED, control-loop heartbeat
//    25  in         LM393 contact comparator — thumb
//    19  in         LM393 contact comparator — index
//    23  in         LM393 contact comparator — middle
//    36  ADC1_CH0   EMG flexor  (VP)
//    39  ADC1_CH3   EMG extensor (VN)
//    35  ADC1_CH7   FSR400 — thumb
//    32  ADC1_CH4   FSR400 — index
//    33  ADC1_CH5   FSR400 — middle
//    34  ADC1_CH6   Battery sense (divider)
//
// GPIO 0, 2, 12 and 15 are strapping pins. 2 is used only as an output after
// boot; 12 and 15 are left unconnected because pulling them at reset changes
// the flash voltage and the boot mode.
#pragma once

#include <cstdint>

#include "jensen/digit.h"

namespace jensen {
namespace board {

// --- I2C / servo bus ---------------------------------------------------------
constexpr int kPinI2cSda = 21;
constexpr int kPinI2cScl = 22;
constexpr uint32_t kI2cFrequencyHz = 400000;
constexpr uint8_t kPca9685Address = 0x40;

/// PCA9685 output-enable is active low.
constexpr int kPinServoOutputEnable = 27;
/// Cuts power to the servo rail entirely. Slower than /OE, so both are used.
constexpr int kPinServoRailEnable = 26;

/// Servo refresh rate. 50 Hz is the analogue-servo standard; MG996R will not
/// track a faster frame reliably.
constexpr float kServoFrequencyHz = 50.0f;

/// The PCA9685's internal oscillator is nominally 25 MHz but parts run high.
/// Adafruit's library exposes this so the prescaler maths can be corrected;
/// measure the frame period on a scope and adjust if pulses read long.
constexpr uint32_t kPca9685OscillatorHz = 27000000;

/// PCA9685 channel assignment, indexed by `Digit`.
constexpr uint8_t kServoChannel[kDigitCount] = {
    0,  // ThumbFlex — MG996R
    1,  // ThumbRot  — MG996R (CMC opposition)
    2,  // Index     — MG996R
    3,  // Middle    — MG996R
    4,  // Ring      — MG996R
    5,  // Pinky     — MG996R
    6,  // Wrist     — SG90 or MG996R, optional
};

// --- User interface ----------------------------------------------------------
constexpr int kPinButton = 4;
constexpr int kPinStatusLed = 13;
constexpr int kPinHeartbeatLed = 2;
constexpr int kPinHaptic = 14;

/// LEDC channels. Kept clear of 0–3, which share timer 0 with the Arduino
/// core's `tone()`.
constexpr uint8_t kLedcChannelHaptic = 4;
constexpr uint8_t kLedcChannelStatusLed = 5;
constexpr uint32_t kLedcFrequencyHz = 5000;
constexpr uint8_t kLedcResolutionBits = 10;

// --- Displays and serial -----------------------------------------------------
constexpr int kPinNextionRx = 16;  // ESP32 receives on this pin
constexpr int kPinNextionTx = 17;
constexpr uint32_t kNextionBaud = 9600;

constexpr int kPinVoiceRx = 18;
constexpr int kPinVoiceTx = 5;
constexpr uint32_t kVoiceBaud = 9600;

constexpr uint32_t kConsoleBaud = 115200;

// --- Analogue inputs ---------------------------------------------------------
constexpr int kPinEmgFlexor = 36;
constexpr int kPinEmgExtensor = 39;
constexpr int kPinBatterySense = 34;

/// ADC pin per grasping digit; -1 where no fingertip sensor is fitted.
constexpr int kPinFsr[kGraspDigitCount] = {
    35,  // ThumbFlex
    -1,  // ThumbRot — no fingertip
    32,  // Index
    33,  // Middle
    -1,  // Ring
    -1,  // Pinky
};

/// LM393 comparator contact outputs; -1 where not fitted.
constexpr int kPinContact[kGraspDigitCount] = {25, -1, 19, 23, -1, -1};

/// Battery divider. Measure the fitted resistors and correct this: an 8.4 V
/// pack read through a wrong ratio is the difference between "charged" and
/// "park the hand".
///
/// Reference divider: R1 = 100 kΩ (pack to node), R2 = 22 kΩ (node to ground)
/// gives 8.4 V -> 1.51 V, comfortably inside the 11 dB ADC range.
constexpr float kBatteryDividerRatio = (100.0f + 22.0f) / 22.0f;

/// Samples averaged per analogue read. The ESP32 SAR ADC is noisy and the
/// servo rail is switching a few centimetres away.
constexpr uint8_t kAdcOversample = 8;
constexpr float kAdcFullScaleCounts = 4095.0f;

// --- Control loop ------------------------------------------------------------
constexpr uint32_t kControlPeriodMicros = 10000;  // 100 Hz
constexpr uint32_t kTelemetryPeriodMillis = 100;  // 10 Hz
constexpr uint32_t kDisplayPeriodMillis = 200;    // 5 Hz

/// Hardware watchdog timeout. Comfortably longer than one control period but
/// short enough that a wedged hand recovers before the user notices more than
/// a stutter.
constexpr uint32_t kWatchdogSeconds = 2;

// --- Button gesture timing ---------------------------------------------------
constexpr uint32_t kButtonDebounceMillis = 25;
constexpr uint32_t kButtonDoubleClickMillis = 350;
constexpr uint32_t kButtonLongPressMillis = 1200;
constexpr uint32_t kButtonVeryLongPressMillis = 4000;

}  // namespace board
}  // namespace jensen
