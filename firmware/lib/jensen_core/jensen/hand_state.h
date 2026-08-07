// Jensen Hand — inputs, outputs and telemetry.
//
// These three structs are the entire boundary between the control policy and
// the hardware. `HandInputs` is everything the ESP32 layer samples,
// `HandOutputs` is everything it drives, and `HandTelemetry` is the read-only
// view the display and the serial CLI render. Nothing in `jensen_core` touches
// a peripheral, which is what allows the whole policy to be exercised on a
// host with synthetic inputs.
#pragma once

#include <cstdint>

#include "jensen/digit.h"
#include "jensen/grip.h"
#include "jensen/modes.h"

namespace jensen {

/// One sample set from the analogue front end and the user controls.
struct HandInputs {
    /// Raw ADC counts. The ESP32 layer oversamples; no filtering is applied
    /// before this point, because every filter in the core is time-aware and
    /// needs to see the sample rate it is actually running at.
    float emgFlexorCounts = 0.0f;
    float emgExtensorCounts = 0.0f;
    float fsrCounts[kGraspDigitCount] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    /// Optional LM393 comparator outputs, one per instrumented fingertip.
    /// Used only to corroborate contact — the analogue path is authoritative.
    bool contactDigital[kGraspDigitCount] = {false, false, false, false, false, false};

    float batteryVolts = 0.0f;

    ButtonEvent button = ButtonEvent::None;

    /// False when the PCA9685 stopped acknowledging on I2C.
    bool servoBusOk = true;
};

/// Haptic cues. The motor is a single eccentric-mass module, so the vocabulary
/// is deliberately small — a user cannot reliably distinguish more than a few
/// patterns through a socket.
enum class HapticCue : uint8_t {
    None = 0,
    Tick,        ///< grip changed
    DoubleTick,  ///< hold engaged / released
    Contact,     ///< fingertips just made contact with an object
    Slip,        ///< object is escaping; the hand is tightening
    Warning,     ///< long buzz: battery low or a fault appeared
    Confirm,     ///< calibration step accepted
};

/// Status LED behaviour, resolved by the HAL into a blink pattern.
enum class LedPattern : uint8_t {
    Off = 0,
    Solid,        ///< active
    Breathe,      ///< idle
    SlowBlink,    ///< holding
    FastBlink,    ///< calibrating
    DoubleBlink,  ///< warning-level fault
    SosBlink,     ///< severe fault / safe torque off
};

/// Everything the ESP32 layer drives after one control iteration.
struct HandOutputs {
    /// Pulse width per digit in microseconds. Zero means "emit no pulse" —
    /// the channel is left unpowered and the digit is back-drivable.
    uint16_t pulseUs[kDigitCount] = {0, 0, 0, 0, 0, 0, 0};

    /// Master servo rail enable (buck converter / MOSFET gate) and the
    /// PCA9685 output-enable line. Both are dropped together for safe torque
    /// off; OE reacts in microseconds, the rail takes longer to collapse.
    bool servoRailEnabled = false;
    bool servoOutputEnabled = false;

    LedPattern led = LedPattern::Off;
    HapticCue haptic = HapticCue::None;
};

struct DigitTelemetry {
    float closure = 0.0f;
    float target = 0.0f;
    uint16_t pulseUs = 0;
    bool locked = false;
    bool released = false;
};

/// Read-only snapshot for the Nextion display and the serial CLI.
struct HandTelemetry {
    ControlMode mode = ControlMode::Boot;
    GripId grip = GripId::Power;
    const char* gripName = "";

    /// Commanded aperture: 0 fully open, 1 fully closed.
    float aperture = 0.0f;
    /// Closure velocity from the drive law, units per second.
    float velocity = 0.0f;

    float emgFlexor = 0.0f;    ///< activation, 0..1
    float emgExtensor = 0.0f;
    float emgFlexorEnvelope = 0.0f;   ///< raw envelope in ADC counts
    float emgExtensorEnvelope = 0.0f;

    float force[kGraspDigitCount] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    /// LM393 comparator outputs, passed through for diagnostics only. The
    /// control path uses the analogue force signal; these exist so an
    /// installer can verify the comparator threshold pots against a load the
    /// analogue channel is already reporting.
    bool contactDigital[kGraspDigitCount] = {false, false, false, false, false, false};
    float peakForce = 0.0f;
    float forceTarget = 0.0f;
    float closureLimit = 1.0f;
    bool contact = false;
    uint32_t slipCount = 0;

    float batteryVolts = 0.0f;
    float batteryCellVolts = 0.0f;
    float stateOfCharge = 0.0f;

    uint16_t faults = 0;
    const char* faultName = nullptr;

    /// Non-null while a guided calibration is running.
    const char* calibrationStep = nullptr;
    float calibrationProgress = 0.0f;

    DigitTelemetry digits[kDigitCount];

    float loopSeconds = 0.0f;
    uint32_t loopCount = 0;
    uint32_t uptimeSeconds = 0;
};

}  // namespace jensen
