// Jensen Hand — guided electrode calibration.
//
// HACKberry calibrates by watching the sensor for four seconds and taking the
// min and max it happens to see. That conflates three different quantities —
// the electrode's DC offset, its noise floor, and the user's maximum
// contraction — into two numbers, and it silently produces a working-looking
// calibration from an electrode that has fallen off.
//
// This runner walks the user through the measurements separately and refuses
// to accept a result where the contraction is not clearly distinguishable from
// rest. It is a timed sequence with no blocking waits, so the control loop
// keeps running (and the hand stays parked and responsive) throughout.
#pragma once

#include <cstdint>

#include "jensen/emg.h"
#include "jensen/hand_state.h"

namespace jensen {

struct CalibrationTiming {
    float settleSeconds = 1.0f;  ///< pause before each capture, with a cue
    float restSeconds = 3.0f;    ///< baseline and noise floor
    float effortSeconds = 3.0f;  ///< maximum voluntary contraction
};

/// Guided two-electrode calibration sequence.
class EmgCalibrationRunner {
public:
    enum class Step : uint8_t {
        Idle = 0,
        SettleRest,     ///< "relax your arm"
        CaptureRest,    ///< recording baseline + noise for both channels
        SettleFlexor,   ///< "get ready to close"
        CaptureFlexor,  ///< recording flexor MVC
        SettleExtensor, ///< "get ready to open"
        CaptureExtensor,///< recording extensor MVC
        Finished,
        Failed,
    };

    void configure(const CalibrationTiming& timing);

    /// Starts the sequence. `dualChannel` selects whether the extensor phases
    /// run at all.
    void begin(EmgChannel& flexor, EmgChannel& extensor, bool dualChannel);
    void abort(EmgChannel& flexor, EmgChannel& extensor);

    /// Advance the sequence by one control tick. Returns a haptic cue on the
    /// ticks where a phase boundary is crossed, `HapticCue::None` otherwise.
    HapticCue update(EmgChannel& flexor, EmgChannel& extensor, float flexorCounts,
                     float extensorCounts, float dt);

    bool running() const;
    bool finished() const { return step_ == Step::Finished || step_ == Step::Failed; }
    bool succeeded() const { return step_ == Step::Finished; }

    Step step() const { return step_; }
    const char* stepName() const;
    /// Overall progress through the sequence, 0..1.
    float progress() const;

    const EmgCalibration& flexorResult() const { return flexorResult_; }
    const EmgCalibration& extensorResult() const { return extensorResult_; }

private:
    void enter(Step step);
    float stepDuration(Step step) const;

    CalibrationTiming timing_{};
    Step step_ = Step::Idle;
    float stepTimer_ = 0.0f;
    float elapsed_ = 0.0f;
    float total_ = 1.0f;
    bool dualChannel_ = true;

    EmgCalibration flexorResult_{};
    EmgCalibration extensorResult_{};
};

/// Bench calibration of one servo endpoint.
///
/// Servo endpoints are found by jogging a digit to its open and closed
/// mechanical limits with the tendon fitted and recording the pulse width at
/// each. This helper holds the in-progress state for that procedure so the
/// serial CLI and the display can drive the same code path.
class DigitCalibrationRunner {
public:
    void begin(Digit digit, const DigitCalibration& current);
    void abort() { active_ = false; }

    bool active() const { return active_; }
    Digit digit() const { return digit_; }

    /// Move the working pulse width. Clamped to the existing guard band, so a
    /// jog can never exceed limits the installer has already established.
    uint16_t jog(int16_t deltaUs);
    uint16_t pulseUs() const { return pulseUs_; }
    void setPulseUs(uint16_t us);

    void markOpen() { open_ = pulseUs_; haveOpen_ = true; }
    void markClosed() { closed_ = pulseUs_; haveClosed_ = true; }
    bool haveOpen() const { return haveOpen_; }
    bool haveClosed() const { return haveClosed_; }

    /// Build the resulting calibration. Guard limits are widened to sit a
    /// small margin outside the captured endpoints.
    bool result(DigitCalibration& out) const;

private:
    bool active_ = false;
    Digit digit_ = Digit::Index;
    DigitCalibration base_{};
    uint16_t pulseUs_ = 1500;
    uint16_t open_ = 1500;
    uint16_t closed_ = 1500;
    bool haveOpen_ = false;
    bool haveClosed_ = false;
};

}  // namespace jensen
