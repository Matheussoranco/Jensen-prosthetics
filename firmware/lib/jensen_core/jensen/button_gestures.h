// Jensen Hand — button gesture recognition.
//
// One momentary switch carries four commands, separated by duration and count:
//
//   click            next grasp pattern
//   double click     hold / release
//   long  (1.2 s)    start calibration
//   very long (4 s)  park the hand / wake it up
//
// This lives in the control core rather than beside the pin read because the
// very-long gesture is the wearer's only always-available way to make the hand
// go limp. It has to be exercised by tests, and a recogniser that calls
// `digitalRead()` cannot be.
//
// Long presses fire while the button is still down, which is what makes them
// discoverable — the user feels the haptic confirmation and lets go. A single
// click is necessarily delayed by the double-click window; that is the cost of
// putting both on one switch.
#pragma once

#include <cstdint>

#include "jensen/modes.h"

namespace jensen {

struct ButtonTiming {
    uint32_t debounceMillis = 25;
    uint32_t doubleClickMillis = 350;
    uint32_t longPressMillis = 1200;
    uint32_t veryLongPressMillis = 4000;
};

class ButtonGestures {
public:
    void configure(const ButtonTiming& timing);
    const ButtonTiming& timing() const { return timing_; }

    void reset(uint32_t nowMillis);

    /// Feed the raw switch state (true = pressed) once per control iteration.
    /// Returns at most one gesture per call.
    ButtonEvent update(bool pressedRaw, uint32_t nowMillis);

    /// Debounced state.
    bool pressed() const { return debounced_; }

private:
    ButtonTiming timing_{};

    bool raw_ = false;
    bool debounced_ = false;
    uint32_t lastEdgeMillis_ = 0;

    uint32_t pressStartMillis_ = 0;
    /// Suppresses the click that would otherwise follow a hold's release.
    bool holdConsumed_ = false;
    /// Tracked separately from `holdConsumed_`: holding past the long-press
    /// threshold must still be able to reach the very-long one.
    bool longFired_ = false;
    bool veryLongFired_ = false;

    bool clickPending_ = false;
    uint32_t clickPendingSinceMillis_ = 0;
};

}  // namespace jensen
