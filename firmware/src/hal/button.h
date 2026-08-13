// Jensen Hand — mode button.
//
// Thin wrapper: it reads the pin and hands the raw state to
// `jensen::ButtonGestures`, which holds the actual recognition logic and is
// covered by the host test suite. Nothing here is worth testing; everything
// that is, lives in the core.
#pragma once

#include <cstdint>

#include "jensen/button_gestures.h"
#include "jensen/modes.h"

namespace jensen {

class Button {
public:
    /// `pin` is driven with an internal pull-up; the switch pulls it to ground.
    void begin(int pin);

    /// Call every control iteration. Returns at most one gesture per call.
    ButtonEvent poll(uint32_t nowMillis);

    bool pressed() const { return gestures_.pressed(); }

private:
    int pin_ = -1;
    ButtonGestures gestures_{};
};

}  // namespace jensen
