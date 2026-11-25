#include "hal/button.h"

#include <Arduino.h>

#include "board_config.h"

namespace jensen {

// The control core accepts the very long press as consent after a servo-driver
// reset only once the hand has been limp for the core's gesture length. A
// board that waited longer would let a press begun before the reset count.
static_assert(board::kButtonVeryLongPressMillis <= ButtonTiming{}.veryLongPressMillis,
              "lengthen the consent guard in hand_controller.cpp with this");

void Button::begin(int pin) {
    pin_ = pin;
    pinMode(pin_, INPUT_PULLUP);

    ButtonTiming timing;
    timing.debounceMillis = board::kButtonDebounceMillis;
    timing.doubleClickMillis = board::kButtonDoubleClickMillis;
    timing.longPressMillis = board::kButtonLongPressMillis;
    timing.veryLongPressMillis = board::kButtonVeryLongPressMillis;
    gestures_.configure(timing);
    gestures_.reset(millis());
}

ButtonEvent Button::poll(uint32_t nowMillis) {
    if (pin_ < 0) return ButtonEvent::None;
    // Active low: the switch pulls the pin to ground.
    return gestures_.update(digitalRead(pin_) == LOW, nowMillis);
}

}  // namespace jensen
