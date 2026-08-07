#include "hal/button.h"

#include <Arduino.h>

#include "board_config.h"

namespace jensen {

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
