#include "jensen/button_gestures.h"

namespace jensen {

void ButtonGestures::configure(const ButtonTiming& timing) {
    timing_ = timing;
}

void ButtonGestures::reset(uint32_t nowMillis) {
    raw_ = false;
    debounced_ = false;
    lastEdgeMillis_ = nowMillis;
    pressStartMillis_ = nowMillis;
    holdConsumed_ = false;
    longFired_ = false;
    veryLongFired_ = false;
    clickPending_ = false;
    clickPendingSinceMillis_ = nowMillis;
}

ButtonEvent ButtonGestures::update(bool pressedRaw, uint32_t nowMillis) {
    if (pressedRaw != raw_) {
        raw_ = pressedRaw;
        lastEdgeMillis_ = nowMillis;
    }

    // Unsigned subtraction throughout: millis() wraps every ~49 days, and a
    // `now > last + period` comparison would stop working across the wrap.
    const bool stable = (nowMillis - lastEdgeMillis_) >= timing_.debounceMillis;
    if (stable && debounced_ != raw_) {
        debounced_ = raw_;
        if (debounced_) {
            pressStartMillis_ = nowMillis;
            holdConsumed_ = false;
            longFired_ = false;
            veryLongFired_ = false;
        } else if (!holdConsumed_) {
            // A clean short press: it either completes a double click or opens
            // the window in which one might arrive.
            if (clickPending_) {
                clickPending_ = false;
                return ButtonEvent::DoubleClick;
            }
            clickPending_ = true;
            clickPendingSinceMillis_ = nowMillis;
        }
    }

    if (debounced_) {
        const uint32_t heldFor = nowMillis - pressStartMillis_;
        // Most severe first, each latching its own flag. Holding past 1.2 s
        // starts calibration and buzzes; carrying on to 4 s escalates to
        // parking the hand, which aborts that calibration. The escalation is
        // intentional — a long hold on the only button the wearer can reach
        // has to end with the hand going limp.
        if (!veryLongFired_ && heldFor >= timing_.veryLongPressMillis) {
            veryLongFired_ = true;
            holdConsumed_ = true;
            clickPending_ = false;
            return ButtonEvent::VeryLongPress;
        }
        if (!longFired_ && heldFor >= timing_.longPressMillis) {
            longFired_ = true;
            holdConsumed_ = true;
            clickPending_ = false;
            return ButtonEvent::LongPress;
        }
    }

    if (clickPending_ && (nowMillis - clickPendingSinceMillis_) >= timing_.doubleClickMillis) {
        clickPending_ = false;
        return ButtonEvent::Click;
    }

    return ButtonEvent::None;
}

}  // namespace jensen
