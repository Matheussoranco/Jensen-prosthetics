// Jensen Hand — application entry point.
//
// This file is deliberately thin. It samples the hardware into a `HandInputs`,
// hands it to the control core, and applies the `HandOutputs` that come back.
// Every decision about *what the hand should do* lives in `lib/jensen_core`,
// which is why the behaviour of the prosthesis can be tested on a host without
// a hand attached.
//
//   setup()  bring up peripherals, load the stored configuration, self-test
//   loop()   run the control law on a fixed 100 Hz tick, then service the UI
//
// Nothing here blocks. `delay()` does not appear in this firmware: a hand that
// stops reading its electrodes to redraw a display is a hand that stutters
// under the user's control.

#include <Arduino.h>
#include <esp_task_wdt.h>

#include "board_config.h"
#include "hal/analog_in.h"
#include "hal/button.h"
#include "hal/config_store.h"
#include "hal/feedback.h"
#include "hal/servo_bus.h"
#include "jensen/hand_config.h"
#include "jensen/hand_controller.h"
#include "ui/nextion.h"
#include "ui/serial_cli.h"

namespace {

using namespace jensen;

HandController g_hand;
ServoBus g_servos;
ConfigStore g_store;
Button g_button;
Haptics g_haptics;
StatusLed g_statusLed;
NextionDisplay g_display;
SerialCli g_cli;

HandInputs g_inputs;

uint32_t g_lastControlMicros = 0;
uint32_t g_lastServoPollMillis = 0;
uint32_t g_lastHeartbeatMillis = 0;
bool g_heartbeatState = false;

#if defined(JENSEN_DEFAULT_SIDE_RIGHT) && JENSEN_DEFAULT_SIDE_RIGHT
constexpr HandSide kDefaultSide = HandSide::Right;
#else
constexpr HandSide kDefaultSide = HandSide::Left;
#endif

/// Samples every input into the struct the control core consumes.
void sampleInputs() {
    g_inputs.emgFlexorCounts =
        static_cast<float>(analog::readCounts(board::kPinEmgFlexor, board::kAdcOversample));
    g_inputs.emgExtensorCounts =
        static_cast<float>(analog::readCounts(board::kPinEmgExtensor, board::kAdcOversample));
    for (uint8_t i = 0; i < kPatternSiteCount - 2; ++i) {
        g_inputs.emgAuxCounts[i] =
            static_cast<float>(analog::readCounts(board::kPinEmgAux[i], board::kAdcOversample));
    }

    for (uint8_t i = 0; i < kGraspDigitCount; ++i) {
        g_inputs.fsrCounts[i] =
            static_cast<float>(analog::readCounts(board::kPinFsr[i], board::kAdcOversample));
        g_inputs.contactDigital[i] =
            board::kPinContact[i] >= 0 && digitalRead(board::kPinContact[i]) == LOW;
    }

    g_inputs.batteryVolts = analog::readBatteryVolts(
        board::kPinBatterySense, board::kBatteryDividerRatio, board::kAdcOversample);

    for (uint8_t i = 0; i < kDigitCount; ++i) {
        g_inputs.motorCurrentAmps[i] = analog::readCurrentAmps(
            board::kPinMotorCurrent[i], board::kMotorCurrentZeroVolts[i],
            board::kMotorCurrentVoltsPerAmp[i], board::kAdcOversample);
    }

    g_inputs.servoBusOk = g_servos.healthy();
}

/// Applies one iteration's outputs to the hardware.
void applyOutputs(const HandOutputs& outputs) {
    // Rail first when energising and last when de-energising, so the outputs
    // are never live against a rail that is still settling.
    if (outputs.servoRailEnabled) {
        g_servos.setRailEnabled(true);
        g_servos.setOutputEnabled(outputs.servoOutputEnabled);
    } else {
        g_servos.setOutputEnabled(false);
        g_servos.setRailEnabled(false);
    }

    for (uint8_t i = 0; i < kDigitCount; ++i) {
        g_servos.write(board::kServoChannel[i], outputs.pulseUs[i]);
    }

    g_statusLed.set(outputs.led);
    if (outputs.haptic != HapticCue::None) g_haptics.play(outputs.haptic);
}

/// Routes a Nextion touch command into the controller.
void handleDisplayCommand(NextionDisplay::Command command, uint8_t argument) {
    switch (command) {
        case NextionDisplay::Command::NextGrip:
            g_hand.nextGrip();
            break;
        case NextionDisplay::Command::PreviousGrip:
            g_hand.previousGrip();
            break;
        case NextionDisplay::Command::SelectGrip:
            if (argument < kGripCount) g_hand.selectGrip(static_cast<GripId>(argument));
            break;
        case NextionDisplay::Command::ToggleHold:
            g_hand.setHold(g_hand.telemetry().mode != ControlMode::Hold);
            break;
        case NextionDisplay::Command::TogglePark:
            g_hand.park(g_hand.telemetry().mode != ControlMode::SafeTorqueOff);
            break;
        case NextionDisplay::Command::Calibrate:
            g_hand.startEmgCalibration();
            break;
        case NextionDisplay::Command::Open:
            g_hand.commandAperture(0.0f);
            break;
        case NextionDisplay::Command::Close:
            g_hand.commandAperture(1.0f);
            break;
        case NextionDisplay::Command::RequestRefresh:
        case NextionDisplay::Command::None:
        default:
            break;
    }
}

}  // namespace

void setup() {
    Serial.begin(jensen::board::kConsoleBaud);

    pinMode(jensen::board::kPinHeartbeatLed, OUTPUT);
    digitalWrite(jensen::board::kPinHeartbeatLed, LOW);

    for (int pin : jensen::board::kPinContact) {
        if (pin >= 0) pinMode(pin, INPUT_PULLUP);
    }

    jensen::analog::begin();
    g_button.begin(jensen::board::kPinButton);
    g_haptics.begin(jensen::board::kPinHaptic, jensen::board::kLedcChannelHaptic);
    g_statusLed.begin(jensen::board::kPinStatusLed, jensen::board::kLedcChannelStatusLed);

    // The servo bus comes up in safe torque off and stays there until the
    // control core asks for torque, which it will not do until its self-test
    // has passed.
    const bool servosOk = g_servos.begin();

    jensen::HandConfig config = jensen::HandConfig::defaults(kDefaultSide);
    const bool storeOk = g_store.begin();
    if (storeOk) {
        jensen::HandConfig stored{};
        if (g_store.load(stored)) {
            config = stored;
        }
    }

    g_hand.begin(config);
    g_haptics.setEnabled(config.hapticsEnabled);

    g_display.begin(Serial2);
    g_cli.begin(Serial, g_hand, g_store, g_servos);

    Serial.printf("servo bus %s, config %s, side %s\r\n", servosOk ? "ok" : "NOT FOUND",
                  storeOk ? "storage ready" : "STORAGE UNAVAILABLE",
                  jensen::handSideName(config.side));
    if (!config.digitsCalibrated()) {
        Serial.println(F("digits are on placeholder endpoints — run 'cal digit <name>' for each"));
    }
    if (!config.emgCalibrated()) {
        Serial.println(F("no electrode calibration stored — run 'cal emg'"));
    }

    // A wedged control loop must not leave servos holding their last command
    // indefinitely; the watchdog resets the ESP32, which brings the bus back up
    // in safe torque off.
    //
    // This is the Arduino-ESP32 2.x / IDF 4.x signature, which is why
    // platformio.ini pins espressif32@^6.5.0. Core 3.x replaced it with a
    // config struct: `esp_task_wdt_config_t cfg = {timeout_ms, mask, true};
    // esp_task_wdt_reconfigure(&cfg);`
    esp_task_wdt_init(jensen::board::kWatchdogSeconds, true);
    esp_task_wdt_add(nullptr);

    g_lastControlMicros = micros();
}

void loop() {
    const uint32_t nowMicros = micros();
    const uint32_t nowMillis = millis();

    // --- Fixed-rate control tick --------------------------------------------
    // `micros()` wraps every ~71 minutes; unsigned subtraction handles the wrap
    // correctly, which a naive `now > last + period` comparison would not.
    const uint32_t elapsedMicros = nowMicros - g_lastControlMicros;
    if (elapsedMicros >= jensen::board::kControlPeriodMicros) {
        g_lastControlMicros = nowMicros;
        const float dt = static_cast<float>(elapsedMicros) * 1e-6f;

        sampleInputs();
        g_inputs.button = g_button.poll(nowMillis);

        const jensen::HandOutputs& outputs = g_hand.update(g_inputs, dt);

        // While the CLI is jogging a servo for endpoint calibration it owns the
        // bus; applying controller output at the same time would fight it.
        if (!g_cli.benchMode()) {
            applyOutputs(outputs);
        } else {
            g_statusLed.set(jensen::LedPattern::FastBlink);
        }

        // An electrode calibration finishing is the one time the core changes
        // its own configuration, so it is persisted immediately rather than
        // waiting for the user to remember to type `save`.
        if (g_hand.consumeConfigDirty()) {
            if (g_store.save(g_hand.config())) {
                Serial.println(F("\r\ncalibration saved"));
            } else {
                Serial.println(F("\r\ncalibration could NOT be saved"));
            }
            g_cli.refreshWorkingConfig();
        }

        esp_task_wdt_reset();
    }

    // --- Everything else, at its own rate -----------------------------------
    g_haptics.update(nowMillis);
    g_statusLed.update(nowMillis);
    g_cli.update(nowMillis);

    uint8_t argument = 0;
    const NextionDisplay::Command command = g_display.poll(argument);
    if (command != NextionDisplay::Command::None) handleDisplayCommand(command, argument);
    g_display.update(g_hand.telemetry(), nowMillis);

    // A harness that works loose is a real failure mode in a worn device, so
    // the bus is re-probed continuously rather than only at boot.
    if ((nowMillis - g_lastServoPollMillis) >= 250) {
        g_lastServoPollMillis = nowMillis;
        g_servos.poll();
    }

    if ((nowMillis - g_lastHeartbeatMillis) >= 500) {
        g_lastHeartbeatMillis = nowMillis;
        g_heartbeatState = !g_heartbeatState;
        digitalWrite(jensen::board::kPinHeartbeatLed, g_heartbeatState ? HIGH : LOW);
    }
}
