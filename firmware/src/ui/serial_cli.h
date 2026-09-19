// Jensen Hand — serial console.
//
// This is the assembly and service interface. It is the only way to establish
// servo endpoints for a freshly built hand, because those depend on how each
// tendon was tensioned during assembly and cannot be guessed. It also streams
// telemetry for tuning (see `tools/jensen_monitor.py`).
//
// Bench mode
// ----------
// `cal digit <name>` takes the servo bus away from the controller so a single
// channel can be jogged to its mechanical limits. While that is active the
// hand is parked and the main loop stops applying controller outputs — see
// `benchMode()`. Nothing else in the firmware writes to the bus at that point,
// so there is no contention to reason about.
#pragma once

#include <Arduino.h>

#include <cstdint>

#include "hal/config_store.h"
#include "hal/servo_bus.h"
#include "jensen/calibration.h"
#include "jensen/hand_config.h"
#include "jensen/hand_controller.h"

namespace jensen {

class SerialCli {
public:
    void begin(Stream& stream, HandController& hand, ConfigStore& store, ServoBus& servos);

    /// Reads pending input and emits telemetry. Non-blocking.
    void update(uint32_t nowMillis);

    /// True while `cal digit` owns the servo bus.
    bool benchMode() const { return digitRunner_.active(); }

    /// Called by the application when the controller has changed the stored
    /// configuration behind the CLI's back (after an electrode calibration),
    /// so the working copy does not overwrite it on the next `set`.
    void refreshWorkingConfig();

private:
    void handleLine(char* line);
    void printPrompt();
    void printHelp();
    void printStatus();
    void printConfig();
    void printGrips();
    void printTelemetry();

    bool applySetting(const char* key, const char* value);
    bool applyFloatSetting(const char* key, float value);
    void commitConfig(const char* what);

    void beginDigitCalibration(const char* digitName);
    void handleDigitCalibration(int argc, char** argv);
    void endDigitCalibration(bool save);

    Stream* out_ = nullptr;
    HandController* hand_ = nullptr;
    ConfigStore* store_ = nullptr;
    ServoBus* servos_ = nullptr;

    HandConfig working_{};

    static constexpr uint8_t kLineMax = 96;
    char line_[kLineMax] = {};
    uint8_t lineLength_ = 0;
    bool lineOverflow_ = false;
    bool wasParkedBeforeCalibration_ = true;

    bool telemetryEnabled_ = false;
    uint32_t telemetryPeriodMillis_ = 100;
    uint32_t lastTelemetryMillis_ = 0;

    DigitCalibrationRunner digitRunner_{};
};

}  // namespace jensen
