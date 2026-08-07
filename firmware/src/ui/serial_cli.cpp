#include "ui/serial_cli.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "board_config.h"

namespace jensen {

namespace {

constexpr uint8_t kMaxTokens = 8;

bool equals(const char* a, const char* b) { return std::strcmp(a, b) == 0; }

bool parseBool(const char* text, bool& out) {
    if (equals(text, "on") || equals(text, "1") || equals(text, "true") || equals(text, "yes")) {
        out = true;
        return true;
    }
    if (equals(text, "off") || equals(text, "0") || equals(text, "false") || equals(text, "no")) {
        out = false;
        return true;
    }
    return false;
}

/// Digit lookup by the same names the telemetry prints.
bool parseDigit(const char* text, Digit& out) {
    for (uint8_t i = 0; i < kDigitCount; ++i) {
        const Digit digit = static_cast<Digit>(i);
        if (equals(text, digitName(digit))) {
            out = digit;
            return true;
        }
    }
    return false;
}

bool parseGrip(const char* text, GripId& out) {
    for (uint8_t i = 0; i < kGripCount; ++i) {
        const GripId grip = static_cast<GripId>(i);
        if (equals(text, GripLibrary::name(grip))) {
            out = grip;
            return true;
        }
    }
    return false;
}

}  // namespace

void SerialCli::begin(Stream& stream, HandController& hand, ConfigStore& store,
                      ServoBus& servos) {
    out_ = &stream;
    hand_ = &hand;
    store_ = &store;
    servos_ = &servos;
    working_ = hand.config();
    lineLength_ = 0;

    out_->println();
    out_->println(F("Jensen Hand control stack"));
    out_->println(F("type 'help' for commands"));
    printPrompt();
}

void SerialCli::refreshWorkingConfig() {
    if (hand_ != nullptr) working_ = hand_->config();
}

void SerialCli::printPrompt() {
    if (out_ == nullptr) return;
    out_->print(digitRunner_.active() ? F("cal> ") : F("jensen> "));
}

void SerialCli::update(uint32_t nowMillis) {
    if (out_ == nullptr || hand_ == nullptr) return;

    // Bounded read per call: a wedged terminal must not stall the control loop.
    uint8_t budget = 64;
    while (out_->available() > 0 && budget-- > 0) {
        const char c = static_cast<char>(out_->read());
        if (c == '\r') continue;
        if (c == '\n') {
            line_[lineLength_] = '\0';
            out_->println();
            if (lineLength_ > 0) handleLine(line_);
            lineLength_ = 0;
            printPrompt();
            continue;
        }
        if (c == 8 || c == 127) {  // backspace / delete
            if (lineLength_ > 0) {
                --lineLength_;
                out_->print(F("\b \b"));
            }
            continue;
        }
        if (lineLength_ < kLineMax - 1) {
            line_[lineLength_++] = c;
            out_->print(c);
        }
    }

    if (telemetryEnabled_ && (nowMillis - lastTelemetryMillis_) >= telemetryPeriodMillis_) {
        lastTelemetryMillis_ = nowMillis;
        printTelemetry();
    }
}

void SerialCli::handleLine(char* line) {
    char* tokens[kMaxTokens] = {};
    int count = 0;
    char* saved = nullptr;
    for (char* token = strtok_r(line, " \t", &saved); token != nullptr && count < kMaxTokens;
         token = strtok_r(nullptr, " \t", &saved)) {
        tokens[count++] = token;
    }
    if (count == 0) return;

    // Bench mode has its own vocabulary; only `save`, `cancel` and `help` are
    // shared, so route everything there while it is active.
    if (digitRunner_.active()) {
        handleDigitCalibration(count, tokens);
        return;
    }

    const char* command = tokens[0];

    if (equals(command, "help") || equals(command, "?")) {
        printHelp();
    } else if (equals(command, "status")) {
        printStatus();
    } else if (equals(command, "config")) {
        printConfig();
    } else if (equals(command, "grips")) {
        printGrips();

    } else if (equals(command, "telem")) {
        if (count >= 2) {
            bool enabled = false;
            if (parseBool(tokens[1], enabled)) {
                telemetryEnabled_ = enabled;
            } else {
                out_->println(F("usage: telem on|off [hz]"));
                return;
            }
        } else {
            telemetryEnabled_ = !telemetryEnabled_;
        }
        if (count >= 3) {
            const float hz = static_cast<float>(std::atof(tokens[2]));
            if (hz > 0.0f && hz <= 200.0f) {
                telemetryPeriodMillis_ = static_cast<uint32_t>(1000.0f / hz);
            }
        }
        out_->printf("telemetry %s at %lu ms\r\n", telemetryEnabled_ ? "on" : "off",
                     static_cast<unsigned long>(telemetryPeriodMillis_));

    } else if (equals(command, "grip")) {
        if (count < 2) {
            out_->printf("grip %s\r\n", GripLibrary::name(hand_->grip()));
            return;
        }
        if (equals(tokens[1], "next")) {
            hand_->nextGrip();
        } else if (equals(tokens[1], "prev")) {
            hand_->previousGrip();
        } else if (equals(tokens[1], "enable") && count >= 4) {
            GripId grip;
            bool enabled = false;
            if (!parseGrip(tokens[2], grip) || !parseBool(tokens[3], enabled)) {
                out_->println(F("usage: grip enable <name> on|off"));
                return;
            }
            const uint16_t bit = static_cast<uint16_t>(1u << static_cast<uint8_t>(grip));
            working_.enabledGrips = enabled ? (working_.enabledGrips | bit)
                                            : (working_.enabledGrips & ~bit);
            commitConfig("grip mask");
            return;
        } else {
            GripId grip;
            if (!parseGrip(tokens[1], grip)) {
                out_->println(F("unknown grip; try 'grips'"));
                return;
            }
            hand_->selectGrip(grip);
        }
        out_->printf("grip %s\r\n", GripLibrary::name(hand_->grip()));

    } else if (equals(command, "hold")) {
        bool enabled = (hand_->telemetry().mode != ControlMode::Hold);
        if (count >= 2 && !parseBool(tokens[1], enabled)) {
            out_->println(F("usage: hold [on|off]"));
            return;
        }
        hand_->setHold(enabled);
        out_->printf("hold %s\r\n", enabled ? "on" : "off");

    } else if (equals(command, "park")) {
        bool enabled = (hand_->telemetry().mode != ControlMode::SafeTorqueOff);
        if (count >= 2 && !parseBool(tokens[1], enabled)) {
            out_->println(F("usage: park [on|off]"));
            return;
        }
        hand_->park(enabled);
        out_->printf("park %s\r\n", enabled ? "on" : "off");

    } else if (equals(command, "open")) {
        hand_->commandAperture(0.0f);
        out_->println(F("opening"));
    } else if (equals(command, "close")) {
        hand_->commandAperture(1.0f);
        out_->println(F("closing"));
    } else if (equals(command, "aperture")) {
        if (count < 2) {
            out_->println(F("usage: aperture <0..1>"));
            return;
        }
        const float value = static_cast<float>(std::atof(tokens[1]));
        hand_->commandAperture(value);
        out_->printf("aperture %.3f\r\n", static_cast<double>(value));
    } else if (equals(command, "wrist")) {
        if (count < 2) {
            out_->println(F("usage: wrist <0..1>"));
            return;
        }
        hand_->setWristTarget(static_cast<float>(std::atof(tokens[1])));
        out_->println(F("ok"));

    } else if (equals(command, "cal")) {
        if (count < 2) {
            out_->println(F("usage: cal emg | cal fsr | cal digit <name> | cal abort"));
            return;
        }
        if (equals(tokens[1], "emg")) {
            hand_->startEmgCalibration();
            out_->println(F("electrode calibration started — follow the prompts"));
        } else if (equals(tokens[1], "abort")) {
            hand_->abortCalibration();
            out_->println(F("aborted"));
        } else if (equals(tokens[1], "fsr")) {
            hand_->tareForceSensors();
            refreshWorkingConfig();
            out_->println(F("fingertip sensors tared (hand must be touching nothing)"));
        } else if (equals(tokens[1], "digit") && count >= 3) {
            beginDigitCalibration(tokens[2]);
        } else {
            out_->println(F("usage: cal emg | cal fsr | cal digit <name> | cal abort"));
        }

    } else if (equals(command, "set")) {
        if (count < 3) {
            out_->println(F("usage: set <key> <value>   (see 'config')"));
            return;
        }
        if (applySetting(tokens[1], tokens[2])) {
            commitConfig(tokens[1]);
        } else {
            out_->printf("unknown or invalid setting: %s\r\n", tokens[1]);
        }

    } else if (equals(command, "side")) {
        if (count < 2) {
            out_->printf("side %s\r\n", handSideName(working_.side));
            return;
        }
        if (equals(tokens[1], "left")) {
            working_ = HandConfig::defaults(HandSide::Left);
        } else if (equals(tokens[1], "right")) {
            working_ = HandConfig::defaults(HandSide::Right);
        } else {
            out_->println(F("usage: side left|right"));
            return;
        }
        out_->println(F("factory defaults loaded for that side; run 'cal digit' for each digit"));
        commitConfig("side");

    } else if (equals(command, "save")) {
        working_ = hand_->config();
        if (store_ != nullptr && store_->save(working_)) {
            out_->printf("saved to slot %u\r\n", static_cast<unsigned>(store_->activeSlot()));
        } else {
            out_->println(F("save FAILED"));
        }

    } else if (equals(command, "load")) {
        HandConfig loaded{};
        if (store_ != nullptr && store_->load(loaded)) {
            working_ = loaded;
            hand_->applyConfig(working_);
            out_->println(F("loaded"));
        } else {
            out_->println(F("no stored configuration"));
        }

    } else if (equals(command, "defaults")) {
        HandSide side = working_.side;
        if (count >= 2) {
            if (equals(tokens[1], "left")) side = HandSide::Left;
            else if (equals(tokens[1], "right")) side = HandSide::Right;
        }
        working_ = HandConfig::defaults(side);
        hand_->applyConfig(working_);
        out_->println(F("factory defaults applied (not saved — use 'save')"));

    } else if (equals(command, "erase")) {
        if (store_ != nullptr && store_->erase()) {
            out_->println(F("stored configuration erased; reboot to come up on defaults"));
        } else {
            out_->println(F("erase FAILED"));
        }

    } else if (equals(command, "reboot")) {
        out_->println(F("rebooting"));
        out_->flush();
        ESP.restart();

    } else {
        out_->printf("unknown command: %s (try 'help')\r\n", command);
    }
}

void SerialCli::commitConfig(const char* what) {
    if (hand_ == nullptr) return;
    if (!working_.isValid()) {
        out_->println(F("rejected: configuration would be invalid"));
        working_ = hand_->config();
        return;
    }
    hand_->applyConfig(working_);
    out_->printf("%s updated (use 'save' to persist)\r\n", what);
}

bool SerialCli::applySetting(const char* key, const char* value) {
    // Non-numeric settings first.
    if (equals(key, "drive.mode")) {
        if (equals(value, "dual")) {
            working_.drive.mode = DriveMode::DualChannel;
            return true;
        }
        if (equals(value, "single")) {
            working_.drive.mode = DriveMode::SingleChannel;
            return true;
        }
        return false;
    }
    if (equals(key, "haptics")) {
        return parseBool(value, working_.hapticsEnabled);
    }
    if (equals(key, "idle.release")) {
        return parseBool(value, working_.idleReleaseEnabled);
    }
    if (equals(key, "grip.default")) {
        GripId grip;
        if (!parseGrip(value, grip)) return false;
        working_.defaultGrip = grip;
        return true;
    }

    return applyFloatSetting(key, static_cast<float>(std::atof(value)));
}

bool SerialCli::applyFloatSetting(const char* key, float value) {
    // Electrode conditioning
    if (equals(key, "emg.attack"))       { working_.emg.attackSeconds = value;  return true; }
    if (equals(key, "emg.release"))      { working_.emg.releaseSeconds = value; return true; }
    if (equals(key, "emg.deadband"))     { working_.emg.deadband = value;       return true; }
    if (equals(key, "emg.gamma"))        { working_.emg.gamma = value;          return true; }
    if (equals(key, "emg.noise_margin")) { working_.emg.noiseMargin = value;    return true; }

    // Drive law
    if (equals(key, "drive.close_speed")) { working_.drive.closeSpeed = value;       return true; }
    if (equals(key, "drive.open_speed"))  { working_.drive.openSpeed = value;        return true; }
    if (equals(key, "drive.single_open")) { working_.drive.singleOpenBelow = value;  return true; }
    if (equals(key, "drive.single_hold")) { working_.drive.singleHoldBelow = value;  return true; }
    if (equals(key, "drive.co_level"))    { working_.drive.coContractionLevel = value; return true; }
    if (equals(key, "drive.co_seconds"))  { working_.drive.coContractionSeconds = value; return true; }

    // Grip force
    if (equals(key, "force.target"))    { working_.gripForce.targetNewtons = value;   return true; }
    if (equals(key, "force.max"))       { working_.gripForce.maxNewtons = value;      return true; }
    if (equals(key, "force.gain"))      { working_.gripForce.integralGain = value;    return true; }
    if (equals(key, "force.slip_step")) { working_.gripForce.slipTightenStep = value; return true; }
    if (equals(key, "force.release"))   { working_.gripForce.releasePerSecond = value; return true; }
    if (equals(key, "force.contact_on"))  { working_.forceSensor.contactOnNewtons = value;  return true; }
    if (equals(key, "force.contact_off")) { working_.forceSensor.contactOffNewtons = value; return true; }
    if (equals(key, "force.slip_rate"))   { working_.forceSensor.slipRateNewtonsPerSec = value; return true; }

    // Supervision
    if (equals(key, "battery.warn"))     { working_.battery.warnCellVolts = value;     return true; }
    if (equals(key, "battery.critical")) { working_.battery.criticalCellVolts = value; return true; }
    if (equals(key, "hold.engage"))      { working_.holdGuard.engageNewtons = value;   return true; }
    if (equals(key, "hold.seconds"))     { working_.holdGuard.holdSeconds = value;     return true; }
    if (equals(key, "idle.timeout"))     { working_.idleTimeoutSeconds = value;        return true; }

    // Per-digit speed:  set speed.index 2.4
    if (std::strncmp(key, "speed.", 6) == 0) {
        Digit digit;
        if (!parseDigit(key + 6, digit)) return false;
        working_.digitSpeed[index(digit)] = value;
        return true;
    }

    // Per-grip force scale:  set scale.pinch 0.4
    if (std::strncmp(key, "scale.", 6) == 0) {
        GripId grip;
        if (!parseGrip(key + 6, grip)) return false;
        working_.gripForceScale[static_cast<uint8_t>(grip)] = value;
        return true;
    }

    return false;
}

// --- Bench calibration -------------------------------------------------------

void SerialCli::beginDigitCalibration(const char* digitName) {
    Digit digit;
    if (!parseDigit(digitName, digit)) {
        out_->println(F("unknown digit; try: thumb thumb_rot index middle ring pinky wrist"));
        return;
    }
    if (hand_ == nullptr || servos_ == nullptr) return;

    working_ = hand_->config();
    digitRunner_.begin(digit, working_.digits[index(digit)]);

    // Take the bus: park the controller, then energise the rail ourselves so
    // exactly one channel is under command.
    hand_->park(true);
    servos_->setRailEnabled(true);
    servos_->setOutputEnabled(true);
    for (uint8_t i = 0; i < kDigitCount; ++i) servos_->write(board::kServoChannel[i], 0);
    servos_->write(board::kServoChannel[index(digit)], digitRunner_.pulseUs());

    out_->printf("\r\ncalibrating %s — servo channel %u\r\n", digitName,
                 static_cast<unsigned>(board::kServoChannel[index(digit)]));
    out_->println(F("  jog <us>     move by a signed number of microseconds (e.g. jog -25)"));
    out_->println(F("  pulse <us>   go to an absolute pulse width"));
    out_->println(F("  open         mark this position as fully open"));
    out_->println(F("  closed       mark this position as fully closed"));
    out_->println(F("  save         store both endpoints and leave"));
    out_->println(F("  cancel       leave without changing anything"));
    out_->println(F("jog in small steps and stop before the tendon goes tight."));
}

void SerialCli::handleDigitCalibration(int argc, char** argv) {
    const char* command = argv[0];

    if (equals(command, "jog") && argc >= 2) {
        const int32_t delta = std::atol(argv[1]);
        const uint16_t pulse = digitRunner_.jog(static_cast<int16_t>(delta));
        servos_->write(board::kServoChannel[index(digitRunner_.digit())], pulse);
        out_->printf("%u us\r\n", static_cast<unsigned>(pulse));

    } else if (equals(command, "pulse") && argc >= 2) {
        digitRunner_.setPulseUs(static_cast<uint16_t>(std::atol(argv[1])));
        servos_->write(board::kServoChannel[index(digitRunner_.digit())], digitRunner_.pulseUs());
        out_->printf("%u us\r\n", static_cast<unsigned>(digitRunner_.pulseUs()));

    } else if (equals(command, "open")) {
        digitRunner_.markOpen();
        out_->printf("open = %u us\r\n", static_cast<unsigned>(digitRunner_.pulseUs()));

    } else if (equals(command, "closed")) {
        digitRunner_.markClosed();
        out_->printf("closed = %u us\r\n", static_cast<unsigned>(digitRunner_.pulseUs()));

    } else if (equals(command, "save")) {
        endDigitCalibration(true);

    } else if (equals(command, "cancel")) {
        endDigitCalibration(false);

    } else if (equals(command, "help") || equals(command, "?")) {
        out_->println(F("jog <us> | pulse <us> | open | closed | save | cancel"));

    } else {
        out_->println(F("in calibration: jog <us> | pulse <us> | open | closed | save | cancel"));
    }
}

void SerialCli::endDigitCalibration(bool save) {
    const Digit digit = digitRunner_.digit();

    if (save) {
        DigitCalibration result{};
        if (!digitRunner_.result(result)) {
            out_->println(F("not saved: mark both 'open' and 'closed' first, "
                            "and leave real travel between them"));
            return;
        }
        working_.digits[index(digit)] = result;
        out_->printf("%s: open %u us, closed %u us, guards %u–%u us\r\n", digitName(digit),
                     static_cast<unsigned>(result.openUs), static_cast<unsigned>(result.closedUs),
                     static_cast<unsigned>(result.minUs), static_cast<unsigned>(result.maxUs));
    }

    digitRunner_.abort();
    servos_->write(board::kServoChannel[index(digit)], 0);

    if (save) {
        hand_->applyConfig(working_);
        out_->println(F("applied — use 'save' to persist"));
    } else {
        working_ = hand_->config();
        out_->println(F("cancelled"));
    }

    // Hand the bus back. The controller re-energises on its own next tick.
    hand_->park(false);
}

// --- Reporting ---------------------------------------------------------------

void SerialCli::printHelp() {
    out_->println(F(
        "\r\nstatus                     one-shot state summary\r\n"
        "telem [on|off] [hz]        stream telemetry (tools/jensen_monitor.py parses it)\r\n"
        "config                     print the active configuration\r\n"
        "grips                      list grip patterns and which are enabled\r\n"
        "\r\n"
        "grip [next|prev|<name>]    select a grip pattern\r\n"
        "grip enable <name> on|off  add or remove a pattern from the cycle\r\n"
        "hold [on|off]              freeze the hand where it is\r\n"
        "park [on|off]              safe torque off / wake up\r\n"
        "open | close               direct aperture command (muscle activity overrides)\r\n"
        "aperture <0..1>            direct aperture command\r\n"
        "wrist <0..1>               wrist rotator, 0.5 is neutral\r\n"
        "\r\n"
        "cal emg                    guided electrode calibration\r\n"
        "cal fsr                    tare the fingertip sensors (touch nothing)\r\n"
        "cal digit <name>           jog a servo to its endpoints\r\n"
        "cal abort                  stop a running calibration\r\n"
        "\r\n"
        "set <key> <value>          change a tuning value (keys shown by 'config')\r\n"
        "side left|right            reload factory defaults for a handedness\r\n"
        "save | load | defaults     persist / restore / reset the configuration\r\n"
        "erase                      wipe stored configuration\r\n"
        "reboot"));
}

void SerialCli::printStatus() {
    const HandTelemetry& t = hand_->telemetry();
    out_->printf("mode      %s\r\n", controlModeName(t.mode));
    out_->printf("grip      %s   aperture %.2f   velocity %+.2f/s\r\n", t.gripName,
                 static_cast<double>(t.aperture), static_cast<double>(t.velocity));
    out_->printf("emg       flex %.2f  ext %.2f   (envelopes %.0f / %.0f counts)\r\n",
                 static_cast<double>(t.emgFlexor), static_cast<double>(t.emgExtensor),
                 static_cast<double>(t.emgFlexorEnvelope),
                 static_cast<double>(t.emgExtensorEnvelope));
    out_->printf("force     peak %.1f N / target %.1f N   limit %.2f   contact %s   slips %lu\r\n",
                 static_cast<double>(t.peakForce), static_cast<double>(t.forceTarget),
                 static_cast<double>(t.closureLimit), t.contact ? "yes" : "no",
                 static_cast<unsigned long>(t.slipCount));
    // Analogue force against the LM393 comparators. A digit reading real load
    // whose comparator never trips (or trips with nothing in the hand) means
    // that comparator's threshold pot needs adjusting.
    out_->print(F("fsr      "));
    for (uint8_t i = 0; i < kGraspDigitCount; ++i) {
        if (!working_.force[i].present) continue;
        out_->printf(" %s %.1fN/cmp:%s", digitName(static_cast<Digit>(i)),
                     static_cast<double>(t.force[i]), t.contactDigital[i] ? "on" : "off");
    }
    out_->println();
    out_->printf("battery   %.2f V pack, %.2f V/cell, %.0f%%\r\n",
                 static_cast<double>(t.batteryVolts), static_cast<double>(t.batteryCellVolts),
                 static_cast<double>(t.stateOfCharge * 100.0f));
    out_->printf("faults    0x%04X %s\r\n", static_cast<unsigned>(t.faults),
                 t.faultName ? t.faultName : "none");
    out_->printf("loop      %.2f ms, %lu iterations, up %lu s\r\n",
                 static_cast<double>(t.loopSeconds * 1000.0f),
                 static_cast<unsigned long>(t.loopCount),
                 static_cast<unsigned long>(t.uptimeSeconds));

    out_->println(F("digit        closure  target   pulse"));
    for (uint8_t i = 0; i < kDigitCount; ++i) {
        const DigitTelemetry& d = t.digits[i];
        out_->printf("  %-10s %6.2f  %6.2f  %5u us%s%s\r\n", digitName(static_cast<Digit>(i)),
                     static_cast<double>(d.closure), static_cast<double>(d.target),
                     static_cast<unsigned>(d.pulseUs), d.locked ? "  [locked]" : "",
                     d.released ? "  [released]" : "");
    }
}

void SerialCli::printGrips() {
    const GripId active = hand_->grip();
    for (uint8_t i = 0; i < kGripCount; ++i) {
        const GripId grip = static_cast<GripId>(i);
        const bool enabled =
            (working_.enabledGrips & (1u << i)) != 0 || grip == GripId::Rest;
        out_->printf("  %c %-8s %-42s scale %.2f\r\n", grip == active ? '*' : ' ',
                     GripLibrary::name(grip), GripLibrary::pattern(grip).description,
                     static_cast<double>(working_.gripForceScale[i]));
        if (!enabled) out_->println(F("      (disabled — not in the cycle)"));
    }
}

void SerialCli::printConfig() {
    const HandConfig& c = working_;
    out_->printf("side %s   grips 0x%04X   default %s   haptics %s   idle %.0f s / release %s\r\n",
                 handSideName(c.side), static_cast<unsigned>(c.enabledGrips),
                 GripLibrary::name(c.defaultGrip), c.hapticsEnabled ? "on" : "off",
                 static_cast<double>(c.idleTimeoutSeconds), c.idleReleaseEnabled ? "on" : "off");

    out_->println(F("\r\nservo endpoints (cal digit <name>)"));
    for (uint8_t i = 0; i < kDigitCount; ++i) {
        const DigitCalibration& d = c.digits[i];
        out_->printf("  %-10s open %4u  closed %4u  guards %4u-%4u  speed %.2f  %s\r\n",
                     digitName(static_cast<Digit>(i)), static_cast<unsigned>(d.openUs),
                     static_cast<unsigned>(d.closedUs), static_cast<unsigned>(d.minUs),
                     static_cast<unsigned>(d.maxUs), static_cast<double>(c.digitSpeed[i]),
                     d.enabled ? (d.isValid() ? "ok" : "INVALID") : "disabled");
    }

    out_->println(F("\r\nemg.*        attack release deadband gamma noise_margin"));
    out_->printf("             %.3f  %.3f   %.3f    %.2f  %.2f\r\n",
                 static_cast<double>(c.emg.attackSeconds),
                 static_cast<double>(c.emg.releaseSeconds), static_cast<double>(c.emg.deadband),
                 static_cast<double>(c.emg.gamma), static_cast<double>(c.emg.noiseMargin));
    out_->printf("  flexor   %s  baseline %.0f noise %.1f mvc %.0f\r\n",
                 c.emgFlexor.isUsable() ? "ok      " : "MISSING ",
                 static_cast<double>(c.emgFlexor.baselineCounts),
                 static_cast<double>(c.emgFlexor.noiseCounts),
                 static_cast<double>(c.emgFlexor.mvcCounts));
    out_->printf("  extensor %s  baseline %.0f noise %.1f mvc %.0f\r\n",
                 c.emgExtensor.isUsable() ? "ok      " : "MISSING ",
                 static_cast<double>(c.emgExtensor.baselineCounts),
                 static_cast<double>(c.emgExtensor.noiseCounts),
                 static_cast<double>(c.emgExtensor.mvcCounts));

    out_->printf("\r\ndrive.*      mode %s  close_speed %.2f  open_speed %.2f\r\n",
                 c.drive.mode == DriveMode::DualChannel ? "dual" : "single",
                 static_cast<double>(c.drive.closeSpeed),
                 static_cast<double>(c.drive.openSpeed));
    out_->printf("             single_open %.2f  single_hold %.2f  co_level %.2f  co_seconds %.2f\r\n",
                 static_cast<double>(c.drive.singleOpenBelow),
                 static_cast<double>(c.drive.singleHoldBelow),
                 static_cast<double>(c.drive.coContractionLevel),
                 static_cast<double>(c.drive.coContractionSeconds));

    out_->printf("\r\nforce.*      target %.1f N  max %.1f N  gain %.3f  slip_step %.3f  release %.2f\r\n",
                 static_cast<double>(c.gripForce.targetNewtons),
                 static_cast<double>(c.gripForce.maxNewtons),
                 static_cast<double>(c.gripForce.integralGain),
                 static_cast<double>(c.gripForce.slipTightenStep),
                 static_cast<double>(c.gripForce.releasePerSecond));
    out_->printf("             contact_on %.2f N  contact_off %.2f N  slip_rate %.1f N/s\r\n",
                 static_cast<double>(c.forceSensor.contactOnNewtons),
                 static_cast<double>(c.forceSensor.contactOffNewtons),
                 static_cast<double>(c.forceSensor.slipRateNewtonsPerSec));
    for (uint8_t i = 0; i < kGraspDigitCount; ++i) {
        if (!c.force[i].present) continue;
        out_->printf("  fsr %-8s zero %4u  span %4u  ref %.1f N  exp %.2f\r\n",
                     digitName(static_cast<Digit>(i)),
                     static_cast<unsigned>(c.force[i].zeroCounts),
                     static_cast<unsigned>(c.force[i].spanCounts),
                     static_cast<double>(c.force[i].referenceNewtons),
                     static_cast<double>(c.force[i].exponent));
    }

    out_->printf("\r\nbattery.*    warn %.2f V/cell  critical %.2f V/cell  cells %u\r\n",
                 static_cast<double>(c.battery.warnCellVolts),
                 static_cast<double>(c.battery.criticalCellVolts),
                 static_cast<unsigned>(c.battery.cellCount));
    out_->printf("hold.*       engage %.1f N  seconds %.0f  cool %.0f  backoff %.2f\r\n",
                 static_cast<double>(c.holdGuard.engageNewtons),
                 static_cast<double>(c.holdGuard.holdSeconds),
                 static_cast<double>(c.holdGuard.coolSeconds),
                 static_cast<double>(c.holdGuard.backoffScale));
    out_->println(F("\r\nalso: speed.<digit>, scale.<grip>, grip.default, haptics, idle.release"));
}

void SerialCli::printTelemetry() {
    const HandTelemetry& t = hand_->telemetry();
    // One line, key=value, comma separated. Parsed by tools/jensen_monitor.py.
    out_->printf("T,t=%lu,mode=%s,grip=%s,ap=%.4f,vel=%+.4f,ef=%.4f,ee=%.4f",
                 static_cast<unsigned long>(millis()), controlModeName(t.mode), t.gripName,
                 static_cast<double>(t.aperture), static_cast<double>(t.velocity),
                 static_cast<double>(t.emgFlexor), static_cast<double>(t.emgExtensor));
    for (uint8_t i = 0; i < kGraspDigitCount; ++i) {
        out_->printf(",f%u=%.2f", static_cast<unsigned>(i), static_cast<double>(t.force[i]));
    }
    uint8_t comparators = 0;
    for (uint8_t i = 0; i < kGraspDigitCount; ++i) {
        if (t.contactDigital[i]) comparators |= static_cast<uint8_t>(1u << i);
    }
    out_->printf(",cd=%u", static_cast<unsigned>(comparators));
    out_->printf(",pk=%.2f,tgt=%.2f,lim=%.3f,ct=%u,slip=%lu",
                 static_cast<double>(t.peakForce), static_cast<double>(t.forceTarget),
                 static_cast<double>(t.closureLimit), t.contact ? 1u : 0u,
                 static_cast<unsigned long>(t.slipCount));
    out_->printf(",vb=%.2f,soc=%.3f,flt=0x%04X", static_cast<double>(t.batteryVolts),
                 static_cast<double>(t.stateOfCharge), static_cast<unsigned>(t.faults));
    for (uint8_t i = 0; i < kDigitCount; ++i) {
        out_->printf(",p%u=%u", static_cast<unsigned>(i),
                     static_cast<unsigned>(t.digits[i].pulseUs));
    }
    out_->printf(",loop=%.4f\r\n", static_cast<double>(t.loopSeconds));
}

}  // namespace jensen
