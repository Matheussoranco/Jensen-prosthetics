#include "ui/nextion.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "board_config.h"

namespace jensen {

namespace {

/// Every Nextion instruction and every event frame ends with three 0xFF bytes.
constexpr uint8_t kTerminator = 0xFF;
/// Touch event frame: 0x65 <page> <component> <event> FF FF FF
constexpr uint8_t kTouchEvent = 0x65;
constexpr uint8_t kTouchFrameLength = 4;
constexpr uint8_t kEventPress = 0x01;

/// Component IDs on the main page. Documented in docs/nextion_hmi.md.
constexpr uint8_t kPageMain = 0;
constexpr uint8_t kBtnNextGrip = 1;
constexpr uint8_t kBtnPrevGrip = 2;
constexpr uint8_t kBtnHold = 3;
constexpr uint8_t kBtnCalibrate = 4;
constexpr uint8_t kBtnPark = 5;
constexpr uint8_t kBtnOpen = 6;
constexpr uint8_t kBtnClose = 7;
constexpr uint8_t kBtnRefresh = 8;
/// Direct grip selection: component (kGripSelectBase + GripId).
constexpr uint8_t kGripSelectBase = 20;

}  // namespace

void NextionDisplay::begin(HardwareSerial& serial) {
    serial_ = &serial;
    serial_->begin(board::kNextionBaud, SERIAL_8N1, board::kPinNextionRx, board::kPinNextionTx);

    // A panel that was mid-instruction when the ESP32 reset needs the current
    // command terminated before it will accept a new one.
    terminate();
    send("bkcmd=0");  // no acknowledgements; nothing here parses them
    send("page 0");
}

void NextionDisplay::terminate() {
    if (serial_ == nullptr) return;
    for (uint8_t i = 0; i < 3; ++i) serial_->write(kTerminator);
}

void NextionDisplay::send(const char* command) {
    if (serial_ == nullptr) return;
    serial_->print(command);
    terminate();
}

void NextionDisplay::sendText(const char* component, const char* value) {
    if (serial_ == nullptr) return;
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%s.txt=\"%s\"", component, value ? value : "");
    send(buffer);
}

void NextionDisplay::sendValue(const char* component, int32_t value) {
    if (serial_ == nullptr) return;
    char buffer[48];
    std::snprintf(buffer, sizeof(buffer), "%s.val=%ld", component, static_cast<long>(value));
    send(buffer);
}

void NextionDisplay::update(const HandTelemetry& telemetry, uint32_t nowMillis) {
    if (serial_ == nullptr) return;
    if ((nowMillis - lastRefreshMillis_) < board::kDisplayPeriodMillis) return;
    lastRefreshMillis_ = nowMillis;

    // Only differences go out. At 9600 baud a full refresh is ~40 ms of link
    // time, which would crowd out touch events at a 5 Hz update rate.
    if (telemetry.mode != lastMode_) {
        lastMode_ = telemetry.mode;
        sendText("tMode", controlModeName(telemetry.mode));
    }

    if (telemetry.grip != lastGrip_) {
        lastGrip_ = telemetry.grip;
        sendText("tGrip", telemetry.gripName);
    }

    const int16_t aperture = static_cast<int16_t>(telemetry.aperture * 100.0f + 0.5f);
    if (aperture != lastAperture_) {
        lastAperture_ = aperture;
        sendValue("jAperture", aperture);
    }

    // BatteryMonitor reports NaN until the first ADC sample seeds it;
    // casting NaN to int16_t is undefined, so hold the last display value
    // instead of pushing garbage in the first control ticks.
    if (!std::isnan(telemetry.batteryVolts) && !std::isnan(telemetry.stateOfCharge)) {
        const int16_t charge = static_cast<int16_t>(telemetry.stateOfCharge * 100.0f + 0.5f);
        if (charge != lastCharge_) {
            lastCharge_ = charge;
            sendValue("jBattery", charge);
            char volts[16];
            std::snprintf(volts, sizeof(volts), "%.2fV", static_cast<double>(telemetry.batteryVolts));
            sendText("tBattery", volts);
        }
    }

    const int16_t force = static_cast<int16_t>(telemetry.peakForce * 10.0f + 0.5f);
    if (force != lastForce_) {
        lastForce_ = force;
        char text[16];
        std::snprintf(text, sizeof(text), "%.1fN", static_cast<double>(telemetry.peakForce));
        sendText("tForce", text);
    }

    if (telemetry.faults != lastFaults_) {
        lastFaults_ = telemetry.faults;
        sendText("tFault", telemetry.faultName ? telemetry.faultName : "");
    }

    // Calibration prompts are the one thing the panel is genuinely needed for,
    // so they are pushed on every change including back to empty.
    if (telemetry.calibrationStep != lastCalibrationStep_) {
        lastCalibrationStep_ = telemetry.calibrationStep;
        sendText("tPrompt", telemetry.calibrationStep ? telemetry.calibrationStep : "");
    }
    if (telemetry.calibrationStep != nullptr) {
        sendValue("jCal", static_cast<int32_t>(telemetry.calibrationProgress * 100.0f));
    }
}

NextionDisplay::Command NextionDisplay::decodeTouch(uint8_t page, uint8_t component,
                                                    uint8_t event, uint8_t& argument) {
    argument = 0;
    if (event != kEventPress) return Command::None;
    if (page != kPageMain) return Command::None;

    if (component >= kGripSelectBase && component < kGripSelectBase + kGripCount) {
        argument = static_cast<uint8_t>(component - kGripSelectBase);
        return Command::SelectGrip;
    }

    switch (component) {
        case kBtnNextGrip:  return Command::NextGrip;
        case kBtnPrevGrip:  return Command::PreviousGrip;
        case kBtnHold:      return Command::ToggleHold;
        case kBtnCalibrate: return Command::Calibrate;
        case kBtnPark:      return Command::TogglePark;
        case kBtnOpen:      return Command::Open;
        case kBtnClose:     return Command::Close;
        case kBtnRefresh:   return Command::RequestRefresh;
        default:            return Command::None;
    }
}

NextionDisplay::Command NextionDisplay::poll(uint8_t& argument) {
    argument = 0;
    if (serial_ == nullptr) return Command::None;

    // Bounded work per call: a panel spewing noise must not be able to stall
    // the control loop.
    uint8_t budget = 32;
    while (serial_->available() > 0 && budget-- > 0) {
        const uint8_t byte = static_cast<uint8_t>(serial_->read());

        if (length_ < sizeof(buffer_)) {
            buffer_[length_++] = byte;
        } else {
            // Overlong frame: resynchronise rather than growing the buffer.
            length_ = 0;
            continue;
        }

        if (length_ < 3) continue;
        const bool terminated = buffer_[length_ - 1] == kTerminator &&
                                buffer_[length_ - 2] == kTerminator &&
                                buffer_[length_ - 3] == kTerminator;
        if (!terminated) continue;

        const uint8_t payload = static_cast<uint8_t>(length_ - 3);
        connected_ = true;

        if (payload == kTouchFrameLength && buffer_[0] == kTouchEvent) {
            uint8_t decodedArgument = 0;
            const Command command =
                decodeTouch(buffer_[1], buffer_[2], buffer_[3], decodedArgument);
            length_ = 0;
            if (command != Command::None) {
                pending_ = command;
                pendingArgument_ = decodedArgument;
                break;
            }
            continue;
        }

        // Any other well-formed frame (acknowledgements, string returns) is
        // simply discarded; this driver never asks the panel for data.
        length_ = 0;
    }

    const Command command = pending_;
    argument = pendingArgument_;
    pending_ = Command::None;
    pendingArgument_ = 0;
    return command;
}

}  // namespace jensen
