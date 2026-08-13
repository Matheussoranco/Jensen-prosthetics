// Jensen Hand — Nextion 2.4" HMI driver.
//
// Nextion panels run their own UI; the firmware's job is to push values into
// named components and to interpret the touch events that come back. The panel
// project itself (the .HMI file) is not source-controlled here — see
// `firmware/docs/nextion_hmi.md` for the component IDs this driver expects, so
// a panel can be rebuilt from scratch in the Nextion Editor.
//
// The link is treated as untrusted and optional: the hand must work identically
// with the display unplugged, so nothing here can block, and every received
// frame is length-checked before it is interpreted.
#pragma once

#include <Arduino.h>

#include <cstdint>

#include "jensen/grip.h"
#include "jensen/hand_state.h"

namespace jensen {

class NextionDisplay {
public:
    /// Commands the panel can send back.
    enum class Command : uint8_t {
        None = 0,
        NextGrip,
        PreviousGrip,
        SelectGrip,   ///< `argument` is a GripId
        ToggleHold,
        TogglePark,
        Calibrate,
        Open,
        Close,
        RequestRefresh,
    };

    void begin(HardwareSerial& serial);

    /// Pushes telemetry to the panel. Rate-limited internally; call freely.
    void update(const HandTelemetry& telemetry, uint32_t nowMillis);

    /// Reads one pending command from the panel, if any.
    Command poll(uint8_t& argument);

    /// True once the panel has sent a well-formed frame. Until then the hand
    /// assumes there is no display fitted.
    bool connected() const { return connected_; }

private:
    void send(const char* command);
    void sendText(const char* component, const char* value);
    void sendValue(const char* component, int32_t value);
    void terminate();

    Command decodeTouch(uint8_t page, uint8_t component, uint8_t event, uint8_t& argument);

    HardwareSerial* serial_ = nullptr;
    bool connected_ = false;

    uint8_t buffer_[16] = {};
    uint8_t length_ = 0;

    Command pending_ = Command::None;
    uint8_t pendingArgument_ = 0;

    uint32_t lastRefreshMillis_ = 0;
    /// Last values pushed, so unchanged fields are not re-sent at 5 Hz over a
    /// 9600 baud link that also has to carry touch events.
    ControlMode lastMode_ = ControlMode::Boot;
    GripId lastGrip_ = GripId::Count;
    int16_t lastAperture_ = -1;
    int16_t lastCharge_ = -1;
    int16_t lastForce_ = -1;
    uint16_t lastFaults_ = 0xFFFF;
    const char* lastCalibrationStep_ = nullptr;
};

}  // namespace jensen
