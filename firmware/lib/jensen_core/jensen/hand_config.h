// Jensen Hand — the complete persisted configuration.
//
// Everything that has to survive a power cycle lives in this one trivially
// copyable struct: servo endpoints, electrode calibration, force calibration,
// tuning constants and user preferences. The storage layer writes it as a
// single CRC-protected blob, so a partial write can be detected and rejected
// rather than leaving the hand running on half-updated servo endpoints.
#pragma once

#include <cstddef>
#include <cstdint>

#include "jensen/current.h"
#include "jensen/digit.h"
#include "jensen/emg.h"
#include "jensen/force.h"
#include "jensen/grip.h"
#include "jensen/pattern.h"
#include "jensen/safety.h"

namespace jensen {

enum class HandSide : uint8_t { Right = 0, Left = 1 };

const char* handSideName(HandSide side);

/// CRC-32 (IEEE 802.3, reflected) over an arbitrary buffer.
uint32_t crc32(const void* data, size_t length);

struct HandConfig {
    static constexpr uint32_t kMagic = 0x4A454E53u;  // 'JENS'
    static constexpr uint16_t kVersion = 2;

    uint32_t magic = kMagic;
    uint16_t version = kVersion;
    HandSide side = HandSide::Right;
    uint8_t reserved0 = 0;

    // --- Actuation -----------------------------------------------------------
    DigitCalibration digits[kDigitCount]{};
    /// Full-travel rate per digit, in closure units per second.
    float digitSpeed[kDigitCount]{};

    // --- Myoelectric input ---------------------------------------------------
    EmgConfig emg{};
    DriveConfig drive{};
    EmgCalibration emgFlexor{};
    EmgCalibration emgExtensor{};
    EmgCalibration emgAux[kPatternSiteCount - 2]{};
    PatternConfig pattern{};

    // --- Force sensing -------------------------------------------------------
    ForceSensorConfig forceSensor{};
    GripForceConfig gripForce{};
    ForceCalibration force[kGraspDigitCount]{};

    // --- Supervision ---------------------------------------------------------
    BatteryConfig battery{};
    HoldGuardConfig holdGuard{};
    MotorCurrentConfig motorCurrent{};

    // --- Preferences ---------------------------------------------------------
    uint16_t enabledGrips = 0;
    GripId defaultGrip = GripId::Power;
    bool hapticsEnabled = true;
    bool idleReleaseEnabled = true;
    /// Seconds without myoelectric activity before relaxing the outputs.
    float idleTimeoutSeconds = 20.0f;
    /// Per-grip scaling of the force setpoint, indexed by GripId. A pinch is
    /// gentler than a power grasp on the same setpoint.
    float gripForceScale[kGripCount]{};

    /// Factory defaults for a given handedness.
    ///
    /// The servo endpoints these produce are deliberately narrow — they move
    /// the hand, but they do not reach the mechanical stops. Every build needs
    /// `cal digit` run against its own tendon tension before the full range is
    /// available. See `firmware/README.md`.
    static HandConfig defaults(HandSide side);

    /// Structural sanity check. Does *not* assert that the hand has been
    /// calibrated — see `digitsCalibrated()` and `emgCalibrated()`.
    bool isValid() const;

    bool digitsCalibrated() const;
    bool emgCalibrated() const;

    /// CRC over every byte of the struct. Used by the storage layer.
    uint32_t checksum() const;
};

static_assert(sizeof(HandConfig) < 1024, "HandConfig must stay small enough for a single NVS blob");

}  // namespace jensen
