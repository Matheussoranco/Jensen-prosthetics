#include "jensen/hand_config.h"

namespace jensen {

const char* handSideName(HandSide side) {
    return side == HandSide::Left ? "left" : "right";
}

uint32_t crc32(const void* data, size_t length) {
    // Bitwise reflected CRC-32. No lookup table: this runs a handful of times
    // per boot, and 1 KB of table is worth more than the microseconds saved.
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            const uint32_t mask = static_cast<uint32_t>(-static_cast<int32_t>(crc & 1u));
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

namespace {

/// Placeholder servo endpoints.
///
/// 1200–1800 us is roughly the middle third of an MG996R's travel. It is
/// enough to see every digit move and to verify wiring and direction, and far
/// enough from the mechanical stops that a mis-wired channel pulls a tendon
/// instead of snapping a printed link. Real endpoints come from `cal digit`.
constexpr uint16_t kDefaultOpenUs = 1200;
constexpr uint16_t kDefaultClosedUs = 1800;
constexpr uint16_t kDefaultGuardLoUs = 1100;
constexpr uint16_t kDefaultGuardHiUs = 1900;

DigitCalibration defaultDigit(bool reversed, bool enabled) {
    DigitCalibration cal{};
    cal.openUs = reversed ? kDefaultClosedUs : kDefaultOpenUs;
    cal.closedUs = reversed ? kDefaultOpenUs : kDefaultClosedUs;
    cal.minUs = kDefaultGuardLoUs;
    cal.maxUs = kDefaultGuardHiUs;
    cal.enabled = enabled;
    return cal;
}

ForceCalibration defaultForce(bool present) {
    ForceCalibration cal{};
    cal.present = present;
    cal.zeroCounts = 60;
    cal.spanCounts = 2600;
    cal.referenceNewtons = 20.0f;
    cal.exponent = 0.85f;
    return cal;
}

constexpr uint16_t gripBit(GripId grip) {
    return static_cast<uint16_t>(1u << static_cast<uint8_t>(grip));
}

}  // namespace

HandConfig HandConfig::defaults(HandSide side) {
    HandConfig cfg{};
    cfg.magic = kMagic;
    cfg.version = kVersion;
    cfg.side = side;

    // Handedness only seeds the *initial guess* at travel direction. Unlike
    // HACKberry, nothing downstream of this struct knows or cares which hand
    // this is — direction lives in the endpoint ordering and is settled by
    // bench calibration.
    const bool mirrored = (side == HandSide::Left);

    cfg.digits[index(Digit::ThumbFlex)] = defaultDigit(mirrored, true);
    cfg.digits[index(Digit::ThumbRot)] = defaultDigit(!mirrored, true);
    cfg.digits[index(Digit::Index)] = defaultDigit(mirrored, true);
    cfg.digits[index(Digit::Middle)] = defaultDigit(mirrored, true);
    cfg.digits[index(Digit::Ring)] = defaultDigit(mirrored, true);
    cfg.digits[index(Digit::Pinky)] = defaultDigit(mirrored, true);
    // The wrist rotator is optional hardware; off until someone fits one.
    cfg.digits[index(Digit::Wrist)] = defaultDigit(false, false);

    cfg.digitSpeed[index(Digit::ThumbFlex)] = 2.0f;
    cfg.digitSpeed[index(Digit::ThumbRot)] = 1.2f;  // opposition moves more mass
    cfg.digitSpeed[index(Digit::Index)] = 2.2f;
    cfg.digitSpeed[index(Digit::Middle)] = 2.2f;
    cfg.digitSpeed[index(Digit::Ring)] = 2.2f;
    cfg.digitSpeed[index(Digit::Pinky)] = 2.2f;
    cfg.digitSpeed[index(Digit::Wrist)] = 0.8f;

    cfg.emg = EmgConfig{};
    cfg.drive = DriveConfig{};
    cfg.emgFlexor = EmgCalibration{};
    cfg.emgExtensor = EmgCalibration{};

    cfg.forceSensor = ForceSensorConfig{};
    cfg.gripForce = GripForceConfig{};
    // FSR400 pads are fitted to the three digits that carry a pinch: thumb,
    // index and middle. Ring and pinky follow the others in every pattern.
    cfg.force[index(Digit::ThumbFlex)] = defaultForce(true);
    cfg.force[index(Digit::ThumbRot)] = defaultForce(false);
    cfg.force[index(Digit::Index)] = defaultForce(true);
    cfg.force[index(Digit::Middle)] = defaultForce(true);
    cfg.force[index(Digit::Ring)] = defaultForce(false);
    cfg.force[index(Digit::Pinky)] = defaultForce(false);

    cfg.battery = BatteryConfig{};
    cfg.holdGuard = HoldGuardConfig{};

    cfg.enabledGrips = gripBit(GripId::Rest) | gripBit(GripId::Power) | gripBit(GripId::Pinch) |
                       gripBit(GripId::Tripod) | gripBit(GripId::LateralKey);
    cfg.defaultGrip = GripId::Power;
    cfg.hapticsEnabled = true;
    cfg.idleReleaseEnabled = true;
    cfg.idleTimeoutSeconds = 20.0f;

    // Precision grips act through a much smaller contact patch; the same
    // setpoint that is comfortable wrapped around a bottle will crush an egg
    // between two fingertips.
    cfg.gripForceScale[static_cast<uint8_t>(GripId::Rest)] = 0.50f;
    cfg.gripForceScale[static_cast<uint8_t>(GripId::Power)] = 1.00f;
    cfg.gripForceScale[static_cast<uint8_t>(GripId::Pinch)] = 0.45f;
    cfg.gripForceScale[static_cast<uint8_t>(GripId::Tripod)] = 0.60f;
    cfg.gripForceScale[static_cast<uint8_t>(GripId::LateralKey)] = 0.55f;
    cfg.gripForceScale[static_cast<uint8_t>(GripId::Point)] = 0.50f;
    cfg.gripForceScale[static_cast<uint8_t>(GripId::Hook)] = 1.00f;

    return cfg;
}

bool HandConfig::isValid() const {
    if (magic != kMagic) return false;
    if (version != kVersion) return false;
    if (static_cast<uint8_t>(defaultGrip) >= kGripCount) return false;
    for (uint8_t i = 0; i < kDigitCount; ++i) {
        if (digitSpeed[i] <= 0.0f || digitSpeed[i] > 20.0f) return false;
        // A disabled digit is allowed to hold nonsense endpoints; an enabled
        // one is not.
        if (digits[i].enabled && !digits[i].isValid()) return false;
    }
    if (idleTimeoutSeconds < 0.0f || idleTimeoutSeconds > 3600.0f) return false;
    return true;
}

bool HandConfig::digitsCalibrated() const {
    // The hand is usable once the thumb and all four fingers are calibrated;
    // thumb opposition and the wrist are allowed to be absent.
    const Digit required[] = {Digit::ThumbFlex, Digit::Index, Digit::Middle, Digit::Ring,
                              Digit::Pinky};
    for (Digit d : required) {
        const DigitCalibration& cal = digits[index(d)];
        if (!cal.enabled || !cal.isValid()) return false;
    }
    return true;
}

bool HandConfig::emgCalibrated() const {
    if (!emgFlexor.isUsable()) return false;
    if (drive.mode == DriveMode::DualChannel && !emgExtensor.isUsable()) return false;
    return true;
}

uint32_t HandConfig::checksum() const {
    return crc32(this, sizeof(HandConfig));
}

}  // namespace jensen
