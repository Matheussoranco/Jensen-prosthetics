#include "hal/config_store.h"

#include <Preferences.h>

#include <cstring>

namespace jensen {

namespace {
constexpr const char* kNamespace = "jensen";
constexpr const char* kSlotKey[2] = {"cfg0", "cfg1"};

/// Layout written by firmware configuration version 1. Keeping this exact
/// definition lets version 2 add current/pattern settings without discarding
/// a user's hard-won servo, force, and primary EMG calibration.
struct LegacyHandConfigV1 {
    uint32_t magic;
    uint16_t version;
    HandSide side;
    uint8_t reserved0;
    DigitCalibration digits[kDigitCount];
    float digitSpeed[kDigitCount];
    EmgConfig emg;
    DriveConfig drive;
    EmgCalibration emgFlexor;
    EmgCalibration emgExtensor;
    ForceSensorConfig forceSensor;
    GripForceConfig gripForce;
    ForceCalibration force[kGraspDigitCount];
    BatteryConfig battery;
    HoldGuardConfig holdGuard;
    uint16_t enabledGrips;
    GripId defaultGrip;
    bool hapticsEnabled;
    bool idleReleaseEnabled;
    float idleTimeoutSeconds;
    float gripForceScale[kGripCount];
};

struct LegacyRecordV1 {
    uint32_t crc;
    uint32_t sequence;
    LegacyHandConfigV1 config;
};

/// CRC covers the sequence number as well as the payload, so a torn write
/// that lands a new sequence next to an old body is rejected.
uint32_t recordChecksum(uint32_t sequence, const HandConfig& config) {
    uint32_t crc = crc32(&sequence, sizeof(sequence));
    // Chain the two regions by folding the first result into the second.
    const uint32_t bodyCrc = config.checksum();
    const uint32_t combined[2] = {crc, bodyCrc};
    return crc32(combined, sizeof(combined));
}

uint32_t legacyRecordChecksum(const LegacyRecordV1& record) {
    const uint32_t sequenceCrc = crc32(&record.sequence, sizeof(record.sequence));
    const uint32_t bodyCrc = crc32(&record.config, sizeof(record.config));
    const uint32_t combined[2] = {sequenceCrc, bodyCrc};
    return crc32(combined, sizeof(combined));
}

bool readLegacySlot(bool ready, uint8_t slot, LegacyRecordV1& out) {
    if (!ready || slot > 1) return false;
    Preferences preferences;
    if (!preferences.begin(kNamespace, true)) return false;
    const size_t length = preferences.getBytesLength(kSlotKey[slot]);
    if (length != sizeof(LegacyRecordV1)) {
        preferences.end();
        return false;
    }
    const size_t read = preferences.getBytes(kSlotKey[slot], &out, sizeof(out));
    preferences.end();
    if (read != sizeof(out) || out.crc != legacyRecordChecksum(out)) return false;
    if (out.config.magic != HandConfig::kMagic || out.config.version != 1) return false;
    if (static_cast<uint8_t>(out.config.defaultGrip) >= kGripCount) return false;
    for (uint8_t i = 0; i < kDigitCount; ++i) {
        if (out.config.digitSpeed[i] <= 0.0f || out.config.digitSpeed[i] > 20.0f) return false;
        if (out.config.digits[i].enabled && !out.config.digits[i].isValid()) return false;
    }
    return out.config.idleTimeoutSeconds >= 0.0f && out.config.idleTimeoutSeconds <= 3600.0f;
}

HandConfig migrate(const LegacyHandConfigV1& old) {
    HandConfig config = HandConfig::defaults(old.side);
    for (uint8_t i = 0; i < kDigitCount; ++i) {
        config.digits[i] = old.digits[i];
        config.digitSpeed[i] = old.digitSpeed[i];
    }
    config.emg = old.emg;
    config.drive = old.drive;
    config.emgFlexor = old.emgFlexor;
    config.emgExtensor = old.emgExtensor;
    config.forceSensor = old.forceSensor;
    config.gripForce = old.gripForce;
    for (uint8_t i = 0; i < kGraspDigitCount; ++i) config.force[i] = old.force[i];
    config.battery = old.battery;
    config.holdGuard = old.holdGuard;
    config.enabledGrips = old.enabledGrips;
    config.defaultGrip = old.defaultGrip;
    config.hapticsEnabled = old.hapticsEnabled;
    config.idleReleaseEnabled = old.idleReleaseEnabled;
    config.idleTimeoutSeconds = old.idleTimeoutSeconds;
    for (uint8_t i = 0; i < kGripCount; ++i) config.gripForceScale[i] = old.gripForceScale[i];
    return config;
}
}  // namespace

bool ConfigStore::begin() {
    Preferences preferences;
    // Probe read-write once: if NVS is not mounted, every later call would
    // fail silently and the hand would quietly forget its calibration on
    // every power cycle.
    ready_ = preferences.begin(kNamespace, false);
    if (ready_) preferences.end();
    return ready_;
}

bool ConfigStore::readSlot(uint8_t slot, Record& out) const {
    if (!ready_ || slot > 1) return false;

    Preferences preferences;
    if (!preferences.begin(kNamespace, true)) return false;

    const size_t length = preferences.getBytesLength(kSlotKey[slot]);
    if (length != sizeof(Record)) {
        preferences.end();
        return false;
    }
    const size_t read = preferences.getBytes(kSlotKey[slot], &out, sizeof(Record));
    preferences.end();
    if (read != sizeof(Record)) return false;

    if (out.crc != recordChecksum(out.sequence, out.config)) return false;
    return out.config.isValid();
}

bool ConfigStore::writeSlot(uint8_t slot, const Record& record) {
    if (!ready_ || slot > 1) return false;

    Preferences preferences;
    if (!preferences.begin(kNamespace, false)) return false;
    const size_t written = preferences.putBytes(kSlotKey[slot], &record, sizeof(Record));
    preferences.end();
    return written == sizeof(Record);
}

bool ConfigStore::load(HandConfig& out) {
    Record records[2];
    const bool ok[2] = {readSlot(0, records[0]), readSlot(1, records[1])};

    if (!ok[0] && !ok[1]) {
        LegacyRecordV1 legacy[2]{};
        const bool legacyOk[2] = {readLegacySlot(ready_, 0, legacy[0]),
                                  readLegacySlot(ready_, 1, legacy[1])};
        if (!legacyOk[0] && !legacyOk[1]) return false;
        uint8_t chosen = legacyOk[1] ? 1 : 0;
        if (legacyOk[0] && legacyOk[1]) {
            const int32_t delta = static_cast<int32_t>(legacy[1].sequence - legacy[0].sequence);
            chosen = delta > 0 ? 1 : 0;
        }
        out = migrate(legacy[chosen].config);
        if (!out.isValid()) return false;
        activeSlot_ = chosen;
        sequence_ = legacy[chosen].sequence;
        // Best effort: immediately rewrite into the other slot as v2. Even if
        // NVS is temporarily read-only, this boot can safely use the migrated
        // in-memory configuration and try again next time.
        save(out);
        return true;
    }

    uint8_t chosen = 0;
    if (ok[0] && ok[1]) {
        // Sequence numbers wrap; compare as a signed difference so the newer
        // record still wins across the wrap.
        const int32_t delta = static_cast<int32_t>(records[1].sequence - records[0].sequence);
        chosen = (delta > 0) ? 1 : 0;
    } else {
        chosen = ok[1] ? 1 : 0;
    }

    out = records[chosen].config;
    activeSlot_ = chosen;
    sequence_ = records[chosen].sequence;
    return true;
}

bool ConfigStore::save(const HandConfig& config) {
    if (!ready_) return false;
    if (!config.isValid()) return false;

    Record record{};
    record.sequence = sequence_ + 1;
    record.config = config;
    record.crc = recordChecksum(record.sequence, record.config);

    // Write to the slot that is *not* currently in use, so a failure here
    // leaves the previous calibration intact and loadable.
    const uint8_t target = activeSlot_ == 0 ? 1 : 0;
    if (!writeSlot(target, record)) return false;

    // Verify by reading it back before adopting it.
    Record verify{};
    if (!readSlot(target, verify)) return false;
    if (verify.crc != record.crc) return false;

    activeSlot_ = target;
    sequence_ = record.sequence;
    ++saveCount_;
    return true;
}

bool ConfigStore::erase() {
    if (!ready_) return false;
    Preferences preferences;
    if (!preferences.begin(kNamespace, false)) return false;
    const bool cleared = preferences.clear();
    preferences.end();
    if (cleared) {
        activeSlot_ = 0;
        sequence_ = 0;
    }
    return cleared;
}

}  // namespace jensen
