#include "hal/config_store.h"

#include <Preferences.h>

#include <cstring>

namespace jensen {

namespace {
constexpr const char* kNamespace = "jensen";
constexpr const char* kSlotKey[2] = {"cfg0", "cfg1"};

/// CRC covers the sequence number as well as the payload, so a torn write
/// that lands a new sequence next to an old body is rejected.
uint32_t recordChecksum(uint32_t sequence, const HandConfig& config) {
    uint32_t crc = crc32(&sequence, sizeof(sequence));
    // Chain the two regions by folding the first result into the second.
    const uint32_t bodyCrc = config.checksum();
    const uint32_t combined[2] = {crc, bodyCrc};
    return crc32(combined, sizeof(combined));
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

    if (!ok[0] && !ok[1]) return false;

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
