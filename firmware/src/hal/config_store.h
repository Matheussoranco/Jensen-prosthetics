// Jensen Hand — persistent configuration storage.
//
// The whole `HandConfig` is written as one CRC-protected blob into NVS. That
// choice matters: servo endpoints and electrode calibration have to be
// mutually consistent, and a key-per-field store can be left half-updated by a
// brownout during a save. Here a partial write fails its checksum and the hand
// falls back to defaults, which is loud and recoverable, instead of running on
// a mixture of two calibrations, which is neither.
//
// Two slots are kept. A save writes the new blob to the spare slot and only
// then updates which slot is current, so an interrupted save can never
// destroy the calibration that was already working.
#pragma once

#include <cstdint>

#include "jensen/hand_config.h"

namespace jensen {

class ConfigStore {
public:
    bool begin();

    /// Loads the newest valid slot. Returns false if neither slot holds a
    /// usable configuration, leaving `out` untouched.
    bool load(HandConfig& out);

    /// Writes to the inactive slot, then commits it as current.
    bool save(const HandConfig& config);

    /// Erases both slots. The next boot comes up on factory defaults.
    bool erase();

    /// Which slot was last read or written, for diagnostics.
    uint8_t activeSlot() const { return activeSlot_; }
    uint32_t saveCount() const { return saveCount_; }

private:
    struct Record {
        uint32_t crc;
        uint32_t sequence;
        HandConfig config;
    };

    bool readSlot(uint8_t slot, Record& out) const;
    bool writeSlot(uint8_t slot, const Record& record);

    bool ready_ = false;
    uint8_t activeSlot_ = 0;
    uint32_t sequence_ = 0;
    uint32_t saveCount_ = 0;
};

}  // namespace jensen
