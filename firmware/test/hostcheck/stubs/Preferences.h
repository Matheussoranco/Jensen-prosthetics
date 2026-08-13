// Compile-only stub — see Arduino.h in this directory.
#pragma once

#include <cstddef>
#include <cstdint>

class Preferences {
public:
    bool begin(const char* name, bool readOnly = false);
    void end();
    bool clear();
    bool remove(const char* key);

    size_t putBytes(const char* key, const void* value, size_t length);
    size_t getBytes(const char* key, void* buffer, size_t maxLength);
    size_t getBytesLength(const char* key);
};
