#pragma once

#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdlib>

namespace jensen {

inline bool parseFiniteFloat(const char* text, float& out) {
    if (!text || !*text || std::isspace(static_cast<unsigned char>(*text))) return false;
    errno = 0;
    char* end = nullptr;
    const float value = std::strtof(text, &end);
    if (end == text || *end != '\0' || errno == ERANGE || !std::isfinite(value)) return false;
    out = value;
    return true;
}

inline bool parseBoundedLong(const char* text, long low, long high, long& out, int base = 10) {
    if (!text || !*text || std::isspace(static_cast<unsigned char>(*text))) return false;
    errno = 0;
    char* end = nullptr;
    const long value = std::strtol(text, &end, base);
    if (end == text || *end != '\0' || errno == ERANGE || value < low || value > high) return false;
    out = value;
    return true;
}

}  // namespace jensen
