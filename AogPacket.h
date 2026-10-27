#pragma once
#include <stdint.h>
#include <stddef.h>
inline bool validAogPacket(const uint8_t* p, size_t n) {
    if (n < 6 || p[0] != 0x80 || p[1] != 0x81 || p[2] != 0x7F || n != size_t(p[4]) + 6) return false;
    uint8_t sum = 0;
    for (size_t i = 2; i < n - 1; ++i) sum += p[i];
    if (sum != p[n - 1]) return false;
    switch (p[3]) {
    case 0xFE: case 0xFC: case 0xFB: return n == 14;
    case 233: return n >= 11;
    case 201: return n == 11;
    case 202: return n == 9;
    default: return true;
    }
}

// PGN 254: signed little-endian hundredths of a degree. Keep fractions.
inline float decodeSteerDegrees(uint8_t low, uint8_t high) {
    return static_cast<int16_t>(uint16_t(low) | (uint16_t(high) << 8)) * 0.01f;
}
