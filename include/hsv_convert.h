#pragma once
#include <stdint.h>
#include "color_types.h"

// Deliberately no <Arduino.h>: this header is the whole colour-classification
// path, and keeping it free of Arduino lets the LUT builder and the blob
// scanner be compiled and unit-tested on a PC (see test/).
static inline uint8_t hsv_max3(uint8_t a, uint8_t b, uint8_t c) {
    uint8_t m = a > b ? a : b;
    return m > c ? m : c;
}
static inline uint8_t hsv_min3(uint8_t a, uint8_t b, uint8_t c) {
    uint8_t m = a < b ? a : b;
    return m < c ? m : c;
}

// ================================================================
// RGB565 → HSV  (integer only, no floats, no heap)
// Output: H 0–179 (OpenCV convention), S 0–255, V 0–255
//
// RGB565 bit layout: RRRRR GGGGGG BBBBB
// OV2640/OV3660 send big-endian over DVP, so buf[i]<<8 | buf[i+1]
// is correct for both sensors.
// ================================================================
inline void rgb565ToHsv(uint16_t px, int &h, int &s, int &v) {
    uint8_t r5 = (px >> 11) & 0x1F;
    uint8_t g6 = (px >>  5) & 0x3F;
    uint8_t b5 =  px        & 0x1F;

    // Bit replication, not a plain shift. A plain <<3 tops out at 248, so
    // pure white lands at (248,252,248) -> d=4, S=4, and cMax==g makes it
    // report H=120 (green). Replicating the high bits gives 31->255 and
    // 63->255, so neutrals correctly come out at S=0.
    uint8_t r = (uint8_t)((r5 << 3) | (r5 >> 2));   // 5-bit → 8-bit
    uint8_t g = (uint8_t)((g6 << 2) | (g6 >> 4));   // 6-bit → 8-bit
    uint8_t b = (uint8_t)((b5 << 3) | (b5 >> 2));   // 5-bit → 8-bit

    uint8_t cMax = hsv_max3(r, g, b);
    uint8_t cMin = hsv_min3(r, g, b);
    int     d    = cMax - cMin;

    v = cMax;
    if (cMax == 0) { s = 0; h = 0; return; }
    s = (d * 255) / cMax;
    if (d == 0)   { h = 0; return; }

    int hue;
    if      (cMax == r) { hue = 60 * ((int)g - (int)b) / d; if (hue < 0) hue += 360; }
    else if (cMax == g) { hue = 60 * ((int)b - (int)r) / d + 120; }
    else                { hue = 60 * ((int)r - (int)g) / d + 240; }
    h = hue / 2;   // 0–179
}

// Wraparound-aware range check. Handles red (hMin > hMax) transparently.
inline bool hsvInRange(int h, int s, int v, const HsvRange &r) {
    bool hOk = (r.hMin <= r.hMax) ? (h >= r.hMin && h <= r.hMax)
                                   : (h >= r.hMin || h <= r.hMax);
    return hOk && s >= r.sMin && s <= r.sMax && v >= r.vMin && v <= r.vMax;
}