#include "color_lut.h"
#include "hsv_convert.h"
#include <string.h>

// lutBuild() is pure and host-testable. Only the convenience wrapper that
// reads the live profile table and times itself needs Arduino, so that part is
// compiled out under the native test build.
#ifdef ARDUINO
  #include <Arduino.h>
  #include "profiles.h"
#endif

// Fraction of the range's half-width that still counts as "core", in percent.
// 60 means the middle 60% of the hue window is core and the outer 40% is edge.
#define CORE_HUE_PERCENT 60
// How far inside the S/V bounds a pixel must sit to count as core. Only applied
// to an upper bound if that bound is actually restrictive (< 255); profiles
// almost always use 255 to mean "no upper limit", and penalising bright,
// saturated pixels for being near it would be backwards.
#define CORE_SV_MARGIN   15

// The table itself. Static, not heap: guarantees internal SRAM placement.
static uint8_t  g_lut[LUT_SIZE];
static uint32_t g_buildMicros = 0;

// ---- circular hue helpers (hue is 0..179, OpenCV convention) ----

// Width of a possibly-wrapped range, in hue units.
static inline int hueSpan(const HsvRange &r) {
    return (r.hMin <= r.hMax) ? (r.hMax - r.hMin) : (180 - r.hMin + r.hMax);
}

static inline int hueCenter(const HsvRange &r) {
    int c = r.hMin + hueSpan(r) / 2;
    return c % 180;
}

// Shortest distance around the hue circle, 0..90.
static inline int hueDist(int a, int b) {
    int d = a - b;
    if (d < 0) d = -d;
    d %= 180;
    return (d > 90) ? (180 - d) : d;
}

static inline bool insideWithMargin(int val, int lo, int hi, int margin) {
    if (val < lo + margin)               return false;
    if (hi < 255 && val > hi - margin)   return false;
    return true;
}

void lutBuild(uint8_t *lut, const HsvRange *profiles) {
    // Precompute per-colour geometry so the 65536-iteration loop stays tight.
    int center[COLOR_COUNT], coreHalf[COLOR_COUNT];
    for (int c = 0; c < COLOR_COUNT; c++) {
        center[c]   = hueCenter(profiles[c]);
        // Half-width of the core zone. Guaranteed >= 1 so a very narrow profile
        // still produces some core pixels rather than none.
        int half    = hueSpan(profiles[c]) / 2;
        coreHalf[c] = (half * CORE_HUE_PERCENT) / 100;
        if (coreHalf[c] < 1) coreHalf[c] = 1;
    }

    for (uint32_t px = 0; px < LUT_SIZE; px++) {
        int h, s, v;
        rgb565ToHsv((uint16_t)px, h, s, v);

        // Pick the best-matching colour. Ties broken by hue proximity, which
        // makes the four classes mutually exclusive.
        int best = -1, bestDist = 1000;
        for (int c = 0; c < COLOR_COUNT; c++) {
            if (!hsvInRange(h, s, v, profiles[c])) continue;
            int d = hueDist(h, center[c]);
            if (d < bestDist) { bestDist = d; best = c; }
        }

        if (best < 0) { lut[px] = 0; continue; }

        uint8_t entry = (uint8_t)(best + 1);
        const HsvRange &r = profiles[best];
        if (bestDist <= coreHalf[best] &&
            insideWithMargin(s, r.sMin, r.sMax, CORE_SV_MARGIN) &&
            insideWithMargin(v, r.vMin, r.vMax, CORE_SV_MARGIN)) {
            entry |= LUT_CORE_BIT;
        }
        lut[px] = entry;
    }
}

#ifdef ARDUINO
void lutRebuild() {
    HsvRange profiles[COLOR_COUNT];
    for (int c = 0; c < COLOR_COUNT; c++) profiles[c] = getColorProfile((ColorId)c);

    uint32_t t0 = micros();
    lutBuild(g_lut, profiles);
    g_buildMicros = micros() - t0;
}
#endif

void lutBuildCustom(const HsvRange *profiles) { lutBuild(g_lut, profiles); }

const uint8_t* lutData()            { return g_lut; }
uint32_t       lutLastBuildMicros() { return g_buildMicros; }

void lutCoverage(uint32_t *countsOut) {
    for (int c = 0; c < COLOR_COUNT; c++) countsOut[c] = 0;
    for (uint32_t px = 0; px < LUT_SIZE; px++) {
        uint8_t c = lutColorOf(g_lut[px]);
        if (c) countsOut[c - 1]++;
    }
}
