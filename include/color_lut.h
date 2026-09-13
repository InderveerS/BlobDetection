#pragma once
#include <stdint.h>
#include "color_types.h"

// ================================================================
// RGB565 -> colour classification lookup table.
//
// RGB565 is only 16 bits wide, so the ENTIRE colour space fits in a 64 KB
// table. That lets us replace the per-pixel HSV conversion (two integer
// divisions) with one byte load, and classify all four colours for the price
// of one. This is the CMVision approach, except that at 16 bits the table is
// exact rather than a quantised approximation.
//
// The table MUST live in internal SRAM. Lookups are effectively random access,
// and a PSRAM table would be slower than the arithmetic it replaces - so it is
// a static array rather than a heap allocation, which makes the placement
// guaranteed and turns "not enough RAM" into a link error instead of a
// competition-day surprise.
//
// Byte layout:
//   bits 0-2 : colour id + 1  (0 = no match, 1..COLOR_COUNT)
//   bit  7   : "core" flag - the pixel sits well inside the profile's range
//              rather than on its edge. A real object is mostly core pixels;
//              a marginal background match clusters at the range boundary, so
//              the core fraction is a free confidence signal.
// ================================================================
#define LUT_SIZE       65536
#define LUT_COLOR_MASK 0x07
#define LUT_CORE_BIT   0x80

static inline uint8_t lutColorOf(uint8_t entry) { return (uint8_t)(entry & LUT_COLOR_MASK); }
static inline bool    lutIsCore (uint8_t entry) { return (entry & LUT_CORE_BIT) != 0; }

// Pure builder: fills `lut` (LUT_SIZE bytes) from the four profiles. No
// Arduino, no globals - unit-testable on a PC.
//
// Where a pixel value satisfies more than one profile it is assigned to the
// nearest by circular hue distance, so the four colours come out mutually
// exclusive. That kills e.g. an orange pixel matching both red and yellow
// before detection ever runs.
void lutBuild(uint8_t *lut, const HsvRange *profiles);

// On-target wrappers around the static table.
void           lutRebuild();      // rebuild from the current active profiles
const uint8_t* lutData();
uint32_t       lutLastBuildMicros();

// Rebuilds the shared table from arbitrary profiles. Autotune uses this to
// temporarily load its search priors, then calls lutRebuild() to restore.
// Costs no extra RAM, and detection doesn't run while tuning anyway.
void lutBuildCustom(const HsvRange *profiles);

// Diagnostics: how many of the 65536 possible pixel values map to each colour.
// A colour claiming a huge slice of colour space is a profile that will pick
// up half the world - worth seeing after a tune.
void lutCoverage(uint32_t *countsOut /* [COLOR_COUNT] */);
