#pragma once
#include "color_types.h"
// ================================================================
// DEFAULT CALIBRATION PROFILES — one table per camera.
//
// These are just starting points. The real workflow is:
//   1. tune <color>      (live preview, see README)
//   2. save
//   3. dump               -> prints a block like the ones below
//   4. paste that block in here, replacing the matching table,
//      so the new values are baked in for next time you flash.
//
// Until you've tuned for real, these are rough ballpark guesses.
// ================================================================

// ---- OV2640 (wide angle) ----
constexpr HsvRange DEFAULT_PROFILES_OV2640[COLOR_COUNT] = {
    /* RED    */ {165,  10,  120, 255,   70, 255},   // wraps (hMin > hMax)
    /* GREEN  */ { 45,  80,   80, 255,   60, 255},
    /* PURPLE */ {130, 160,   80, 255,   50, 255},
    /* YELLOW */ { 20,  35,  110, 255,  100, 255},
};

// ---- OV3660 (narrow lens) ----
// Different sensor = different color response, so don't assume
// these should match OV2640 once you've actually tuned both.
constexpr HsvRange DEFAULT_PROFILES_OV3660[COLOR_COUNT] = {
    /* RED    */ {165,  10,  120, 255,   70, 255},
    /* GREEN  */ { 45,  80,   80, 255,   60, 255},
    /* PURPLE */ {130, 160,   80, 255,   50, 255},
    /* YELLOW */ { 20,  35,  110, 255,  100, 255},
};