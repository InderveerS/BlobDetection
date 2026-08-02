#pragma once
// ================================================================
// Shared types. Everything else (profiles, blob_detect, autotune,
// serial_commands) builds on these.
// ================================================================

// Add a 5th color later by inserting it before COLOR_COUNT and adding
// matching entries in profiles_defaults.h and the COLOR_PRIORS table
// in autotune.cpp.
enum ColorId { COLOR_RED = 0, COLOR_GREEN, COLOR_PURPLE, COLOR_YELLOW, COLOR_COUNT };

enum CameraId { CAM_OV2640 = 0, CAM_OV3660, CAM_UNKNOWN };

struct HsvRange {
    // hMin <= hMax  -> normal range [hMin, hMax]
    // hMin >  hMax  -> wraps through 0 (h >= hMin || h <= hMax) — this is how
    //                  red is represented now; no more "red"/"red2" split.
    int hMin, hMax;
    int sMin, sMax;
    int vMin, vMax;
};

// Detection output lives in blob_detect.h (BlobComponent) now. The old `Blob`
// struct went away with the frame-wide-average detector it belonged to.