#pragma once
#include <stdint.h>
#include "color_types.h"

// ================================================================
// Single-pass connected-component analysis over an RGB565 frame.
//
// Replaces the old detectBlob(), which averaged every matching pixel in the
// frame into one centroid - so two objects, or one object plus a background
// wash, produced a centroid pointing at the empty space between them.
//
// Algorithm: per row, run-length encode the LUT-classified pixels; link each
// run to overlapping same-colour runs in the row above via union-find. One
// raster pass, all four colours at once. Runs are the unit of work, so the
// per-pixel cost is a LUT load and a compare.
//
// Deliberately depends on neither <Arduino.h> nor <esp_camera.h> - it takes a
// raw buffer and a raw LUT - so it compiles and runs under test/ on a PC.
// ================================================================

// Inclusive region of interest. Restricting the scan is the cheapest and most
// effective defence against same-coloured background objects: a teletubby is
// on the course, a person's torso is higher in frame.
struct ScanRoi {
    int16_t x0, y0, x1, y1;
};

// A raw connected component, before any filtering.
struct BlobComponent {
    uint8_t  color;        // ColorId
    uint32_t pixels;
    uint32_t corePixels;   // subset well inside the profile - confidence signal
    int16_t  x0, y0, x1, y1;
    int16_t  cx, cy;       // centroid
};

// Upper bound on components returned from one scan. With the minimum-run
// filter active, real scenes produce tens; hitting this cap means the frame is
// pathological and blobScan reports it rather than silently dropping blobs.
#define MAX_COMPONENTS   1024
#define MAX_RUNS_PER_ROW 180

// Any of the three cap flags means blobs were dropped. They should never fire
// in a real scene; if one does, the frame is pathological (or minRunLength is
// too low) and you want to know rather than wonder why a target vanished.
struct BlobScanStats {
    uint16_t componentCount;
    uint32_t runsTotal;
    uint32_t scanMicros;
    bool     labelCapHit;   // ran out of labels
    bool     runCapHit;     // a row had more runs than MAX_RUNS_PER_ROW
    bool     outCapHit;     // more components than the caller's array could hold
};

// Minimum horizontal run length, in pixels. Acts as a free horizontal erosion:
// salt-and-pepper noise never becomes a component. 1 disables it.
void setMinRunLength(int px);
int  getMinRunLength();

// Scans `frame` (FRAME_W x FRAME_H, RGB565, big-endian per frame_config.h)
// using `lut` (LUT_SIZE bytes from color_lut.h). Writes up to `maxOut`
// components and returns how many were written. `stats` may be null.
int blobScan(const uint8_t *frame, const uint8_t *lut, const ScanRoi &roi,
             BlobComponent *out, int maxOut, BlobScanStats *stats);

// Convenience: the full frame.
ScanRoi fullFrameRoi();

// Shared output buffer, MAX_COMPONENTS long. Detection and tuning never run in
// the same frame, so they share this rather than each carrying a 24 KB copy.
// Tests pass their own arrays instead.
BlobComponent* blobScratch();
