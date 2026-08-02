#pragma once
#include <stdint.h>
#include "color_types.h"
#include "blob_detect.h"

// ================================================================
// Turns raw connected components into at most one trusted candidate per
// colour, and is where false positives get killed.
//
// The realistic false positive is a same-coloured object in the background -
// a person in a matching t-shirt - not sensor noise. Defences, in order of how
// much work they actually do:
//
//   1. ROI          - a teletubby is on the course; a standing person's torso
//                     is higher in frame. Costs nothing (the scan just starts
//                     at a later row) and removes most of the problem.
//   2. Area band    - at a known approach distance a target subtends a
//                     predictable pixel area. A person metres back, or one
//                     close up, does not.
//   3. Fill ratio   - pixels/bboxArea. A real object is dense; a lit-wall wash
//                     is diffuse. Weaker against a t-shirt, which is also
//                     dense, hence its position in this list.
//   4. Core fraction- share of pixels sitting deep inside the profile rather
//                     than on its edge (the LUT's core bit). A marginal
//                     background match clusters at the boundary.
//
// Temporal M-of-N confirmation on top of this lives in tracker.h.
//
// No <Arduino.h> here either - this is unit-tested on the host.
// ================================================================

enum RejectReason {
    REJ_NONE = 0,
    REJ_MIN_AREA,
    REJ_MAX_AREA,
    REJ_FILL,
    REJ_ASPECT,
    REJ_CORE,
    REJ_REASON_COUNT
};

const char* rejectReasonName(int r);

// Per-colour geometry limits. Defaults are ballpark; they want setting at the
// venue against the real course, which is what the `limits` serial command and
// `fieldcheck` are for.
struct ColorLimits {
    uint32_t minArea, maxArea;      // pixels
    uint8_t  minFillPct;            // 100*pixels/bboxArea
    uint16_t minAspectX100;         // 100*width/height
    uint16_t maxAspectX100;
    uint8_t  minCorePct;            // 100*corePixels/pixels
};

struct ColorCandidate {
    bool     valid;
    uint8_t  confidence;      // 0-255
    int16_t  cx, cy;
    int16_t  x0, y0, x1, y1;
    uint32_t pixels;
    uint8_t  fillPct;
    uint8_t  corePct;
    uint8_t  fragments;       // components merged into this candidate
    bool     touchesEdge;     // partially out of frame - penalised, not rejected
};

struct FilterResult {
    ColorCandidate color[COLOR_COUNT];
    uint16_t       rejected[REJ_REASON_COUNT];   // counts, for fieldcheck
    // Size of the BIGGEST blob thrown out for each reason. This is the number
    // you actually need when a gate is too strict: "too-small, largest was
    // 87px" tells you exactly where to put minArea, which the bare count does
    // not.
    uint32_t       rejectedMaxPx[REJ_REASON_COUNT];
    uint16_t       mergedAway;                   // fragments absorbed by merging
};

// ---- tunables ----
void         filterSetDefaults();
ColorLimits& colorLimits(ColorId c);

void    setScanRoi(const ScanRoi &r);
ScanRoi getScanRoi();

// Max bounding-box gap, in pixels, across which same-colour fragments are
// fused. Occlusion behind a rock splits a target into pieces; this puts it
// back together. 0 disables merging entirely.
void setMergeGap(int px);
int  getMergeGap();

// Fragments smaller than this are dropped before merging. Keeps the O(n^2)
// merge cheap and stops noise specks from dragging a bounding box outwards.
void setMinFragmentPx(int px);
int  getMinFragmentPx();

// Runs the whole filter. `comps` is modified in place (fragments are merged).
void blobFilter(BlobComponent *comps, int n, FilterResult &out);
