#pragma once
#include <stdint.h>
#include "color_types.h"

// ================================================================
// The stable public output of the vision pipeline.
//
// This is the seam: Serial debug formats it, and the UART link to the other
// MCU serialises it. Nothing downstream should reach past this into the
// detector's internals.
// ================================================================

struct ColorDetection {
    bool     found;        // tracker-confirmed, not merely seen this frame
    uint8_t  confidence;   // 0-255
    int16_t  cx, cy;       // smoothed centroid
    int16_t  x0, y0, x1, y1;
    uint16_t pixels;
    uint8_t  fillPct;
    uint8_t  corePct;
    uint8_t  fragments;    // >1 means it was reassembled from occluded pieces
    bool     touchesEdge;  // partially out of frame
};

struct DetectResult {
    uint32_t       frameId;      // lets the receiver detect a stalled pipeline
    ColorDetection color[COLOR_COUNT];
    uint8_t        bestColor;    // highest-confidence confirmed colour, or NO_COLOR
};

#define NO_COLOR 0xFF

// The most recent result, defined in main.cpp. Read-only, and only safe to
// read from the vision core (core 1) - the link task gets its own copy through
// the mailbox rather than reaching in here.
const DetectResult& latestResult();
