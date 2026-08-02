#pragma once
#include <stdint.h>

// ================================================================
// Debug output verbosity.
//
// Printing is not free: ~90 chars at 115200 baud is ~7.8 ms of blocking
// transmit, which at 30 fps is a quarter of the frame budget. The old code
// printed every frame unconditionally, including when nothing was found.
//
// Defaults to VERB_SILENT on purpose - the competition-safe setting is what
// you get if nobody remembers to change it.
//
// Interactive output (tune previews, command responses, errors) ignores this
// and always prints; only the per-frame detection stream is gated.
// ================================================================
enum Verbosity {
    VERB_SILENT  = 0,   // no per-frame output at all
    VERB_SUMMARY = 1,   // throttled to ~10 Hz
    VERB_FRAME   = 2,   // every frame
};

void        setVerbosity(int v);
int         getVerbosity();
const char* verbosityName(int v);

// True if a per-frame detection line should be printed this iteration.
// Handles the VERB_SUMMARY throttle internally, so callers just do:
//     if (shouldPrintDetection()) Serial.printf(...);
bool shouldPrintDetection();
