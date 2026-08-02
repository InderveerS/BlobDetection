#pragma once
#include <stdint.h>
#include "color_types.h"
#include "blob_filter.h"
#include "detect_result.h"

// ================================================================
// Temporal confirmation - the strongest single false-positive filter here,
// and almost free.
//
// A candidate must be seen in M of the last N frames, within a position gate,
// before `found` goes true. The hysteresis is deliberately asymmetric:
//
//   fast to confirm  - a real target locks on in a few frames
//   slow to drop     - a target fully hidden behind a rock for a third of a
//                      second must not un-confirm and make the robot give up
//
// A one-frame flash of a matching colour (someone walking past, a reflection)
// never survives M-of-N. A teletubby briefly occluded always does.
//
// No <Arduino.h>: host-tested.
// ================================================================

void trackerSetDefaults();
void trackerReset();

// Feeds one frame of filtered candidates in and produces the public result.
// Call exactly once per frame - the M-of-N history advances on every call.
void trackerUpdate(const FilterResult &f, uint32_t frameId, DetectResult &out);

// ---- tunables ----
void setTrackerConfirm(int m, int n);   // default 3 of 5
void getTrackerConfirm(int &m, int &n);

// How far a candidate may be from the track's smoothed position and still
// count as the same object, in pixels.
void setTrackerGatePx(int px);          // default 60
int  getTrackerGatePx();

// Consecutive misses tolerated before a confirmed track is dropped.
void setTrackerMaxMiss(int frames);     // default 10
int  getTrackerMaxMiss();

// Centroid EMA weight, percent. 100 = no smoothing.
void setTrackerSmoothing(int alphaPct); // default 40
int  getTrackerSmoothing();
