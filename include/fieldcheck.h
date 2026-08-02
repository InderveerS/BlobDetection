#pragma once
#include <stdint.h>
#include "detect_result.h"
#include "blob_filter.h"
#include "blob_detect.h"

// ================================================================
// Venue setup tool: run detection for N seconds and report what actually
// happened, per colour.
//
// The point is the rejection histogram. Pointing this at an empty course, and
// then at a teammate wearing a matching-colour shirt, tells you WHICH gate is
// doing the work - or that nothing is, and you're only getting away with it by
// luck. That is not something you can see from the live detection stream.
//
// Typical use at the venue:
//   fieldcheck 10     with the teletubby at competition distance -> want a high
//                     detection rate and steady confidence
//   fieldcheck 10     with the course empty -> want ZERO confirmed detections
//   fieldcheck 10     with a same-coloured distraction in the background
// ================================================================

void fieldcheckStart(int seconds);
bool fieldcheckActive();
void fieldcheckSample(const DetectResult &r, const FilterResult &f, const BlobScanStats &st);
void fieldcheckAbort();
