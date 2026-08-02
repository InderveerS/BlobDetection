#include "vision_output.h"
#include <Arduino.h>

#define SUMMARY_INTERVAL_MS 100   // ~10 Hz at VERB_SUMMARY

static int      g_verbosity   = VERB_SILENT;
static uint32_t g_lastPrintMs = 0;

void setVerbosity(int v) {
    if (v < VERB_SILENT) v = VERB_SILENT;
    if (v > VERB_FRAME)  v = VERB_FRAME;
    g_verbosity = v;
}

int getVerbosity() { return g_verbosity; }

const char* verbosityName(int v) {
    switch (v) {
        case VERB_SILENT:  return "silent (competition default)";
        case VERB_SUMMARY: return "summary ~10Hz";
        case VERB_FRAME:   return "every frame";
        default:           return "?";
    }
}

bool shouldPrintDetection() {
    if (g_verbosity == VERB_SILENT) return false;
    if (g_verbosity == VERB_FRAME)  return true;

    uint32_t now = millis();
    if (now - g_lastPrintMs < SUMMARY_INTERVAL_MS) return false;
    g_lastPrintMs = now;
    return true;
}
