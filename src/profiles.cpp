#include "profiles.h"
#include "profiles_defaults.h"
#include "color_lut.h"
#include <Arduino.h>

static CameraId g_camId = CAM_UNKNOWN;
static HsvRange g_active[COLOR_COUNT];
static ColorId  g_activeDetectColor = COLOR_YELLOW;  // arbitrary boot default
static int      g_minPx = 300;

const char* colorName(ColorId c) {
    switch (c) {
        case COLOR_RED:    return "red";
        case COLOR_GREEN:  return "green";
        case COLOR_PURPLE: return "purple";
        case COLOR_YELLOW: return "yellow";
        default:           return "?";
    }
}

const char* cameraName(CameraId id) {
    switch (id) {
        case CAM_OV2640: return "OV2640(wide)";
        case CAM_OV3660: return "OV3660(narrow)";
        default:         return "UNKNOWN";
    }
}

static const HsvRange* defaultsForCamera(CameraId id) {
    if (id == CAM_OV3660) return DEFAULT_PROFILES_OV3660;
    return DEFAULT_PROFILES_OV2640;  // fallback for CAM_UNKNOWN too
}

void profilesInit(CameraId detectedCam) {
    g_camId = detectedCam;
    resetAllToDefaults();
}

CameraId getActiveCameraId() { return g_camId; }

HsvRange getColorProfile(ColorId c) { return g_active[c]; }

// Every mutation rebuilds the LUT. Doing it here rather than at the call sites
// means "I changed a profile but forgot to rebuild" - which would show up as
// detection silently ignoring your edit - simply cannot happen.
void setColorProfile(ColorId c, const HsvRange &r) {
    g_active[c] = r;
    lutRebuild();
}

void    setActiveDetectColor(ColorId c) { g_activeDetectColor = c; }
ColorId getActiveDetectColor() { return g_activeDetectColor; }

void resetColorToDefault(ColorId c) {
    g_active[c] = defaultsForCamera(g_camId)[c];
    lutRebuild();
}

void resetAllToDefaults() {
    const HsvRange *def = defaultsForCamera(g_camId);
    for (int i = 0; i < COLOR_COUNT; i++) g_active[i] = def[i];
    lutRebuild();
}

int  getMinBlobPx() { return g_minPx; }
void setMinBlobPx(int px) { g_minPx = px; }

void printRange(const HsvRange &r) {
    Serial.printf("H:%d-%d%s  S:%d-%d  V:%d-%d\n",
        r.hMin, r.hMax, (r.hMin > r.hMax) ? "(wrap)" : "",
        r.sMin, r.sMax, r.vMin, r.vMax);
}

void dumpProfiles() {
    const char* tableName = (g_camId == CAM_OV3660) ? "OV3660" : "OV2640";
    static const char* names[COLOR_COUNT] = {"RED   ", "GREEN ", "PURPLE", "YELLOW"};

    Serial.printf("\n// ---- %s ----  (paste over the matching block in profiles_defaults.h)\n",
                  cameraName(g_camId));
    Serial.printf("static const HsvRange DEFAULT_PROFILES_%s[COLOR_COUNT] = {\n", tableName);
    for (int i = 0; i < COLOR_COUNT; i++) {
        HsvRange &r = g_active[i];
        Serial.printf("    /* %s */ {%4d, %4d,   %4d, %4d,   %4d, %4d},\n",
            names[i], r.hMin, r.hMax, r.sMin, r.sMax, r.vMin, r.vMax);
    }
    Serial.println("};\n");
}