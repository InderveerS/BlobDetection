#include "vision_nvs.h"
#include "profiles.h"
#include "blob_detect.h"
#include "blob_filter.h"
#include "tracker.h"
#include "camera_setup.h"
#include <Arduino.h>
#include <Preferences.h>

// Bump if the layout of StoredSettings changes; a mismatch is treated as
// "nothing stored" rather than loading garbage into the detector.
#define NVS_MAGIC   0x54423203UL   // 'TB2' + layout revision
#define NVS_NS      "vision"

struct StoredSettings {
    uint32_t magic;

    HsvRange    profiles[COLOR_COUNT];
    ColorLimits limits[COLOR_COUNT];

    int16_t roiX0, roiY0, roiX1, roiY1;
    int16_t mergeGap, minFragmentPx, minRunLength;

    int16_t confirmM, confirmN, gatePx, maxMiss, smoothing;

    uint8_t  cameraLocked;
    uint16_t aecValue;
    uint8_t  agcGain;
    uint8_t  mirror;
};

// One blob per camera, so a single flash image works on both boards.
static const char* keyForCamera() {
    return (getActiveCameraId() == CAM_OV3660) ? "ov3660" : "ov2640";
}

static void gather(StoredSettings &s) {
    s.magic = NVS_MAGIC;
    for (int c = 0; c < COLOR_COUNT; c++) {
        s.profiles[c] = getColorProfile((ColorId)c);
        s.limits[c]   = colorLimits((ColorId)c);
    }
    ScanRoi r = getScanRoi();
    s.roiX0 = r.x0; s.roiY0 = r.y0; s.roiX1 = r.x1; s.roiY1 = r.y1;

    s.mergeGap      = (int16_t)getMergeGap();
    s.minFragmentPx = (int16_t)getMinFragmentPx();
    s.minRunLength  = (int16_t)getMinRunLength();

    int m, n;
    getTrackerConfirm(m, n);
    s.confirmM  = (int16_t)m;
    s.confirmN  = (int16_t)n;
    s.gatePx    = (int16_t)getTrackerGatePx();
    s.maxMiss   = (int16_t)getTrackerMaxMiss();
    s.smoothing = (int16_t)getTrackerSmoothing();

    CameraLockState lk = cameraGetLockState();
    s.cameraLocked = lk.locked ? 1 : 0;
    s.aecValue     = lk.aecValue;
    s.agcGain      = lk.agcGain;
    s.mirror       = cameraGetMirror() ? 1 : 0;
}

static void apply(const StoredSettings &s) {
    for (int c = 0; c < COLOR_COUNT; c++) {
        colorLimits((ColorId)c) = s.limits[c];
        // setColorProfile rebuilds the LUT, so do it last per colour and let
        // the final call leave the table consistent.
        setColorProfile((ColorId)c, s.profiles[c]);
    }
    ScanRoi r = { s.roiX0, s.roiY0, s.roiX1, s.roiY1 };
    setScanRoi(r);

    setMergeGap(s.mergeGap);
    setMinFragmentPx(s.minFragmentPx);
    setMinRunLength(s.minRunLength);

    setTrackerConfirm(s.confirmM, s.confirmN);
    setTrackerGatePx(s.gatePx);
    setTrackerMaxMiss(s.maxMiss);
    setTrackerSmoothing(s.smoothing);

    cameraSetMirror(s.mirror != 0);
    if (s.cameraLocked) cameraApplyLock(s.aecValue, s.agcGain);
}

bool visionNvsSave() {
    StoredSettings s;
    gather(s);

    Preferences p;
    if (!p.begin(NVS_NS, false)) {
        Serial.println("[NVS] Could not open storage for writing.");
        return false;
    }
    size_t written = p.putBytes(keyForCamera(), &s, sizeof(s));
    p.end();

    if (written != sizeof(s)) {
        Serial.printf("[NVS] Write failed (%u of %u bytes).\n",
                      (unsigned)written, (unsigned)sizeof(s));
        return false;
    }
    Serial.printf("[NVS] Saved %u bytes for %s. These now load automatically at boot.\n",
                  (unsigned)written, cameraName(getActiveCameraId()));
    return true;
}

bool visionNvsLoad() {
    Preferences p;
    if (!p.begin(NVS_NS, true)) return false;

    StoredSettings s;
    size_t read = p.getBytes(keyForCamera(), &s, sizeof(s));
    p.end();

    if (read != sizeof(s)) return false;
    if (s.magic != NVS_MAGIC) {
        Serial.println("[NVS] Stored settings are from an older layout -- ignoring them.");
        return false;
    }
    apply(s);
    return true;
}

bool visionNvsClear() {
    Preferences p;
    if (!p.begin(NVS_NS, false)) return false;
    bool ok = p.remove(keyForCamera());
    p.end();
    Serial.printf("[NVS] %s stored settings for %s. Compiled-in defaults apply on next boot.\n",
                  ok ? "Cleared" : "No", cameraName(getActiveCameraId()));
    return ok;
}

void visionNvsStatus() {
    Preferences p;
    if (!p.begin(NVS_NS, true)) { Serial.println("[NVS] Storage unavailable."); return; }
    size_t sz = p.getBytesLength(keyForCamera());
    p.end();

    if (sz == sizeof(StoredSettings))
        Serial.printf("[NVS] Settings stored for %s (%u bytes), loaded at boot.\n",
                      cameraName(getActiveCameraId()), (unsigned)sz);
    else if (sz == 0)
        Serial.printf("[NVS] Nothing stored for %s -- running compiled-in defaults.\n",
                      cameraName(getActiveCameraId()));
    else
        Serial.printf("[NVS] Stored blob for %s is the wrong size (%u vs %u) -- ignored.\n",
                      cameraName(getActiveCameraId()), (unsigned)sz, (unsigned)sizeof(StoredSettings));
}
