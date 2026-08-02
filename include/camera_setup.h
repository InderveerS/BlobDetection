#pragma once
#include "esp_camera.h"
#include "color_types.h"

bool      cameraInit();        // returns false on failure
CameraId  detectCameraId();    // reads sensor PID register to tell OV2640 from OV3660

// ================================================================
// Exposure / white-balance locking.
//
// Auto white balance actively fights colour-based detection: it retunes the
// very thing the HSV profiles measure, so a range tuned under one light drifts
// under another. Auto exposure does the same to V.
//
// The workflow is: let the sensor settle under the venue lighting, tune the
// colours, then LOCK. After that the sensor stops chasing the scene and the
// saved ranges stay valid.
//
// Locking reads the converged AEC/AGC values back out of the sensor registers
// (via the driver's init_status) and re-applies them with auto disabled, so the
// exact exposure is preserved and can be persisted to NVS.
// ================================================================
struct CameraLockState {
    bool     locked;
    uint16_t aecValue;    // 0-1200
    uint8_t  agcGain;     // 0-30
};

// Waits settleMs for AEC/AGC to converge, then freezes them.
bool cameraLock(uint32_t settleMs = 1500);
void cameraUnlock();

// Re-applies a previously saved lock without waiting to converge. Used at boot
// when NVS holds a lock from a previous session.
bool cameraApplyLock(uint16_t aecValue, uint8_t agcGain);

CameraLockState cameraGetLockState();
void            cameraPrintStatus();

// Horizontal mirror. This matters more than it looks: if the image is mirrored,
// cx increases to the left, and any steering that keys off it is inverted.
void cameraSetMirror(bool on);
bool cameraGetMirror();
