#include "camera_setup.h"
#include "camera_pins.h"
#include <Arduino.h>

// Redefined locally (rather than relying on a specific esp32-camera header
// path) so this compiles regardless of exact library version/layout.
#ifndef OV2640_PID
#define OV2640_PID 0x26
#endif
#ifndef OV3660_PID
#define OV3660_PID 0x3660
#endif

bool cameraInit() {
    camera_config_t cfg = {};
    cfg.ledc_channel = LEDC_CHANNEL_0;
    cfg.ledc_timer   = LEDC_TIMER_0;
    cfg.pin_d0       = CAM_D0;
    cfg.pin_d1       = CAM_D1;
    cfg.pin_d2       = CAM_D2;
    cfg.pin_d3       = CAM_D3;
    cfg.pin_d4       = CAM_D4;
    cfg.pin_d5       = CAM_D5;
    cfg.pin_d6       = CAM_D6;
    cfg.pin_d7       = CAM_D7;
    cfg.pin_xclk     = CAM_XCLK;
    cfg.pin_pclk     = CAM_PCLK;
    cfg.pin_vsync    = CAM_VSYNC;
    cfg.pin_href     = CAM_HREF;
    cfg.pin_sccb_sda = CAM_SIOD;
    cfg.pin_sccb_scl = CAM_SIOC;
    cfg.pin_pwdn     = CAM_PWDN;
    cfg.pin_reset    = CAM_RESET;
    cfg.xclk_freq_hz = 20000000;           // 20 MHz (try 10000000 if init fails)
    cfg.pixel_format = PIXFORMAT_RGB565;   // raw pixels for processing
    cfg.frame_size   = FRAMESIZE_QVGA;     // 320 × 240 — must match frame_config.h
    cfg.grab_mode    = CAMERA_GRAB_LATEST; // always get newest frame
    cfg.fb_location  = CAMERA_FB_IN_PSRAM;
    cfg.fb_count     = 2;

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        Serial.printf("[ERROR] Camera init failed: 0x%x\n", err);
        if (err == 0x20003)
            Serial.println("  -> Camera not found on SCCB bus. Check power (3.3V) and ribbon cable.");
        else if (err == 0x105)
            Serial.println("  -> Camera ID mismatch. Try xclk_freq_hz = 10000000.");
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_vflip(s, 1);         // set to 1 if image is upside down
        s->set_hmirror(s, 0);       // see cameraSetMirror() -- affects the sign of cx
        // Auto everything at boot so the sensor can converge on the venue
        // lighting. Freeze it with `lock` once you've tuned; see camera_setup.h.
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
        s->set_exposure_ctrl(s, 1);
        s->set_gain_ctrl(s, 1);
    }
    return true;
}

static CameraLockState g_lock = { false, 0, 0 };
static bool            g_mirror = false;

bool cameraApplyLock(uint16_t aecValue, uint8_t agcGain) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return false;

    // Order matters: disable the auto loops first, then write the values,
    // otherwise the loop overwrites what we just set.
    s->set_exposure_ctrl(s, 0);
    s->set_gain_ctrl(s, 0);
    s->set_whitebal(s, 0);
    s->set_awb_gain(s, 0);
    s->set_aec_value(s, aecValue);
    s->set_agc_gain(s, agcGain);

    g_lock.locked   = true;
    g_lock.aecValue = aecValue;
    g_lock.agcGain  = agcGain;
    return true;
}

bool cameraLock(uint32_t settleMs) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) { Serial.println("[LOCK] No sensor."); return false; }

    Serial.printf("[LOCK] Letting exposure settle for %u ms -- hold the scene steady...\n",
                  (unsigned)settleMs);
    // Keep pulling frames so the AEC/AGC loops actually run and converge.
    uint32_t until = millis() + settleMs;
    while ((int32_t)(millis() - until) < 0) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) esp_camera_fb_return(fb);
        delay(10);
    }

    // init_status re-reads the real sensor registers (GAIN, AEC) into
    // s->status. Without this the shadow copy would still hold boot defaults,
    // because the auto loops adjust the hardware directly.
    if (s->init_status) s->init_status(s);
    uint16_t aec = s->status.aec_value;
    uint8_t  agc = s->status.agc_gain;

    if (!cameraApplyLock(aec, agc)) return false;

    Serial.printf("[LOCK] Frozen: exposure=%u gain=%u, AWB off.\n", (unsigned)aec, (unsigned)agc);
    Serial.println("       Verify: wave a coloured card through frame -- a static object's");
    Serial.println("       reported hue must NOT shift. Retune colours if you relight the venue.");
    return true;
}

void cameraUnlock() {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return;
    s->set_exposure_ctrl(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    g_lock.locked = false;
    Serial.println("[LOCK] Auto exposure/gain/white-balance re-enabled.");
}

CameraLockState cameraGetLockState() { return g_lock; }

void cameraSetMirror(bool on) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return;
    s->set_hmirror(s, on ? 1 : 0);
    g_mirror = on;
}

bool cameraGetMirror() { return g_mirror; }

void cameraPrintStatus() {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) { Serial.println("No sensor."); return; }
    if (s->init_status) s->init_status(s);

    Serial.printf("Camera: %s   mirror:%s vflip:%u\n",
        (s->id.PID == OV3660_PID) ? "OV3660" : "OV2640",
        g_mirror ? "ON" : "off", s->status.vflip);
    Serial.printf("  exposure: %s (value %u)   gain: %s (value %u)\n",
        s->status.aec ? "AUTO" : "LOCKED", (unsigned)s->status.aec_value,
        s->status.agc ? "AUTO" : "LOCKED", (unsigned)s->status.agc_gain);
    Serial.printf("  white balance: %s   awb_gain: %s\n",
        s->status.awb ? "AUTO" : "LOCKED", s->status.awb_gain ? "AUTO" : "LOCKED");
    if (!g_lock.locked)
        Serial.println("  -> Not locked. Auto white balance will drift your tuned ranges.");
}

CameraId detectCameraId() {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return CAM_UNKNOWN;

    Serial.printf("[CAM] Sensor PID: 0x%04X\n", s->id.PID);
    if (s->id.PID == OV2640_PID) return CAM_OV2640;
    if (s->id.PID == OV3660_PID) return CAM_OV3660;
    Serial.println("[CAM] WARNING: unrecognized sensor PID, falling back to OV2640 defaults.");
    return CAM_UNKNOWN;
}