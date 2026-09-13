#pragma once

// ================================================================
// Persistence to flash (NVS).
//
// Without this the only way to keep a tuned value is `dump`, paste into
// profiles_defaults.h, recompile and reflash. At the venue with the robot
// assembled that is the difference between a 30-second retune and an hour.
//
// Settings are keyed per detected sensor, so a board with an OV2640 and one
// with an OV3660 can share a flash image and still load the right calibration.
//
// Saved: the four HSV profiles, ROI, per-colour limits, merge/erosion
// settings, tracker parameters, and the camera exposure lock.
// ================================================================

bool visionNvsSave();       // persist current settings for the detected camera
bool visionNvsLoad();       // returns false if nothing stored for this camera
bool visionNvsClear();
void visionNvsStatus();
