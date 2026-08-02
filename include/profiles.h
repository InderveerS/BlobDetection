#pragma once
#include "color_types.h"

// Centralized runtime configuration: active HSV profiles per color for
// whichever camera was detected at boot, the currently-selected detect
// color, and the min-blob-pixels detection knob. Everything else reads
// and writes through here.

void profilesInit(CameraId detectedCam);   // call once in setup(), after detectCameraId()

CameraId    getActiveCameraId();
const char* cameraName(CameraId id);
const char* colorName(ColorId id);

HsvRange getColorProfile(ColorId c);
void     setColorProfile(ColorId c, const HsvRange &r);

void    setActiveDetectColor(ColorId c);
ColorId getActiveDetectColor();

void resetColorToDefault(ColorId c);
void resetAllToDefaults();

void dumpProfiles();                 // print current camera's table as pasteable C code
void printRange(const HsvRange &r);  // shared "H:.. S:.. V:.." print helper

int  getMinBlobPx();
void setMinBlobPx(int px);