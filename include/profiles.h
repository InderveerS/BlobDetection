#pragma once
#include "color_types.h"

// Active HSV profiles for whichever camera was detected at boot, plus the
// colour that `set h/s/v` edits. Everything else reads and writes through here.

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
