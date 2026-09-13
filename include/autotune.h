#pragma once
#include "esp_camera.h"
#include "color_types.h"

void tuneStart(ColorId c);
void tuneProcessFrame(camera_fb_t *fb);   // call every loop() iteration while tuneIsActive()
void tuneSave();
void tuneCancel();
bool tuneIsActive();
