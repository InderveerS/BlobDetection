#pragma once
#include <stddef.h>

// Must match cfg.frame_size in camera_setup.cpp (currently FRAMESIZE_QVGA).
// If you change one, change the other. main.cpp validates every frame against
// these at runtime, so a silent sensor fallback gets caught instead of walking
// off the end of the buffer.
#define FRAME_W 320
#define FRAME_H 240

#define FRAME_PIXELS ((size_t)FRAME_W * (size_t)FRAME_H)
#define FRAME_BYTES  (FRAME_PIXELS * 2)   // RGB565 = 2 bytes/pixel