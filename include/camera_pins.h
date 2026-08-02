#pragma once
// ================================================================
// CAMERA PINS — Freenove FNK0085 / CAMERA_MODEL_ESP32S3_EYE
// Source: https://docs.freenove.com/projects/fnk0085
// Same pinout for OV2640 and OV3660 — they're drop-in compatible
// on this socket. Do NOT change these.
// ================================================================
#define CAM_PWDN    -1
#define CAM_RESET   -1
#define CAM_XCLK    15
#define CAM_SIOD     4   // SCCB SDA
#define CAM_SIOC     5   // SCCB SCL
#define CAM_D0      11   // Y2
#define CAM_D1       9   // Y3
#define CAM_D2       8   // Y4
#define CAM_D3      10   // Y5
#define CAM_D4      12   // Y6
#define CAM_D5      18   // Y7
#define CAM_D6      17   // Y8
#define CAM_D7      16   // Y9
#define CAM_VSYNC    6
#define CAM_HREF     7
#define CAM_PCLK    13