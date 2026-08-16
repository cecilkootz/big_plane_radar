#pragma once

#ifndef PLANE_RADAR_RGB_PCLK_HZ
#define PLANE_RADAR_RGB_PCLK_HZ (13 * 1000 * 1000)
#endif

#ifndef PLANE_RADAR_RGB_7B_PCLK_HZ
// Applied automatically when the 7B is auto-detected (make7BBoardConfig).
// 30 MHz needs the high-perf SDK (build_arduino_highperf.sh): its 64-byte cache
// line + PSRAM XIP give the scan-out DMA the bandwidth to avoid tearing at
// 1024x600. On the stock precompiled SDK 30 MHz tears; for a stock build pass
// RGB_7B_PCLK_MHZ=14.
#define PLANE_RADAR_RGB_7B_PCLK_HZ (30 * 1000 * 1000)
#endif

// 0: auto, 7: ESP32-S3-Touch-LCD-7, 8: ESP32-S3-Touch-LCD-7B.
#ifndef PLANE_RADAR_DISPLAY_PROFILE
#define PLANE_RADAR_DISPLAY_PROFILE 0
#endif

#if PLANE_RADAR_DISPLAY_PROFILE != 0 && \
    PLANE_RADAR_DISPLAY_PROFILE != 7 && \
    PLANE_RADAR_DISPLAY_PROFILE != 8
#error "PLANE_RADAR_DISPLAY_PROFILE must be 0, 7, or 8"
#endif

#ifndef PLANE_RADAR_RGB_BOUNCE_LINES
#define PLANE_RADAR_RGB_BOUNCE_LINES 10
#endif

#if PLANE_RADAR_RGB_BOUNCE_LINES != 10 && PLANE_RADAR_RGB_BOUNCE_LINES != 20
#error "PLANE_RADAR_RGB_BOUNCE_LINES must be 10 or 20"
#endif
