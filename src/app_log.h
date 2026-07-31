#pragma once

#include <Arduino.h>

#define PLANE_RADAR_LOG_LEVEL_OFF 0
#define PLANE_RADAR_LOG_LEVEL_ERROR 1
#define PLANE_RADAR_LOG_LEVEL_INFO 2
#define PLANE_RADAR_LOG_LEVEL_DEBUG 3

#ifndef PLANE_RADAR_LOG_LEVEL
#define PLANE_RADAR_LOG_LEVEL PLANE_RADAR_LOG_LEVEL_INFO
#endif

#if PLANE_RADAR_LOG_LEVEL >= PLANE_RADAR_LOG_LEVEL_ERROR
#define RADAR_LOGE(...) do { Serial.printf(__VA_ARGS__); } while (0)
#define RADAR_LOGE_FLUSH() do { Serial.flush(); } while (0)
#else
#define RADAR_LOGE(...) do {} while (0)
#define RADAR_LOGE_FLUSH() do {} while (0)
#endif

#if PLANE_RADAR_LOG_LEVEL >= PLANE_RADAR_LOG_LEVEL_INFO
#define RADAR_LOGI(...) do { Serial.printf(__VA_ARGS__); } while (0)
#else
#define RADAR_LOGI(...) do {} while (0)
#endif

#if PLANE_RADAR_LOG_LEVEL >= PLANE_RADAR_LOG_LEVEL_DEBUG
#define RADAR_LOGD(...) do { Serial.printf(__VA_ARGS__); } while (0)
#else
#define RADAR_LOGD(...) do {} while (0)
#endif
