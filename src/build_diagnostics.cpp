#include "build_diagnostics.h"

#include "app_log.h"
#include "display_tuning.h"

#include <esp_arduino_version.h>
#include <esp_heap_caps.h>
#include <esp_idf_version.h>
#include <sdkconfig.h>

#if defined(PLANE_RADAR_REQUIRE_HIGH_PERF) && PLANE_RADAR_REQUIRE_HIGH_PERF
#if !defined(CONFIG_COMPILER_OPTIMIZATION_PERF) || !CONFIG_COMPILER_OPTIMIZATION_PERF
#error "High-performance build requires CONFIG_COMPILER_OPTIMIZATION_PERF"
#endif
#if !defined(CONFIG_ESP32S3_DATA_CACHE_LINE_64B) || !CONFIG_ESP32S3_DATA_CACHE_LINE_64B
#error "High-performance build requires a 64-byte ESP32-S3 data cache line"
#endif
#if !defined(CONFIG_SPIRAM_XIP_FROM_PSRAM) || !CONFIG_SPIRAM_XIP_FROM_PSRAM
#error "High-performance build requires XIP from PSRAM"
#endif
#if !defined(CONFIG_SPIRAM_FETCH_INSTRUCTIONS) || !CONFIG_SPIRAM_FETCH_INSTRUCTIONS
#error "High-performance build requires instruction fetch from PSRAM"
#endif
#if !defined(CONFIG_SPIRAM_RODATA) || !CONFIG_SPIRAM_RODATA
#error "High-performance build requires read-only data in PSRAM"
#endif
#if !defined(CONFIG_SPIRAM_MODE_OCT) || !CONFIG_SPIRAM_MODE_OCT
#error "High-performance build requires Octal PSRAM"
#endif
#if !defined(CONFIG_SPIRAM_SPEED_80M) || !CONFIG_SPIRAM_SPEED_80M
#error "High-performance build requires 80 MHz PSRAM"
#endif
#endif

namespace BuildDiagnostics {

void logBuildConfiguration() {
#if PLANE_RADAR_LOG_LEVEL >= PLANE_RADAR_LOG_LEVEL_DEBUG
#if defined(PLANE_RADAR_REQUIRE_HIGH_PERF) && PLANE_RADAR_REQUIRE_HIGH_PERF
    constexpr int highPerfRequired = 1;
#else
    constexpr int highPerfRequired = 0;
#endif
#if defined(CONFIG_COMPILER_OPTIMIZATION_PERF) && CONFIG_COMPILER_OPTIMIZATION_PERF
    constexpr int optimizationPerf = 1;
#else
    constexpr int optimizationPerf = 0;
#endif
#if defined(CONFIG_SPIRAM_XIP_FROM_PSRAM) && CONFIG_SPIRAM_XIP_FROM_PSRAM
    constexpr int xipFromPsram = 1;
#else
    constexpr int xipFromPsram = 0;
#endif
#if defined(CONFIG_SPIRAM_MODE_OCT) && CONFIG_SPIRAM_MODE_OCT
    constexpr int octalPsram = 1;
#else
    constexpr int octalPsram = 0;
#endif

    constexpr uint32_t horizontalTotal = 800U + 4U + 8U + 8U;
    constexpr uint32_t verticalTotal = 480U + 4U + 8U + 8U;
    const float panelFps = static_cast<float>(PLANE_RADAR_RGB_PCLK_HZ) /
                           static_cast<float>(horizontalTotal * verticalTotal);

    RADAR_LOGD(
        "[build] arduino=%d.%d.%d idf=%d.%d.%d require_high_perf=%d\n",
        ESP_ARDUINO_VERSION_MAJOR,
        ESP_ARDUINO_VERSION_MINOR,
        ESP_ARDUINO_VERSION_PATCH,
        ESP_IDF_VERSION_MAJOR,
        ESP_IDF_VERSION_MINOR,
        ESP_IDF_VERSION_PATCH,
        highPerfRequired
    );
    RADAR_LOGD(
        "[build] perf_o2=%d xip_psram=%d octal_psram=%d psram_mhz=%d cache_line=%d\n",
        optimizationPerf,
        xipFromPsram,
        octalPsram,
#ifdef CONFIG_SPIRAM_SPEED
        CONFIG_SPIRAM_SPEED,
#else
        0,
#endif
#ifdef CONFIG_ESP32S3_DATA_CACHE_LINE_SIZE
        CONFIG_ESP32S3_DATA_CACHE_LINE_SIZE
#else
        0
#endif
    );
    RADAR_LOGD(
        "[build] rgb_pclk_hz=%lu bounce_lines=%d bounce_bytes=%lu panel_fps=%.2f\n",
        static_cast<unsigned long>(PLANE_RADAR_RGB_PCLK_HZ),
        PLANE_RADAR_RGB_BOUNCE_LINES,
        static_cast<unsigned long>(800U * PLANE_RADAR_RGB_BOUNCE_LINES * sizeof(uint16_t) * 2U),
        panelFps
    );
#endif
}

void logMemory(const char *stage) {
#if PLANE_RADAR_LOG_LEVEL >= PLANE_RADAR_LOG_LEVEL_DEBUG
    RADAR_LOGD(
        "[memory] %s heap=%u internal_free=%u internal_largest=%u psram_free=%u psram_largest=%u psram_total=%u\n",
        stage,
        static_cast<unsigned>(ESP.getFreeHeap()),
        static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
        static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
        static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)),
        static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM)),
        static_cast<unsigned>(heap_caps_get_total_size(MALLOC_CAP_SPIRAM))
    );
#else
    (void)stage;
#endif
}

} // namespace BuildDiagnostics
