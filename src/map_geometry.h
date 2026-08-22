#pragma once

// Pure Web Mercator / XYZ tile math, free of Arduino dependencies so the native
// tests can exercise it.

#include <algorithm>
#include <cmath>
#include <stddef.h>
#include <stdint.h>

namespace RadarMap {

inline constexpr int MAP_TILE_SIZE = 256;
inline constexpr int MAP_MAX_TILE_COLUMNS = 8;
inline constexpr int MAP_MAX_TILE_ROWS = 8;
inline constexpr double WEB_MERCATOR_MAX_LATITUDE = 85.05112878;
inline constexpr double WEB_MERCATOR_METERS_PER_PIXEL_Z0 = 156543.03392804097;
inline constexpr double MAP_PI = 3.14159265358979323846;
inline constexpr double MAP_DEG_TO_RAD = MAP_PI / 180.0;

struct MapGeometry {
    int zoom = 0;
    double centerPixelX = 0;
    double centerPixelY = 0;
    double sourcePixelsPerDestinationPixel = 1;
    double worldPixelSize = 0;
    int64_t tileMinX = 0;
    int64_t tileMaxX = 0;
    int tileMinY = 0;
    int tileMaxY = 0;
    int tileColumns = 0;
    int tileRows = 0;
    int sourceWidth = 0;
    int sourceHeight = 0;
    // Horizontal span the strip actually has to hold, in source pixels. Sized
    // from the sampled range rather than from whole tiles: the leading and
    // trailing tiles are usually only partly sampled, and rounding them up to a
    // full 256 columns is what used to push wide views past the tile-count cap.
    int64_t stripOriginX = 0;
    int stripWidth = 0;
};

struct PixelSample {
    int64_t first = 0;
    int64_t second = 0;
    uint16_t weight = 0;
};

struct BilinearSample {
    uint16_t first = 0;
    uint16_t second = 0;
    uint16_t weight = 0;
};

inline int64_t floorDiv(int64_t value, int64_t divisor) {
    if (value >= 0) return value / divisor;
    return -((-value + divisor - 1) / divisor);
}

inline int wrapTileX(int64_t tileX, int zoom) {
    int64_t tileCount = 1LL << zoom;
    int64_t wrapped = tileX % tileCount;
    if (wrapped < 0) wrapped += tileCount;
    return static_cast<int>(wrapped);
}

inline PixelSample pixelSample(double coordinate) {
    double firstValue = std::floor(coordinate);
    double fraction = coordinate - firstValue;
    int weight = static_cast<int>(std::lround(fraction * 256.0));
    int64_t first = static_cast<int64_t>(firstValue);
    if (weight >= 256) {
        first++;
        weight = 0;
    }

    PixelSample result;
    result.first = first;
    result.second = weight == 0 ? first : first + 1;
    result.weight = static_cast<uint16_t>(weight);
    return result;
}

inline double sourceCoordinate(
    double centerPixel,
    int destinationIndex,
    int destinationSize,
    double scale
) {
    return centerPixel +
        (destinationIndex + 0.5 - destinationSize / 2.0) * scale - 0.5;
}

inline double clampSourceY(double coordinate, double worldPixelSize) {
    return std::max(0.0, std::min(worldPixelSize - 1.0, coordinate));
}

// The zoom is rounded up so a destination pixel never samples less than one
// source pixel, which caps the scale below 2 and the strip span below twice the
// destination width plus the bilinear partner pixel.
inline int stripCapacityWidth(int destinationWidth) {
    return 2 * destinationWidth + 2;
}

inline MapGeometry mapGeometry(
    double centerLat,
    double centerLon,
    float outerKm,
    int radarRadius,
    int destinationWidth,
    int destinationHeight
) {
    MapGeometry result;
    double latitude = std::max(
        -WEB_MERCATOR_MAX_LATITUDE,
        std::min(WEB_MERCATOR_MAX_LATITUDE, centerLat)
    );
    double longitude = std::fmod(centerLon + 180.0, 360.0);
    if (longitude < 0) longitude += 360.0;
    longitude -= 180.0;

    double metersPerDestinationPixel = (outerKm * 1000.0) / radarRadius;
    double latitudeScale = std::max(0.01, std::cos(latitude * MAP_DEG_TO_RAD));
    double rawZoom = std::log2(
        (WEB_MERCATOR_METERS_PER_PIXEL_Z0 * latitudeScale) /
        metersPerDestinationPixel
    );
    result.zoom = std::max(0, std::min(20, static_cast<int>(std::ceil(rawZoom))));
    result.worldPixelSize = std::ldexp(static_cast<double>(MAP_TILE_SIZE), result.zoom);

    double latitudeRadians = latitude * MAP_DEG_TO_RAD;
    double sinLatitude = std::sin(latitudeRadians);
    result.centerPixelX = ((longitude + 180.0) / 360.0) * result.worldPixelSize;
    result.centerPixelY =
        (0.5 - std::log((1.0 + sinLatitude) / (1.0 - sinLatitude)) / (4.0 * MAP_PI)) *
        result.worldPixelSize;

    double metersPerSourcePixel =
        (WEB_MERCATOR_METERS_PER_PIXEL_Z0 * latitudeScale) /
        static_cast<double>(1UL << result.zoom);
    result.sourcePixelsPerDestinationPixel =
        metersPerDestinationPixel / metersPerSourcePixel;
    result.sourceWidth = static_cast<int>(std::ceil(
        destinationWidth * result.sourcePixelsPerDestinationPixel
    ));
    result.sourceHeight = static_cast<int>(std::ceil(
        destinationHeight * result.sourcePixelsPerDestinationPixel
    ));

    PixelSample firstX = pixelSample(sourceCoordinate(
        result.centerPixelX,
        0,
        destinationWidth,
        result.sourcePixelsPerDestinationPixel
    ));
    PixelSample lastX = pixelSample(sourceCoordinate(
        result.centerPixelX,
        destinationWidth - 1,
        destinationWidth,
        result.sourcePixelsPerDestinationPixel
    ));
    PixelSample firstY = pixelSample(clampSourceY(sourceCoordinate(
        result.centerPixelY,
        0,
        destinationHeight,
        result.sourcePixelsPerDestinationPixel
    ), result.worldPixelSize));
    PixelSample lastY = pixelSample(clampSourceY(sourceCoordinate(
        result.centerPixelY,
        destinationHeight - 1,
        destinationHeight,
        result.sourcePixelsPerDestinationPixel
    ), result.worldPixelSize));

    result.tileMinX = floorDiv(firstX.first, MAP_TILE_SIZE);
    result.tileMaxX = floorDiv(lastX.second, MAP_TILE_SIZE);
    result.tileMinY = static_cast<int>(floorDiv(firstY.first, MAP_TILE_SIZE));
    result.tileMaxY = static_cast<int>(floorDiv(lastY.second, MAP_TILE_SIZE));
    int worldTileCount = 1 << result.zoom;
    result.tileMinY = std::max(0, std::min(worldTileCount - 1, result.tileMinY));
    result.tileMaxY = std::max(0, std::min(worldTileCount - 1, result.tileMaxY));
    result.tileColumns = static_cast<int>(result.tileMaxX - result.tileMinX + 1);
    result.tileRows = result.tileMaxY - result.tileMinY + 1;
    result.stripOriginX = firstX.first;
    result.stripWidth = static_cast<int>(lastX.second - firstX.first + 1);
    return result;
}

} // namespace RadarMap
