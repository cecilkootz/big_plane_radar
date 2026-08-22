#include "map_geometry.h"

#include <stdio.h>
#include <stdlib.h>

using RadarMap::MAP_MAX_TILE_COLUMNS;
using RadarMap::MAP_MAX_TILE_ROWS;
using RadarMap::MAP_TILE_SIZE;
using RadarMap::MapGeometry;
using RadarMap::PixelSample;
using RadarMap::mapGeometry;
using RadarMap::pixelSample;
using RadarMap::sourceCoordinate;
using RadarMap::stripCapacityWidth;

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        exit(1); \
    } \
} while (0)

struct PanelGeometry {
    const char *name;
    int mapWidth;
    int mapHeight;
};

// configureDisplayLayout() in main.cpp derives these from the detected panel.
static const PanelGeometry kPanels[] = {
    {"LCD-7 800x480", 520, 480},
    {"LCD-7B 1024x600", 680, 600},
};

static const float kRangeOuterKm[] = {6.7f, 13.3f, 20.0f, 33.3f};

static int radarRadiusFor(const PanelGeometry &panel) {
    int byHeight = panel.mapHeight / 2 - 22;
    int byWidth = panel.mapWidth / 2 - 42;
    return byHeight < byWidth ? byHeight : byWidth;
}

// Every pixel the renderer samples has to fall inside the strip, and the strip
// has to fit the buffer that fetchStadia allocates up front.
static void checkGeometryFitsStrip(const PanelGeometry &panel, const MapGeometry &geometry) {
    int capacity = stripCapacityWidth(panel.mapWidth);
    CHECK(geometry.stripWidth > 0);
    CHECK(geometry.stripWidth <= capacity);
    CHECK(geometry.tileColumns > 0 && geometry.tileColumns <= MAP_MAX_TILE_COLUMNS);
    CHECK(geometry.tileRows > 0 && geometry.tileRows <= MAP_MAX_TILE_ROWS);

    for (int x = 0; x < panel.mapWidth; x++) {
        PixelSample sample = pixelSample(sourceCoordinate(
            geometry.centerPixelX,
            x,
            panel.mapWidth,
            geometry.sourcePixelsPerDestinationPixel
        ));
        int64_t first = sample.first - geometry.stripOriginX;
        int64_t second = sample.second - geometry.stripOriginX;
        CHECK(first >= 0);
        CHECK(second < geometry.stripWidth);
    }

    // Tiles are still fetched on tile boundaries, so the grid must cover the
    // strip even though the strip no longer starts on one.
    int64_t tileSpanStart = geometry.tileMinX * MAP_TILE_SIZE;
    int64_t tileSpanEnd = (geometry.tileMaxX + 1) * MAP_TILE_SIZE;
    CHECK(tileSpanStart <= geometry.stripOriginX);
    CHECK(tileSpanEnd >= geometry.stripOriginX + geometry.stripWidth);
}

static void testStripFitsEverywhere() {
    for (const PanelGeometry &panel : kPanels) {
        int radarRadius = radarRadiusFor(panel);
        for (float outerKm : kRangeOuterKm) {
            for (int latTenths = -850; latTenths <= 850; latTenths += 1) {
                for (int lon = -180; lon < 180; lon += 7) {
                    MapGeometry geometry = mapGeometry(
                        latTenths / 10.0,
                        lon,
                        outerKm,
                        radarRadius,
                        panel.mapWidth,
                        panel.mapHeight
                    );
                    checkGeometryFitsStrip(panel, geometry);
                }
            }
        }
    }
}

// Rounding the zoom up keeps the scale under 2 source pixels per destination
// pixel, which is what bounds the strip.
static void testScaleStaysBelowTwo() {
    for (const PanelGeometry &panel : kPanels) {
        int radarRadius = radarRadiusFor(panel);
        for (float outerKm : kRangeOuterKm) {
            for (int latTenths = -850; latTenths <= 850; latTenths += 3) {
                MapGeometry geometry = mapGeometry(
                    latTenths / 10.0,
                    0.0,
                    outerKm,
                    radarRadius,
                    panel.mapWidth,
                    panel.mapHeight
                );
                CHECK(geometry.sourcePixelsPerDestinationPixel >= 1.0);
                CHECK(geometry.sourcePixelsPerDestinationPixel < 2.0);
                CHECK(geometry.sourceWidth <= 2 * panel.mapWidth);
                CHECK(geometry.sourceHeight <= 2 * panel.mapHeight);
            }
        }
    }
}

// Regression: on the 7B the widest source spans need a seven-column grid. The
// strip used to be sized as tileColumns * 256 and capped at six columns, so
// these views were rejected as "XYZ GEOMETRY TOO LARGE" and the range lost its
// map. San Francisco and Tokyo both lose the widest preset that way.
static void testSevenColumnGridLoads() {
    struct SevenColumnCase {
        const char *name;
        double lat;
        double lon;
        float outerKm;
    };
    static const SevenColumnCase kCases[] = {
        {"San Francisco 25km", 37.7749, -122.4194, 33.3f},
        {"Tokyo 25km", 35.6762, 139.6503, 33.3f},
        {"antimeridian 5km", -50.9, -166.0, 6.7f},
    };

    const PanelGeometry &panel = kPanels[1];
    for (const SevenColumnCase &testCase : kCases) {
        MapGeometry geometry = mapGeometry(
            testCase.lat,
            testCase.lon,
            testCase.outerKm,
            radarRadiusFor(panel),
            panel.mapWidth,
            panel.mapHeight
        );
        CHECK(geometry.tileColumns == 7);
        CHECK(geometry.stripWidth <= stripCapacityWidth(panel.mapWidth));
        // The seven tiles span 1792 columns; the strip only holds what is sampled.
        CHECK(geometry.stripWidth < geometry.tileColumns * MAP_TILE_SIZE);
        checkGeometryFitsStrip(panel, geometry);
    }
}

// The strip is the largest PSRAM block the map load asks for, and it is taken
// while the four view caches are already resident.
static void testStripStaysSmallerThanTileAlignedStrip() {
    for (const PanelGeometry &panel : kPanels) {
        size_t capacityBytes =
            static_cast<size_t>(stripCapacityWidth(panel.mapWidth)) *
            (MAP_TILE_SIZE + 1) * sizeof(uint16_t);
        size_t tileAlignedBytes =
            static_cast<size_t>(MAP_MAX_TILE_COLUMNS) * MAP_TILE_SIZE *
            (MAP_TILE_SIZE + 1) * sizeof(uint16_t);
        CHECK(capacityBytes < tileAlignedBytes);
        printf("  %s strip capacity %zu bytes\n", panel.name, capacityBytes);
    }
}

int main() {
    testStripFitsEverywhere();
    testScaleStaysBelowTwo();
    testSevenColumnGridLoads();
    testStripStaysSmallerThanTileAlignedStrip();
    printf("map geometry tests passed\n");
    return 0;
}
