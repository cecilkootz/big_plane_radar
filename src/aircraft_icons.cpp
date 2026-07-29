#include "aircraft_icons.h"

#include <math.h>
#include <pgmspace.h>

#include "aircraft_icon_data.inc"

namespace AircraftIcons {
namespace {

struct IconSheet {
    uint8_t canvasSize;
    size_t frameBytes;
    const uint8_t *frames;
};

static IconSheet sheetFor(bool helicopter, uint8_t planeSizeClass) {
    using namespace AircraftIconData;
    if (helicopter) {
        return {kHelicopterCanvasSize, kHelicopterFrameBytes, kHelicopterFrames};
    }
    if (planeSizeClass == 0) {
        return {kPlaneSmallCanvasSize, kPlaneSmallFrameBytes, kPlaneSmallFrames};
    }
    if (planeSizeClass >= 2) {
        return {kPlaneLargeCanvasSize, kPlaneLargeFrameBytes, kPlaneLargeFrames};
    }
    return {kPlaneNormalCanvasSize, kPlaneNormalFrameBytes, kPlaneNormalFrames};
}

static size_t frameIndexFor(float headingDegrees) {
    if (!isfinite(headingDegrees)) return 0;
    float normalized = fmodf(headingDegrees, 360.0f);
    if (normalized < 0.0f) normalized += 360.0f;
    return static_cast<size_t>(lroundf(
        normalized / AircraftIconData::kAngleStepDegrees
    )) % AircraftIconData::kAngleCount;
}

} // namespace

void draw(
    PanelDisplay::Canvas &canvas,
    bool helicopter,
    uint8_t planeSizeClass,
    float headingDegrees,
    int centerX,
    int centerY,
    uint16_t color
) {
    IconSheet sheet = sheetFor(helicopter, planeSizeClass);
    size_t frameIndex = frameIndexFor(headingDegrees);
    const uint8_t *frame = sheet.frames + frameIndex * sheet.frameBytes;
    canvas.blendAlphaMask4(
        centerX - sheet.canvasSize / 2,
        centerY - sheet.canvasSize / 2,
        sheet.canvasSize,
        sheet.canvasSize,
        frame,
        color
    );
}

int halfExtent(bool helicopter, uint8_t planeSizeClass) {
    return (sheetFor(helicopter, planeSizeClass).canvasSize + 1) / 2;
}

} // namespace AircraftIcons
