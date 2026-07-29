#pragma once

#include <Arduino.h>

#include "panel_display.h"

namespace AircraftIcons {

void draw(
    PanelDisplay::Canvas &canvas,
    bool helicopter,
    uint8_t planeSizeClass,
    float headingDegrees,
    int centerX,
    int centerY,
    uint16_t color
);

int halfExtent(bool helicopter, uint8_t planeSizeClass);

} // namespace AircraftIcons
