#pragma once

#include <stddef.h>

namespace AircraftListScroll {

inline size_t maxOffset(size_t itemCount, size_t visibleRows) {
    return itemCount > visibleRows ? itemCount - visibleRows : 0;
}

inline size_t clampOffset(
    size_t offset,
    size_t itemCount,
    size_t visibleRows
) {
    size_t maximum = maxOffset(itemCount, visibleRows);
    return offset < maximum ? offset : maximum;
}

inline size_t offsetForDrag(
    size_t startOffset,
    int dragUpPixels,
    size_t itemCount,
    size_t visibleRows,
    int startThresholdPixels,
    int rowStepPixels
) {
    size_t maximum = maxOffset(itemCount, visibleRows);
    startOffset = startOffset < maximum ? startOffset : maximum;
    int magnitude = dragUpPixels < 0 ? -dragUpPixels : dragUpPixels;
    if (magnitude < startThresholdPixels || rowStepPixels <= 0) {
        return startOffset;
    }

    size_t rows = 1 + static_cast<size_t>(
        (magnitude - startThresholdPixels) / rowStepPixels
    );
    if (dragUpPixels > 0) {
        size_t remaining = maximum - startOffset;
        return startOffset + (rows < remaining ? rows : remaining);
    }
    return rows < startOffset ? startOffset - rows : 0;
}

}  // namespace AircraftListScroll
