#pragma once

#include <stddef.h>
#include <stdint.h>

namespace RadarLabels {

static constexpr size_t kMaxLabels = 64;
static constexpr size_t kMaxAircraftObstacles = 64;
static constexpr size_t kMaxStaticObstacles = 8;

struct LabelLayoutBounds {
    float left;
    float top;
    float right;
    float bottom;
};

struct LabelLayoutInput {
    uint32_t id = 0;
    float anchorX = 0;
    float anchorY = 0;
    float width = 0;
    float height = 0;
    float symbolRadius = 0;
    float courseDeg = 0;
    float distanceKm = 0;
    bool courseValid = false;
    bool mustShow = false;
};

struct AircraftObstacle {
    float x = 0;
    float y = 0;
    float radius = 0;
};

struct LabelRectObstacle {
    float x = 0;
    float y = 0;
    float width = 0;
    float height = 0;
};

struct LabelLayoutOutput {
    float x = 0;
    float y = 0;
    bool visible = false;
};

struct LabelLayoutMetrics {
    size_t visibleCount = 0;
    size_t hiddenCount = 0;
    float maxOverlapPx = 0;
};

class LabelLayout {
public:
    void reset();

    void solve(
        const LabelLayoutInput *inputs,
        size_t inputCount,
        const AircraftObstacle *aircraftObstacles,
        size_t aircraftObstacleCount,
        const LabelRectObstacle *staticObstacles,
        size_t staticObstacleCount,
        const LabelLayoutBounds &bounds,
        uint32_t nowMs,
        uint32_t layoutRevision,
        float deltaSeconds,
        LabelLayoutOutput *outputs,
        LabelLayoutMetrics *metrics = nullptr
    );

private:
    struct State {
        uint32_t id = 0;
        uint32_t lastSeenMs = 0;
        uint32_t layoutRevision = 0;
        float x = 0;
        float y = 0;
        float anchorX = 0;
        float anchorY = 0;
        float width = 0;
        float height = 0;
        float orbitTargetAngle = 0;
        uint32_t orbitLockUntilMs = 0;
        uint8_t conflictFrames = 0;
        uint8_t cleanFrames = 0;
        int8_t orbitDirection = 0;
        bool occupied = false;
        bool visible = true;
        bool orbitTargetActive = false;
    };

    struct WorkItem {
        const LabelLayoutInput *input = nullptr;
        State *state = nullptr;
        size_t outputIndex = 0;
        float x = 0;
        float y = 0;
        float baseX = 0;
        float baseY = 0;
        float forceX = 0;
        float forceY = 0;
        float aircraftForceX = 0;
        float aircraftForceY = 0;
        float forwardX = 0;
        float forwardY = -1;
        float rightX = 1;
        float rightY = 0;
        bool isNew = false;
        bool orbiting = false;
    };

    State states_[kMaxLabels];
    WorkItem work_[kMaxLabels];
    size_t orbitCursor_ = 0;
    size_t collisionSearchCursor_ = 0;
};

}  // namespace RadarLabels
