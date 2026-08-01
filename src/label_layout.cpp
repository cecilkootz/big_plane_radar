#include "label_layout.h"

#include <math.h>
#include <string.h>

namespace RadarLabels {
namespace {

static constexpr uint32_t kStateTtlMs = 60000;
static constexpr float kAnchorJumpPx = 80.0f;
static constexpr float kMinimumSymbolGapPx = 2.0f;
static constexpr float kPreferredSymbolGapPx = 4.0f;
static constexpr float kNormalMaxGapPx = 64.0f;
static constexpr float kPriorityMaxGapPx = 96.0f;
static constexpr float kMaxMovementPxPerSecond = 64.0f;
static constexpr float kMovementDeadZonePx = 0.25f;
static constexpr float kCourseAvoidDistancePx = 80.0f;
static constexpr float kCourseConeCosineSquared = 0.58682409f;
static constexpr float kInverseSqrtTwo = 0.70710678f;
static constexpr float kOrbitMinExcessPx = 0.75f;
static constexpr float kOrbitForceLimitPx = 2.0f;
static constexpr size_t kOrbitLabelsPerFrame = 16;
static constexpr uint8_t kHideAfterConflictFrames = 6;
static constexpr uint8_t kShowAfterCleanFrames = 20;

static float clampFloat(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static float minFloat(float a, float b) {
    return a < b ? a : b;
}

static float maxFloat(float a, float b) {
    return a > b ? a : b;
}

static float rectOverlapDepth(
    float ax,
    float ay,
    float aw,
    float ah,
    float bx,
    float by,
    float bw,
    float bh
) {
    float overlapX = minFloat(ax + aw, bx + bw) - maxFloat(ax, bx);
    float overlapY = minFloat(ay + ah, by + bh) - maxFloat(ay, by);
    if (overlapX <= 0.0f || overlapY <= 0.0f) return 0.0f;
    return minFloat(overlapX, overlapY);
}

static float pointToRectDistanceSquared(
    float px,
    float py,
    float x,
    float y,
    float width,
    float height
) {
    float nearestX = clampFloat(px, x, x + width);
    float nearestY = clampFloat(py, y, y + height);
    float dx = px - nearestX;
    float dy = py - nearestY;
    return dx * dx + dy * dy;
}

static float approximateLength(float x, float y) {
    float ax = fabsf(x);
    float ay = fabsf(y);
    float largest = maxFloat(ax, ay);
    float smallest = minFloat(ax, ay);
    return largest + smallest * 0.41421356f;
}

static void clampToBounds(
    float &x,
    float &y,
    float width,
    float height,
    const LabelLayoutBounds &bounds
) {
    float maxX = maxFloat(bounds.left, bounds.right - width);
    float maxY = maxFloat(bounds.top, bounds.bottom - height);
    x = clampFloat(x, bounds.left, maxX);
    y = clampFloat(y, bounds.top, maxY);
}

static void courseVectors(
    const LabelLayoutInput &input,
    float &forwardX,
    float &forwardY,
    float &rightX,
    float &rightY
) {
    float radians = input.courseDeg * 0.01745329251994329577f;
    forwardX = sinf(radians);
    forwardY = -cosf(radians);
    rightX = cosf(radians);
    rightY = sinf(radians);
}

static bool isInsideForwardCone(
    const LabelLayoutInput &input,
    float centerX,
    float centerY,
    float forwardX,
    float forwardY
) {
    if (!input.courseValid) return false;
    float dx = centerX - input.anchorX;
    float dy = centerY - input.anchorY;
    float distanceSquared = dx * dx + dy * dy;
    if (distanceSquared < 0.000001f ||
        distanceSquared > kCourseAvoidDistancePx * kCourseAvoidDistancePx) {
        return false;
    }
    float forwardProjection = dx * forwardX + dy * forwardY;
    return forwardProjection > 0.0f &&
        forwardProjection * forwardProjection >=
            kCourseConeCosineSquared * distanceSquared;
}

static bool inputContainsId(
    const LabelLayoutInput *inputs,
    size_t inputCount,
    uint32_t id
) {
    for (size_t i = 0; i < inputCount; i++) {
        if (inputs[i].id == id) return true;
    }
    return false;
}

}  // namespace

void LabelLayout::reset() {
    memset(states_, 0, sizeof(states_));
    memset(work_, 0, sizeof(work_));
    orbitCursor_ = 0;
}

void LabelLayout::solve(
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
    LabelLayoutMetrics *metrics
) {
    if (outputs == nullptr) return;
    if (inputCount > kMaxLabels) inputCount = kMaxLabels;
    if (aircraftObstacleCount > kMaxAircraftObstacles) {
        aircraftObstacleCount = kMaxAircraftObstacles;
    }
    if (staticObstacleCount > kMaxStaticObstacles) {
        staticObstacleCount = kMaxStaticObstacles;
    }

    for (size_t i = 0; i < inputCount; i++) {
        outputs[i] = LabelLayoutOutput();
    }
    if (metrics != nullptr) *metrics = LabelLayoutMetrics();
    if (inputs == nullptr || inputCount == 0) return;

    for (size_t i = 0; i < kMaxLabels; i++) {
        if (states_[i].occupied && nowMs - states_[i].lastSeenMs > kStateTtlMs) {
            states_[i] = State();
        }
    }

    size_t workCount = 0;
    for (size_t inputIndex = 0; inputIndex < inputCount; inputIndex++) {
        const LabelLayoutInput &input = inputs[inputIndex];
        if (input.id == 0 || input.width <= 0.0f || input.height <= 0.0f) continue;

        State *state = nullptr;
        size_t hashIndex = static_cast<size_t>(input.id % kMaxLabels);
        for (size_t probe = 0; probe < kMaxLabels; probe++) {
            size_t stateIndex = (hashIndex + probe) % kMaxLabels;
            if (states_[stateIndex].occupied && states_[stateIndex].id == input.id) {
                state = &states_[stateIndex];
                break;
            }
        }

        bool isNew = false;
        if (state == nullptr) {
            size_t replacement = kMaxLabels;
            uint32_t oldestAge = 0;
            for (size_t probe = 0; probe < kMaxLabels; probe++) {
                size_t stateIndex = (hashIndex + probe) % kMaxLabels;
                if (!states_[stateIndex].occupied) {
                    replacement = stateIndex;
                    break;
                }
                if (inputContainsId(inputs, inputCount, states_[stateIndex].id)) continue;
                uint32_t age = nowMs - states_[stateIndex].lastSeenMs;
                if (replacement == kMaxLabels || age > oldestAge) {
                    replacement = stateIndex;
                    oldestAge = age;
                }
            }
            if (replacement == kMaxLabels) continue;
            states_[replacement] = State();
            state = &states_[replacement];
            state->occupied = true;
            state->id = input.id;
            state->visible = true;
            isNew = true;
        }

        WorkItem &work = work_[workCount++];
        work = WorkItem();
        work.input = &input;
        work.state = state;
        work.outputIndex = inputIndex;
        work.isNew = isNew;
        state->lastSeenMs = nowMs;
    }

    // Stable ICAO ordering keeps initialization and pairwise relaxation independent
    // from the distance sort used by the renderer.
    for (size_t i = 1; i < workCount; i++) {
        WorkItem value = work_[i];
        size_t j = i;
        while (j > 0 && work_[j - 1].input->id > value.input->id) {
            work_[j] = work_[j - 1];
            j--;
        }
        work_[j] = value;
    }

    bool hasNewLabel = false;
    for (size_t i = 0; i < workCount; i++) {
        WorkItem &work = work_[i];
        State &state = *work.state;
        const LabelLayoutInput &input = *work.input;
        courseVectors(
            input,
            work.forwardX,
            work.forwardY,
            work.rightX,
            work.rightY
        );

        float anchorDx = input.anchorX - state.anchorX;
        float anchorDy = input.anchorY - state.anchorY;
        float anchorMovementSquared = anchorDx * anchorDx + anchorDy * anchorDy;
        bool reinitialize = work.isNew ||
            state.layoutRevision != layoutRevision ||
            anchorMovementSquared > kAnchorJumpPx * kAnchorJumpPx;

        if (!reinitialize) {
            float centerX = state.x + state.width * 0.5f + anchorDx;
            float centerY = state.y + state.height * 0.5f + anchorDy;
            work.x = centerX - input.width * 0.5f;
            work.y = centerY - input.height * 0.5f;
            clampToBounds(work.x, work.y, input.width, input.height, bounds);
        } else {
            work.isNew = true;
            hasNewLabel = true;

            const float directions[8][2] = {
                {work.rightX, work.rightY},
                {-work.rightX, -work.rightY},
                {
                    (work.rightX - work.forwardX) * kInverseSqrtTwo,
                    (work.rightY - work.forwardY) * kInverseSqrtTwo
                },
                {
                    (-work.rightX - work.forwardX) * kInverseSqrtTwo,
                    (-work.rightY - work.forwardY) * kInverseSqrtTwo
                },
                {-work.forwardX, -work.forwardY},
                {
                    (work.rightX + work.forwardX) * kInverseSqrtTwo,
                    (work.rightY + work.forwardY) * kInverseSqrtTwo
                },
                {
                    (-work.rightX + work.forwardX) * kInverseSqrtTwo,
                    (-work.rightY + work.forwardY) * kInverseSqrtTwo
                },
                {work.forwardX, work.forwardY},
            };

            float bestScore = 1.0e30f;
            float bestX = input.anchorX;
            float bestY = input.anchorY;
            for (size_t candidateIndex = 0; candidateIndex < 8; candidateIndex++) {
                float directionX = directions[candidateIndex][0];
                float directionY = directions[candidateIndex][1];

                float halfWidth = input.width * 0.5f;
                float halfHeight = input.height * 0.5f;
                float extent = fabsf(directionX) * halfWidth +
                    fabsf(directionY) * halfHeight;
                float centerDistance = input.symbolRadius +
                    kPreferredSymbolGapPx + extent;
                float candidateX = input.anchorX + directionX * centerDistance - halfWidth;
                float candidateY = input.anchorY + directionY * centerDistance - halfHeight;
                float unclampedX = candidateX;
                float unclampedY = candidateY;
                clampToBounds(candidateX, candidateY, input.width, input.height, bounds);

                float score = static_cast<float>(candidateIndex) * 0.01f;
                score += (fabsf(candidateX - unclampedX) + fabsf(candidateY - unclampedY)) * 80.0f;
                if (isInsideForwardCone(
                        input,
                        candidateX + halfWidth,
                        candidateY + halfHeight,
                        work.forwardX,
                        work.forwardY)) {
                    score += 300.0f;
                }
                for (size_t obstacleIndex = 0;
                     obstacleIndex < aircraftObstacleCount;
                     obstacleIndex++) {
                    const AircraftObstacle &obstacle = aircraftObstacles[obstacleIndex];
                    float nearestX = clampFloat(
                        obstacle.x,
                        candidateX,
                        candidateX + input.width
                    );
                    float nearestY = clampFloat(
                        obstacle.y,
                        candidateY,
                        candidateY + input.height
                    );
                    float dx = obstacle.x - nearestX;
                    float dy = obstacle.y - nearestY;
                    float distanceSquared = dx * dx + dy * dy;
                    float required = obstacle.radius + kMinimumSymbolGapPx;
                    float requiredSquared = required * required;
                    if (distanceSquared < requiredSquared) {
                        score += (requiredSquared - distanceSquared) *
                            (60.0f / required);
                    }
                }
                for (size_t obstacleIndex = 0;
                     obstacleIndex < staticObstacleCount;
                     obstacleIndex++) {
                    const LabelRectObstacle &obstacle = staticObstacles[obstacleIndex];
                    score += rectOverlapDepth(
                        candidateX,
                        candidateY,
                        input.width,
                        input.height,
                        obstacle.x,
                        obstacle.y,
                        obstacle.width,
                        obstacle.height
                    ) * 80.0f;
                }
                for (size_t previous = 0; previous < i; previous++) {
                    score += rectOverlapDepth(
                        candidateX,
                        candidateY,
                        input.width,
                        input.height,
                        work_[previous].x,
                        work_[previous].y,
                        work_[previous].input->width,
                        work_[previous].input->height
                    ) * 40.0f;
                }
                if (score < bestScore) {
                    bestScore = score;
                    bestX = candidateX;
                    bestY = candidateY;
                }
            }
            work.x = bestX;
            work.y = bestY;
        }

        work.baseX = work.x;
        work.baseY = work.y;
        state.anchorX = input.anchorX;
        state.anchorY = input.anchorY;
        state.width = input.width;
        state.height = input.height;
        state.layoutRevision = layoutRevision;
    }

    auto updateAircraftForce = [&](WorkItem &work) {
        const LabelLayoutInput &input = *work.input;
        work.aircraftForceX = 0;
        work.aircraftForceY = 0;
        for (size_t obstacleIndex = 0;
             obstacleIndex < aircraftObstacleCount;
             obstacleIndex++) {
            const AircraftObstacle &obstacle = aircraftObstacles[obstacleIndex];
            float required = obstacle.radius + kMinimumSymbolGapPx;
            bool insideX = obstacle.x >= work.x &&
                obstacle.x <= work.x + input.width;
            bool insideY = obstacle.y >= work.y &&
                obstacle.y <= work.y + input.height;
            if (insideX && insideY) {
                float leftDistance = obstacle.x - work.x;
                float rightDistance = work.x + input.width - obstacle.x;
                float topDistance = obstacle.y - work.y;
                float bottomDistance = work.y + input.height - obstacle.y;
                if (leftDistance <= rightDistance &&
                    leftDistance <= topDistance &&
                    leftDistance <= bottomDistance) {
                    work.aircraftForceX -= (leftDistance + required) * 0.7f;
                } else if (rightDistance <= topDistance &&
                           rightDistance <= bottomDistance) {
                    work.aircraftForceX += (rightDistance + required) * 0.7f;
                } else if (topDistance <= bottomDistance) {
                    work.aircraftForceY -= (topDistance + required) * 0.7f;
                } else {
                    work.aircraftForceY += (bottomDistance + required) * 0.7f;
                }
                continue;
            }

            float nearestX = clampFloat(obstacle.x, work.x, work.x + input.width);
            float nearestY = clampFloat(obstacle.y, work.y, work.y + input.height);
            float pushX = nearestX - obstacle.x;
            float pushY = nearestY - obstacle.y;
            float distanceSquared = pushX * pushX + pushY * pushY;
            if (distanceSquared >= required * required) continue;
            float distance = sqrtf(distanceSquared);
            if (distance < 0.001f) continue;
            float strength = (required - distance) * 0.7f;
            work.aircraftForceX += (pushX / distance) * strength;
            work.aircraftForceY += (pushY / distance) * strength;
        }
    };

    const size_t iterations = hasNewLabel ? 12 : 3;
    size_t orbitStart = workCount > 0 ? orbitCursor_ % workCount : 0;
    for (size_t iteration = 0; iteration < iterations; iteration++) {
        // Aircraft move much more slowly than labels. Reusing the aggregate force
        // for three relaxation steps preserves all obstacles while avoiding a
        // 64x64 scan on every sub-step.
        if (iteration % 3 == 0) {
            for (size_t i = 0; i < workCount; i++) {
                updateAircraftForce(work_[i]);
                work_[i].labelForceX = 0;
                work_[i].labelForceY = 0;
            }
            for (size_t a = 0; a < workCount; a++) {
                for (size_t b = a + 1; b < workCount; b++) {
                    WorkItem &first = work_[a];
                    WorkItem &second = work_[b];
                    bool firstVisible = first.state->visible;
                    bool secondVisible = second.state->visible;
                    if (!firstVisible && !secondVisible) continue;

                    float overlapX = minFloat(
                        first.x + first.input->width,
                        second.x + second.input->width
                    ) - maxFloat(first.x, second.x);
                    float overlapY = minFloat(
                        first.y + first.input->height,
                        second.y + second.input->height
                    ) - maxFloat(first.y, second.y);
                    if (overlapX <= 0.0f || overlapY <= 0.0f) continue;

                    float firstWeight = 0.5f;
                    float secondWeight = 0.5f;
                    if (first.input->mustShow && !second.input->mustShow) {
                        firstWeight = 0.0f;
                        secondWeight = 1.0f;
                    } else if (!first.input->mustShow && second.input->mustShow) {
                        firstWeight = 1.0f;
                        secondWeight = 0.0f;
                    } else if (firstVisible && !secondVisible) {
                        firstWeight = 0.0f;
                        secondWeight = 1.0f;
                    } else if (!firstVisible && secondVisible) {
                        firstWeight = 1.0f;
                        secondWeight = 0.0f;
                    }

                    float firstCenterX = first.x + first.input->width * 0.5f;
                    float firstCenterY = first.y + first.input->height * 0.5f;
                    float secondCenterX = second.x + second.input->width * 0.5f;
                    float secondCenterY = second.y + second.input->height * 0.5f;
                    if (overlapX < overlapY) {
                        float direction = firstCenterX <= secondCenterX ? -1.0f : 1.0f;
                        float push = (overlapX + 1.0f) * 0.65f;
                        first.labelForceX += direction * push * firstWeight;
                        second.labelForceX -= direction * push * secondWeight;
                    } else {
                        float direction = firstCenterY <= secondCenterY ? -1.0f : 1.0f;
                        float push = (overlapY + 1.0f) * 0.65f;
                        first.labelForceY += direction * push * firstWeight;
                        second.labelForceY -= direction * push * secondWeight;
                    }
                }
            }
        }
        for (size_t i = 0; i < workCount; i++) {
            WorkItem &work = work_[i];
            work.forceX = work.aircraftForceX + work.labelForceX;
            work.forceY = work.aircraftForceY + work.labelForceY;
            const LabelLayoutInput &input = *work.input;

            float centerX = work.x + input.width * 0.5f;
            float centerY = work.y + input.height * 0.5f;
            float fromAnchorX = centerX - input.anchorX;
            float fromAnchorY = centerY - input.anchorY;
            float nearestX = clampFloat(input.anchorX, work.x, work.x + input.width);
            float nearestY = clampFloat(input.anchorY, work.y, work.y + input.height);
            float edgeX = nearestX - input.anchorX;
            float edgeY = nearestY - input.anchorY;
            float edgeDistance = approximateLength(edgeX, edgeY);
            float directionX = edgeX;
            float directionY = edgeY;
            float directionDistance = edgeDistance;
            float springForceX = 0.0f;
            float springForceY = 0.0f;
            if (directionDistance < 0.001f) {
                directionX = fromAnchorX;
                directionY = fromAnchorY;
                directionDistance = sqrtf(
                    directionX * directionX + directionY * directionY
                );
            }
            if (directionDistance > 0.001f) {
                float unitX = directionX / directionDistance;
                float unitY = directionY / directionDistance;
                float targetDistance = input.symbolRadius + kPreferredSymbolGapPx;
                float spring = (targetDistance - edgeDistance) * 0.28f;
                springForceX = unitX * spring;
                springForceY = unitY * spring;
                work.forceX += springForceX;
                work.forceY += springForceY;
            }

            if (isInsideForwardCone(
                    input,
                    centerX,
                    centerY,
                    work.forwardX,
                    work.forwardY)) {
                float side = fromAnchorX * work.rightX +
                    fromAnchorY * work.rightY;
                float sideSign = fabsf(side) > 0.01f
                    ? (side > 0.0f ? 1.0f : -1.0f)
                    : ((input.id & 1U) ? 1.0f : -1.0f);
                work.forceX += work.rightX * sideSign * 1.5f -
                    work.forwardX * 0.45f;
                work.forceY += work.rightY * sideSign * 1.5f -
                    work.forwardY * 0.45f;
            }

            for (size_t obstacleIndex = 0;
                 obstacleIndex < staticObstacleCount;
                 obstacleIndex++) {
                const LabelRectObstacle &obstacle = staticObstacles[obstacleIndex];
                float overlapX = minFloat(work.x + input.width, obstacle.x + obstacle.width) -
                    maxFloat(work.x, obstacle.x);
                float overlapY = minFloat(work.y + input.height, obstacle.y + obstacle.height) -
                    maxFloat(work.y, obstacle.y);
                if (overlapX <= 0.0f || overlapY <= 0.0f) continue;
                float obstacleCenterX = obstacle.x + obstacle.width * 0.5f;
                float obstacleCenterY = obstacle.y + obstacle.height * 0.5f;
                if (overlapX < overlapY) {
                    work.forceX += centerX < obstacleCenterX
                        ? -(overlapX + 1.0f)
                        : overlapX + 1.0f;
                } else {
                    work.forceY += centerY < obstacleCenterY
                        ? -(overlapY + 1.0f)
                        : overlapY + 1.0f;
                }
            }

            float targetDistance = input.symbolRadius + kPreferredSymbolGapPx;
            float excessDistance = edgeDistance - targetDistance;
            if (iteration % 3 == 0 &&
                excessDistance > kOrbitMinExcessPx &&
                (fromAnchorX != 0.0f || fromAnchorY != 0.0f)) {
                size_t orbitOffset = i >= orbitStart
                    ? i - orbitStart
                    : i + workCount - orbitStart;
                bool orbitScheduled = workCount <= kOrbitLabelsPerFrame ||
                    orbitOffset < kOrbitLabelsPerFrame || input.mustShow;
                if (!orbitScheduled) continue;
                float centerDistance = approximateLength(fromAnchorX, fromAnchorY);
                float radialX = fromAnchorX / centerDistance;
                float radialY = fromAnchorY / centerDistance;
                float avoidanceX = work.forceX - springForceX;
                float avoidanceY = work.forceY - springForceY;
                float outwardAvoidance = avoidanceX * radialX + avoidanceY * radialY;
                if (outwardAvoidance > 0.1f) {
                    float tangentX = -radialY;
                    float tangentY = radialX;
                    float tangentAvoidance = avoidanceX * tangentX +
                        avoidanceY * tangentY;
                    float orbitSign = fabsf(tangentAvoidance) >
                            outwardAvoidance * 0.2f + 0.1f
                        ? (tangentAvoidance > 0.0f ? 1.0f : -1.0f)
                        : ((input.id & 1U) ? 1.0f : -1.0f);
                    float orbitStrength = minFloat(
                        kOrbitForceLimitPx,
                        0.35f + excessDistance * 0.08f + outwardAvoidance * 0.15f
                    );
                    work.forceX += tangentX * orbitSign * orbitStrength;
                    work.forceY += tangentY * orbitSign * orbitStrength;
                }
            }
        }

        for (size_t i = 0; i < workCount; i++) {
            WorkItem &work = work_[i];
            float forceLength = approximateLength(work.forceX, work.forceY);
            if (forceLength > 6.0f) {
                work.forceX *= 6.0f / forceLength;
                work.forceY *= 6.0f / forceLength;
            }
            work.x += work.forceX;
            work.y += work.forceY;
            clampToBounds(
                work.x,
                work.y,
                work.input->width,
                work.input->height,
                bounds
            );

            float nearestX = clampFloat(
                work.input->anchorX,
                work.x,
                work.x + work.input->width
            );
            float nearestY = clampFloat(
                work.input->anchorY,
                work.y,
                work.y + work.input->height
            );
            float toAnchorX = work.input->anchorX - nearestX;
            float toAnchorY = work.input->anchorY - nearestY;
            float gapSquared = toAnchorX * toAnchorX + toAnchorY * toAnchorY;
            float maxGap = work.input->mustShow
                ? kPriorityMaxGapPx
                : kNormalMaxGapPx;
            if (gapSquared > maxGap * maxGap) {
                float gap = sqrtf(gapSquared);
                float correction = gap - maxGap;
                work.x += toAnchorX * correction / gap;
                work.y += toAnchorY * correction / gap;
                clampToBounds(
                    work.x,
                    work.y,
                    work.input->width,
                    work.input->height,
                    bounds
                );
            }
        }
    }

    if (workCount > kOrbitLabelsPerFrame) {
        orbitCursor_ = (orbitStart + kOrbitLabelsPerFrame) % workCount;
    } else {
        orbitCursor_ = 0;
    }

    deltaSeconds = clampFloat(deltaSeconds, 0.0f, 0.1f);
    float maxMovement = kMaxMovementPxPerSecond * deltaSeconds;
    for (size_t i = 0; i < workCount; i++) {
        WorkItem &work = work_[i];
        if (!work.isNew) {
            float dx = work.x - work.baseX;
            float dy = work.y - work.baseY;
            float distanceSquared = dx * dx + dy * dy;
            if (maxMovement <= 0.0f ||
                distanceSquared <= kMovementDeadZonePx * kMovementDeadZonePx) {
                work.x = work.baseX;
                work.y = work.baseY;
            } else if (distanceSquared > maxMovement * maxMovement) {
                float distance = approximateLength(dx, dy);
                work.x = work.baseX + dx * maxMovement / distance;
                work.y = work.baseY + dy * maxMovement / distance;
            }
        }
        clampToBounds(
            work.x,
            work.y,
            work.input->width,
            work.input->height,
            bounds
        );
        work.state->x = work.x;
        work.state->y = work.y;
    }

    size_t priorityOrder[kMaxLabels];
    for (size_t i = 0; i < workCount; i++) priorityOrder[i] = i;
    for (size_t i = 1; i < workCount; i++) {
        size_t value = priorityOrder[i];
        size_t j = i;
        while (j > 0) {
            const LabelLayoutInput &left = *work_[priorityOrder[j - 1]].input;
            const LabelLayoutInput &right = *work_[value].input;
            bool rightBeforeLeft = right.mustShow != left.mustShow
                ? right.mustShow
                : (right.distanceKm != left.distanceKm
                    ? right.distanceKm < left.distanceKm
                    : right.id < left.id);
            if (!rightBeforeLeft) break;
            priorityOrder[j] = priorityOrder[j - 1];
            j--;
        }
        priorityOrder[j] = value;
    }

    bool accepted[kMaxLabels] = {};
    float maxOverlap = 0;
    for (size_t priorityIndex = 0; priorityIndex < workCount; priorityIndex++) {
        size_t workIndex = priorityOrder[priorityIndex];
        WorkItem &work = work_[workIndex];
        State &state = *work.state;
        const LabelLayoutInput &input = *work.input;
        bool conflict = false;

        float maxGap = input.mustShow ? kPriorityMaxGapPx : kNormalMaxGapPx;
        if (pointToRectDistanceSquared(
                input.anchorX,
                input.anchorY,
                work.x,
                work.y,
                input.width,
                input.height) > maxGap * maxGap) {
            conflict = true;
        }

        for (size_t acceptedIndex = 0; acceptedIndex < workCount; acceptedIndex++) {
            if (!accepted[acceptedIndex]) continue;
            const WorkItem &other = work_[acceptedIndex];
            float overlap = rectOverlapDepth(
                work.x,
                work.y,
                input.width,
                input.height,
                other.x,
                other.y,
                other.input->width,
                other.input->height
            );
            if (overlap > maxOverlap) maxOverlap = overlap;
            if (overlap > 0.5f) conflict = true;
        }

        if (input.mustShow) {
            state.visible = true;
            state.conflictFrames = 0;
            state.cleanFrames = 0;
        } else if (conflict) {
            if (state.conflictFrames < 255) state.conflictFrames++;
            state.cleanFrames = 0;
            if (state.conflictFrames >= kHideAfterConflictFrames) {
                state.visible = false;
            }
        } else {
            state.conflictFrames = 0;
            if (state.cleanFrames < 255) state.cleanFrames++;
            if (!state.visible && state.cleanFrames >= kShowAfterCleanFrames) {
                state.visible = true;
            }
        }

        if (state.visible) accepted[workIndex] = true;
        LabelLayoutOutput &output = outputs[work.outputIndex];
        output.x = work.x;
        output.y = work.y;
        output.visible = state.visible;
    }

    if (metrics != nullptr) {
        metrics->maxOverlapPx = maxOverlap;
        for (size_t i = 0; i < workCount; i++) {
            if (work_[i].state->visible) {
                metrics->visibleCount++;
            } else {
                metrics->hiddenCount++;
            }
        }
    }
}

}  // namespace RadarLabels
