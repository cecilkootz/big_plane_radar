#include "label_layout.h"

#include <math.h>
#include <new>
#include <stdio.h>
#include <stdlib.h>

using RadarLabels::AircraftObstacle;
using RadarLabels::LabelLayout;
using RadarLabels::LabelLayoutBounds;
using RadarLabels::LabelLayoutInput;
using RadarLabels::LabelLayoutMetrics;
using RadarLabels::LabelLayoutOutput;
using RadarLabels::LabelRectObstacle;

static size_t allocationCount = 0;

void *operator new(size_t size) {
    allocationCount++;
    void *memory = malloc(size);
    if (memory == nullptr) throw std::bad_alloc();
    return memory;
}

void operator delete(void *memory) noexcept {
    free(memory);
}

void operator delete(void *memory, size_t) noexcept {
    free(memory);
}

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        exit(1); \
    } \
} while (0)

static constexpr LabelLayoutBounds kBounds{4, 4, 508, 476};

static LabelLayoutInput makeInput(
    uint32_t id,
    float x,
    float y,
    float distanceKm = 10.0f
) {
    LabelLayoutInput input;
    input.id = id;
    input.anchorX = x;
    input.anchorY = y;
    input.width = 54;
    input.height = 25;
    input.symbolRadius = 10;
    input.courseDeg = 0;
    input.courseValid = true;
    input.distanceKm = distanceKm;
    return input;
}

static AircraftObstacle makeOwnerObstacle(const LabelLayoutInput &input) {
    AircraftObstacle obstacle;
    obstacle.x = input.anchorX;
    obstacle.y = input.anchorY;
    obstacle.radius = input.symbolRadius;
    return obstacle;
}

static void checkBounds(const LabelLayoutInput &input, const LabelLayoutOutput &output) {
    CHECK(output.x >= kBounds.left - 0.001f);
    CHECK(output.y >= kBounds.top - 0.001f);
    CHECK(output.x + input.width <= kBounds.right + 0.001f);
    CHECK(output.y + input.height <= kBounds.bottom + 0.001f);
}

static float labelGapFromSymbol(
    const LabelLayoutInput &input,
    const LabelLayoutOutput &output
) {
    float nearestX = fmaxf(output.x, fminf(input.anchorX, output.x + input.width));
    float nearestY = fmaxf(output.y, fminf(input.anchorY, output.y + input.height));
    return hypotf(nearestX - input.anchorX, nearestY - input.anchorY) -
        input.symbolRadius;
}

static bool insideForwardCone(
    const LabelLayoutInput &input,
    const LabelLayoutOutput &output
) {
    float centerX = output.x + input.width * 0.5f;
    float centerY = output.y + input.height * 0.5f;
    float dx = centerX - input.anchorX;
    float dy = centerY - input.anchorY;
    float distance = sqrtf(dx * dx + dy * dy);
    if (distance < 0.001f || distance > 80.0f) return false;
    float radians = input.courseDeg * 0.01745329251994329577f;
    float forwardX = sinf(radians);
    float forwardY = -cosf(radians);
    return (dx * forwardX + dy * forwardY) / distance >= 0.76604444f;
}

static bool labelsOverlap(
    const LabelLayoutInput &leftInput,
    const LabelLayoutOutput &leftOutput,
    const LabelLayoutInput &rightInput,
    const LabelLayoutOutput &rightOutput
) {
    return leftOutput.x < rightOutput.x + rightInput.width &&
        leftOutput.x + leftInput.width > rightOutput.x &&
        leftOutput.y < rightOutput.y + rightInput.height &&
        leftOutput.y + leftInput.height > rightOutput.y;
}

static void testSingleAircraftAndCourseCone() {
    LabelLayout layout;
    layout.reset();
    LabelLayoutInput input = makeInput(0xabc001, 260, 240);
    AircraftObstacle obstacle = makeOwnerObstacle(input);
    LabelLayoutOutput output;
    layout.solve(
        &input, 1, &obstacle, 1, nullptr, 0, kBounds,
        1000, 1, 1.0f / 30.0f, &output
    );

    CHECK(output.visible);
    checkBounds(input, output);
    CHECK(!insideForwardCone(input, output));
    CHECK(labelGapFromSymbol(input, output) >= 1.9f);
    CHECK(labelGapFromSymbol(input, output) <= 4.1f);
}

static void testBoundsAndResize() {
    LabelLayout layout;
    layout.reset();
    LabelLayoutInput inputs[4] = {
        makeInput(1, 5, 5),
        makeInput(2, 507, 5),
        makeInput(3, 5, 475),
        makeInput(4, 507, 475),
    };
    AircraftObstacle obstacles[4];
    LabelLayoutOutput outputs[4];
    for (size_t i = 0; i < 4; i++) obstacles[i] = makeOwnerObstacle(inputs[i]);
    layout.solve(inputs, 4, obstacles, 4, nullptr, 0, kBounds, 1000, 1, 0.033f, outputs);
    for (size_t i = 0; i < 4; i++) checkBounds(inputs[i], outputs[i]);

    LabelLayout resizeLayout;
    resizeLayout.reset();
    LabelLayoutInput resizeInput = makeInput(50, 200, 200);
    AircraftObstacle resizeObstacle = makeOwnerObstacle(resizeInput);
    LabelLayoutOutput resizeOutput;
    resizeLayout.solve(
        &resizeInput, 1, &resizeObstacle, 1, nullptr, 0, kBounds,
        1000, 1, 0.033f, &resizeOutput
    );
    float oldCenter = resizeOutput.x + resizeInput.width * 0.5f;
    resizeInput.width = 92;
    resizeLayout.solve(
        &resizeInput, 1, &resizeObstacle, 1, nullptr, 0, kBounds,
        1033, 1, 0.033f, &resizeOutput
    );
    checkBounds(resizeInput, resizeOutput);
    float newCenter = resizeOutput.x + resizeInput.width * 0.5f;
    CHECK(fabsf(newCenter - oldCenter) < 4.0f);
}

static void testInputOrderDoesNotChangeLayout() {
    LabelLayout first;
    LabelLayout second;
    first.reset();
    second.reset();
    LabelLayoutInput normal[3] = {
        makeInput(30, 250, 235, 30),
        makeInput(10, 260, 240, 10),
        makeInput(20, 270, 245, 20),
    };
    LabelLayoutInput reversed[3] = {normal[2], normal[1], normal[0]};
    AircraftObstacle obstacles[3] = {
        makeOwnerObstacle(normal[0]),
        makeOwnerObstacle(normal[1]),
        makeOwnerObstacle(normal[2]),
    };
    LabelLayoutOutput normalOut[3];
    LabelLayoutOutput reversedOut[3];
    for (uint32_t frame = 0; frame < 12; frame++) {
        for (size_t i = 0; i < 3; i++) {
            normal[i].anchorX += 0.25f;
            reversed[2 - i] = normal[i];
            obstacles[i] = makeOwnerObstacle(normal[i]);
        }
        first.solve(
            normal, 3, obstacles, 3, nullptr, 0, kBounds,
            1000 + frame * 33, 1, 0.033f, normalOut
        );
        second.solve(
            reversed, 3, obstacles, 3, nullptr, 0, kBounds,
            1000 + frame * 33, 1, 0.033f, reversedOut
        );

        for (size_t i = 0; i < 3; i++) {
            size_t reversedIndex = 0;
            while (reversed[reversedIndex].id != normal[i].id) reversedIndex++;
            CHECK(fabsf(normalOut[i].x - reversedOut[reversedIndex].x) < 0.001f);
            CHECK(fabsf(normalOut[i].y - reversedOut[reversedIndex].y) < 0.001f);
            CHECK(normalOut[i].visible == reversedOut[reversedIndex].visible);
        }
    }
}

static void testStaticObstacleAndLayoutRevision() {
    LabelLayout layout;
    layout.reset();
    LabelLayoutInput input = makeInput(88, 260, 240);
    AircraftObstacle aircraftObstacle = makeOwnerObstacle(input);
    LabelRectObstacle staticObstacle{286, 190, 100, 100};
    LabelLayoutOutput output;
    layout.solve(
        &input, 1, &aircraftObstacle, 1, &staticObstacle, 1, kBounds,
        1000, 1, 0.033f, &output
    );
    CHECK(output.x + input.width <= staticObstacle.x ||
          output.x >= staticObstacle.x + staticObstacle.width ||
          output.y + input.height <= staticObstacle.y ||
          output.y >= staticObstacle.y + staticObstacle.height);

    input.courseDeg = 180;
    layout.solve(
        &input, 1, &aircraftObstacle, 1, nullptr, 0, kBounds,
        1033, 2, 0.033f, &output
    );
    CHECK(output.x + input.width * 0.5f < input.anchorX);
}

static void testAnchorMovementAndTemporaryDisappearance() {
    LabelLayout layout;
    layout.reset();
    LabelLayoutInput input = makeInput(77, 200, 200);
    AircraftObstacle obstacle = makeOwnerObstacle(input);
    LabelLayoutOutput output;
    layout.solve(&input, 1, &obstacle, 1, nullptr, 0, kBounds, 1000, 1, 0.033f, &output);
    float firstX = output.x;
    float firstY = output.y;

    input.anchorX += 5;
    obstacle = makeOwnerObstacle(input);
    layout.solve(&input, 1, &obstacle, 1, nullptr, 0, kBounds, 1033, 1, 0.033f, &output);
    CHECK(fabsf((output.x - firstX) - 5.0f) <= 2.2f);

    LabelLayoutOutput unused;
    layout.solve(nullptr, 0, nullptr, 0, nullptr, 0, kBounds, 2000, 1, 0.033f, &unused);
    float beforeMissingX = output.x;
    float beforeMissingY = output.y;
    layout.solve(&input, 1, &obstacle, 1, nullptr, 0, kBounds, 3000, 1, 0.033f, &output);
    CHECK(hypotf(output.x - beforeMissingX, output.y - beforeMissingY) <= 2.2f);
    CHECK(fabsf(output.y - firstY) < 5.0f);
}

static void testOrbitSearchKeepsBlockedLabelClose() {
    LabelLayout layout;
    layout.reset();
    LabelLayoutInput input = makeInput(0xabc123, 260, 240);
    AircraftObstacle obstacle = makeOwnerObstacle(input);
    LabelLayoutOutput output;
    layout.solve(
        &input, 1, &obstacle, 1, nullptr, 0, kBounds,
        1000, 1, 0.033f, &output
    );

    LabelRectObstacle blocker{264, 220, 28, 40};
    for (uint32_t frame = 1; frame <= 90; frame++) {
        layout.solve(
            &input, 1, &obstacle, 1, &blocker, 1, kBounds,
            1000 + frame * 33, 1, 0.033f, &output
        );
    }

    float centerY = output.y + input.height * 0.5f;
    CHECK(fabsf(centerY - input.anchorY) > 5.0f);
    CHECK(labelGapFromSymbol(input, output) < 24.0f);
    CHECK(output.visible);
}

static void testCrossingLabelsMayOverlapWithoutOrbiting() {
    LabelLayout layout;
    layout.reset();
    LabelLayoutInput inputs[2] = {
        makeInput(10, 170, 240),
        makeInput(20, 330, 240),
    };
    inputs[0].courseValid = false;
    inputs[1].courseValid = false;
    LabelLayoutOutput outputs[2];
    layout.solve(
        inputs, 2, nullptr, 0, nullptr, 0, kBounds,
        1000, 1, 0.033f, outputs
    );
    float relativeX[2];
    float relativeY[2];
    for (size_t i = 0; i < 2; i++) {
        relativeX[i] = outputs[i].x - inputs[i].anchorX;
        relativeY[i] = outputs[i].y - inputs[i].anchorY;
    }

    for (uint32_t frame = 1; frame <= 80; frame++) {
        inputs[0].anchorX += 0.75f;
        inputs[1].anchorX -= 0.75f;
        layout.solve(
            inputs, 2, nullptr, 0, nullptr, 0, kBounds,
            1000 + frame * 33, 1, 0.033f, outputs
        );
        for (size_t i = 0; i < 2; i++) {
            CHECK(fabsf(outputs[i].x - inputs[i].anchorX - relativeX[i]) < 0.25f);
            CHECK(fabsf(outputs[i].y - inputs[i].anchorY - relativeY[i]) < 0.25f);
            CHECK(outputs[i].visible);
        }
    }

    CHECK(labelsOverlap(inputs[0], outputs[0], inputs[1], outputs[1]));
}

static void testConvergingLabelsStayAnchoredWhenOverlapping() {
    LabelLayout layout;
    layout.reset();
    LabelLayoutInput inputs[3] = {
        makeInput(10, 260, 240),
        makeInput(20, 180, 240),
        makeInput(30, 340, 240),
    };
    for (LabelLayoutInput &input : inputs) input.courseValid = false;
    LabelLayoutOutput outputs[3];
    layout.solve(
        inputs, 3, nullptr, 0, nullptr, 0, kBounds,
        1000, 1, 0.033f, outputs
    );
    float relativeX[3];
    float relativeY[3];
    for (size_t i = 0; i < 3; i++) {
        relativeX[i] = outputs[i].x - inputs[i].anchorX;
        relativeY[i] = outputs[i].y - inputs[i].anchorY;
    }

    inputs[1].anchorX = 260;
    inputs[2].anchorX = 260;
    LabelLayoutMetrics metrics;
    layout.solve(
        inputs, 3, nullptr, 0, nullptr, 0, kBounds,
        1033, 1, 0.033f, outputs, &metrics
    );
    for (uint32_t frame = 2; frame <= 180; frame++) {
        layout.solve(
            inputs, 3, nullptr, 0, nullptr, 0, kBounds,
            1000 + frame * 33, 1, 0.033f, outputs, &metrics
        );
        for (size_t i = 0; i < 3; i++) {
            CHECK(fabsf(outputs[i].x - inputs[i].anchorX - relativeX[i]) < 0.25f);
            CHECK(fabsf(outputs[i].y - inputs[i].anchorY - relativeY[i]) < 0.25f);
            CHECK(outputs[i].visible);
        }
    }

    CHECK(labelsOverlap(inputs[0], outputs[0], inputs[1], outputs[1]));
    CHECK(labelsOverlap(inputs[0], outputs[0], inputs[2], outputs[2]));
    CHECK(labelsOverlap(inputs[1], outputs[1], inputs[2], outputs[2]));
    CHECK(metrics.visibleCount == 3);
    CHECK(metrics.hiddenCount == 0);
    CHECK(metrics.maxOverlapPx > 0.0f);
}

static void testRadialBlockerDoesNotPumpTheLabel() {
    LabelLayout layout;
    layout.reset();
    LabelLayoutInput input = makeInput(0xabc777, 260, 240);
    input.courseValid = false;
    LabelLayoutOutput output;
    layout.solve(
        &input, 1, nullptr, 0, nullptr, 0, kBounds,
        1000, 1, 0.033f, &output
    );

    LabelRectObstacle blocker{286, 180, 10, 120};
    float previousGap = labelGapFromSymbol(input, output);
    int radialDirection = 0;
    size_t radialDirectionChanges = 0;
    for (uint32_t frame = 1; frame <= 180; frame++) {
        layout.solve(
            &input, 1, nullptr, 0, &blocker, 1, kBounds,
            1000 + frame * 33, 1, 0.033f, &output
        );
        float gap = labelGapFromSymbol(input, output);
        float delta = gap - previousGap;
        if (frame > 10 && fabsf(delta) > 0.1f) {
            int direction = delta > 0.0f ? 1 : -1;
            if (radialDirection != 0 && direction != radialDirection) {
                radialDirectionChanges++;
            }
            radialDirection = direction;
        }
        previousGap = gap;
    }

    CHECK(radialDirectionChanges <= 1);
    CHECK(previousGap > 5.0f);

    float previousReturnGap = previousGap;
    for (uint32_t frame = 181; frame <= 360; frame++) {
        layout.solve(
            &input, 1, nullptr, 0, nullptr, 0, kBounds,
            1000 + frame * 33, 1, 0.033f, &output
        );
        float gap = labelGapFromSymbol(input, output);
        CHECK(gap <= previousReturnGap + 0.1f);
        previousReturnGap = gap;
    }
    CHECK(previousReturnGap <= 4.2f);
    CHECK(output.visible);
}

static void testAircraftBlockingReservedTargetDoesNotBounce() {
    LabelLayout layout;
    layout.reset();
    LabelLayoutInput input = makeInput(7000, 260, 240);
    input.courseValid = false;
    AircraftObstacle obstacles[2] = {
        makeOwnerObstacle(input),
        {275.0f, 240.0f, 10.0f},
    };
    LabelLayoutOutput output;
    layout.solve(
        &input, 1, obstacles, 1, nullptr, 0, kBounds,
        1000, 1, 0.033f, &output
    );

    float previousX = output.x;
    float previousY = output.y;
    int previousDirectionX = 0;
    int previousDirectionY = 0;
    size_t reversals = 0;
    for (uint32_t frame = 1; frame <= 180; frame++) {
        layout.solve(
            &input, 1, obstacles, 2, nullptr, 0, kBounds,
            1000 + frame * 33, 1, 0.033f, &output
        );
        float deltaX = output.x - previousX;
        float deltaY = output.y - previousY;
        int directionX = fabsf(deltaX) > 0.35f ? (deltaX > 0.0f ? 1 : -1) : 0;
        int directionY = fabsf(deltaY) > 0.35f ? (deltaY > 0.0f ? 1 : -1) : 0;
        if (directionX != 0 && previousDirectionX != 0 &&
            directionX != previousDirectionX) reversals++;
        if (directionY != 0 && previousDirectionY != 0 &&
            directionY != previousDirectionY) reversals++;
        if (directionX != 0) previousDirectionX = directionX;
        if (directionY != 0) previousDirectionY = directionY;
        previousX = output.x;
        previousY = output.y;
    }

    float nearestX = fmaxf(output.x, fminf(obstacles[1].x, output.x + input.width));
    float nearestY = fmaxf(output.y, fminf(obstacles[1].y, output.y + input.height));
    CHECK(reversals <= 1);
    CHECK(hypotf(nearestX - obstacles[1].x, nearestY - obstacles[1].y) >= 11.9f);
    CHECK(output.visible);
}

static void runDenseScene(size_t count) {
    LabelLayout layout;
    layout.reset();
    LabelLayoutInput inputs[64];
    AircraftObstacle obstacles[64];
    LabelLayoutOutput outputs[64];
    LabelLayoutMetrics metrics;
    for (size_t i = 0; i < count; i++) {
        inputs[i] = makeInput(
            static_cast<uint32_t>(1000 + i),
            250.0f + static_cast<float>(i % 4) * 3.0f,
            230.0f + static_cast<float>((i / 4) % 4) * 3.0f,
            static_cast<float>(i + 1)
        );
        inputs[i].mustShow = i < 2;
        obstacles[i] = makeOwnerObstacle(inputs[i]);
    }

    for (uint32_t frame = 0; frame < 30; frame++) {
        layout.solve(
            inputs, count, obstacles, count, nullptr, 0, kBounds,
            1000 + frame * 33, 1, 0.033f, outputs, &metrics
        );
        for (size_t i = 0; i < count; i++) checkBounds(inputs[i], outputs[i]);
        CHECK(outputs[0].visible);
        CHECK(outputs[1].visible);
    }
    CHECK(metrics.hiddenCount == 0);
    CHECK(metrics.maxOverlapPx > 0.0f);
    CHECK(metrics.visibleCount + metrics.hiddenCount == count);
}

static void testDenseSceneAndMandatoryLabels() {
    runDenseScene(32);
    runDenseScene(64);
}

static void testNoHeapAllocationDuringSolve() {
    LabelLayout layout;
    layout.reset();
    LabelLayoutInput input = makeInput(900, 260, 240);
    AircraftObstacle obstacle = makeOwnerObstacle(input);
    LabelLayoutOutput output;
    size_t before = allocationCount;
    layout.solve(&input, 1, &obstacle, 1, nullptr, 0, kBounds, 1000, 1, 0.033f, &output);
    CHECK(allocationCount == before);
}

int main() {
    testSingleAircraftAndCourseCone();
    testBoundsAndResize();
    testInputOrderDoesNotChangeLayout();
    testStaticObstacleAndLayoutRevision();
    testAnchorMovementAndTemporaryDisappearance();
    testOrbitSearchKeepsBlockedLabelClose();
    testCrossingLabelsMayOverlapWithoutOrbiting();
    testConvergingLabelsStayAnchoredWhenOverlapping();
    testRadialBlockerDoesNotPumpTheLabel();
    testAircraftBlockingReservedTargetDoesNotBounce();
    testDenseSceneAndMandatoryLabels();
    testNoHeapAllocationDuringSolve();
    puts("label_layout tests passed");
    return 0;
}
