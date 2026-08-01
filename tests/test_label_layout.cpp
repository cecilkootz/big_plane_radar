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
    CHECK(metrics.hiddenCount > 0);
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
    testDenseSceneAndMandatoryLabels();
    testNoHeapAllocationDuringSolve();
    puts("label_layout tests passed");
    return 0;
}
