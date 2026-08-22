#include "battery_gauge.h"

#include <stdio.h>
#include <stdlib.h>

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        exit(1); \
    } \
} while (0)

using BatteryGauge::PowerStateTracker;
using BatteryGauge::percentFromVoltage;
using BatteryGauge::voltsFromAdcCounts;

static void feedSteady(PowerStateTracker &tracker, float volts, size_t samples) {
    for (size_t i = 0; i < samples; i++) {
        tracker.addSample(volts);
    }
}

int main() {
    CHECK(voltsFromAdcCounts(0) == 0.0f);
    float fullScale = voltsFromAdcCounts(1023);
    CHECK(fullScale > 9.89f && fullScale < 9.91f);

    CHECK(percentFromVoltage(4.30f) == 100);
    CHECK(percentFromVoltage(4.20f) == 100);
    CHECK(percentFromVoltage(3.30f) == 0);
    CHECK(percentFromVoltage(3.00f) == 0);
    CHECK(percentFromVoltage(4.00f) == 80);
    int mid = percentFromVoltage(3.96f);
    CHECK(mid > 68 && mid < 80);
    CHECK(percentFromVoltage(3.85f) == 56);

    {
        // Boot on external power.
        PowerStateTracker tracker;
        CHECK(!tracker.primed());
        CHECK(!tracker.onBattery());
        feedSteady(tracker, 4.18f, PowerStateTracker::WINDOW);
        CHECK(tracker.primed());
        CHECK(!tracker.onBattery());
        CHECK(tracker.displayPercent() >= 95);
    }

    {
        // Boot on battery.
        PowerStateTracker tracker;
        feedSteady(tracker, 3.85f, PowerStateTracker::WINDOW);
        CHECK(tracker.primed());
        CHECK(tracker.onBattery());
        int percent = tracker.displayPercent();
        CHECK(percent % 5 == 0);
        CHECK(percent > 56 && percent <= 75);
    }

    {
        // Unplug is detected by the downward step and holds through refill.
        PowerStateTracker tracker;
        feedSteady(tracker, 4.17f, PowerStateTracker::WINDOW);
        CHECK(!tracker.onBattery());
        tracker.addSample(4.05f);
        CHECK(tracker.onBattery());
        feedSteady(tracker, 4.06f, 3 * PowerStateTracker::WINDOW);
        CHECK(tracker.onBattery());
    }

    {
        // Plug-in is detected by the upward step even in the level dead zone,
        // and the level rule cannot flip back mid-refill.
        PowerStateTracker tracker;
        feedSteady(tracker, 3.80f, PowerStateTracker::WINDOW);
        CHECK(tracker.onBattery());
        tracker.addSample(3.95f);
        CHECK(!tracker.onBattery());
        feedSteady(tracker, 4.16f, PowerStateTracker::WINDOW);
        CHECK(!tracker.onBattery());
    }

    {
        // The rise from our own backlight dimming stays below the step
        // threshold and must not flip the state back.
        PowerStateTracker tracker;
        feedSteady(tracker, 4.17f, PowerStateTracker::WINDOW);
        tracker.addSample(4.05f);
        CHECK(tracker.onBattery());
        tracker.addSample(4.10f);
        CHECK(tracker.onBattery());
    }

    {
        // Dead-zone boot defaults to external, then the declining average
        // crosses the battery threshold.
        PowerStateTracker tracker;
        feedSteady(tracker, 4.05f, PowerStateTracker::WINDOW);
        CHECK(!tracker.onBattery());
        feedSteady(tracker, 4.00f, PowerStateTracker::WINDOW);
        CHECK(tracker.onBattery());
    }

    puts("battery_gauge tests passed");
    return 0;
}
