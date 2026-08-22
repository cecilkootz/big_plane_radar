#include "route_plausibility.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

using RadarRoute::corridorDistanceKm;
using RadarRoute::GeoPoint;
using RadarRoute::kRouteCorridorToleranceKm;
using RadarRoute::kRouteDirectionStrikeLimit;
using RadarRoute::kRouteRecoverySampleLimit;
using RadarRoute::routeDirectionIsPlausible;
using RadarRoute::routeIsPlausible;
using RadarRoute::RouteSample;
using RadarRoute::RouteVerdict;
using RadarRoute::updateRouteVerdict;

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        exit(1); \
    } \
} while (0)

#define CHECK_NEAR(actual, expected, tolerance) do { \
    float actualValue = (actual); \
    float expectedValue = (expected); \
    if (fabsf(actualValue - expectedValue) > (tolerance)) { \
        fprintf(stderr, "CHECK_NEAR failed at %s:%d: %s = %.3f, expected %.3f +/- %.3f\n", \
                __FILE__, __LINE__, #actual, actualValue, expectedValue, (float)(tolerance)); \
        exit(1); \
    } \
} while (0)

static constexpr GeoPoint kJfk{40.6398f, -73.7789f};
static constexpr GeoPoint kSan{32.7336f, -117.1900f};
static constexpr GeoPoint kSea{47.4490f, -122.3093f};
static constexpr GeoPoint kFll{26.0726f, -80.1527f};
static constexpr GeoPoint kDenver{39.8617f, -104.6731f};
static constexpr GeoPoint kBoston{42.3656f, -71.0096f};
static constexpr GeoPoint kAnchorage{61.1700f, -150.0000f};

// Live DAL338 position observed while it was descending into south Florida.
static constexpr GeoPoint kTampa{27.733658f, -82.389118f};

static void testPointOnCorridorIsClose() {
    CHECK_NEAR(corridorDistanceKm(kJfk, kSan, kJfk), 0.0f, 1.0f);
    CHECK_NEAR(corridorDistanceKm(kJfk, kSan, kSan), 0.0f, 1.0f);
}

// The reported defect: ADSBdb claims DAL338 is JFK->SAN while the aircraft is
// descending over Tampa Bay.
static void testStaleRouteIsRejected() {
    CHECK_NEAR(corridorDistanceKm(kJfk, kSan, kTampa), 1411.0f, 25.0f);
    CHECK(!routeIsPlausible(kJfk, kSan, kTampa, kRouteCorridorToleranceKm));
}

// The route the aircraft is actually flying must stay accepted.
static void testActualRouteIsAccepted() {
    CHECK_NEAR(corridorDistanceKm(kSea, kFll, kTampa), 20.0f, 15.0f);
    CHECK(routeIsPlausible(kSea, kFll, kTampa, kRouteCorridorToleranceKm));
}

// Real traffic does not fly exact great circles; ordinary routing deviation
// must not be mistaken for bad data.
static void testOrdinaryRoutingDeviationIsAccepted() {
    CHECK_NEAR(corridorDistanceKm(kJfk, kSan, kDenver), 312.0f, 20.0f);
    CHECK(routeIsPlausible(kJfk, kSan, kDenver, kRouteCorridorToleranceKm));
}

// Off the ends of the segment the distance clamps to the nearer endpoint, so a
// departure or an arrival hold stays plausible.
static void testDistanceClampsToEndpoints() {
    CHECK_NEAR(corridorDistanceKm(kJfk, kSan, kBoston), 300.0f, 20.0f);
    CHECK(routeIsPlausible(kJfk, kSan, kBoston, kRouteCorridorToleranceKm));

    CHECK_NEAR(corridorDistanceKm(kSea, kFll, kAnchorage), 2326.0f, 30.0f);
    CHECK(!routeIsPlausible(kSea, kFll, kAnchorage, kRouteCorridorToleranceKm));
}

// Round trips report the same airport twice; fall back to distance from it.
static void testDegenerateRouteMeasuresFromSinglePoint() {
    CHECK_NEAR(corridorDistanceKm(kJfk, kJfk, kTampa), 1637.0f, 25.0f);
    CHECK(!routeIsPlausible(kJfk, kJfk, kTampa, kRouteCorridorToleranceKm));
    CHECK(routeIsPlausible(kJfk, kJfk, kJfk, kRouteCorridorToleranceKm));
}

// Without usable coordinates the check must not run: suppressing a route we
// cannot verify would hide good data.
static void testUnknownCoordinatesStayPlausible() {
    CHECK(!RadarRoute::isUsableCoordinate(GeoPoint{0.0f, 0.0f}));
    CHECK(!RadarRoute::isUsableCoordinate(GeoPoint{91.0f, 10.0f}));
    CHECK(!RadarRoute::isUsableCoordinate(GeoPoint{10.0f, 181.0f}));
    CHECK(RadarRoute::isUsableCoordinate(kJfk));

    CHECK(routeIsPlausible(GeoPoint{0.0f, 0.0f}, kSan, kTampa, kRouteCorridorToleranceKm));
    CHECK(routeIsPlausible(kJfk, GeoPoint{0.0f, 0.0f}, kTampa, kRouteCorridorToleranceKm));
    CHECK(routeIsPlausible(kJfk, kSan, GeoPoint{0.0f, 0.0f}, kRouteCorridorToleranceKm));
}

// A non-positive tolerance disables the gate entirely.
static void testNonPositiveToleranceDisablesGate() {
    CHECK(routeIsPlausible(kJfk, kSan, kTampa, 0.0f));
    CHECK(routeIsPlausible(kJfk, kSan, kTampa, -1.0f));
}

static void testToleranceSeparatesRealFromStale() {
    // The tolerance must sit above ordinary deviation and below the observed
    // failure, otherwise the gate is either useless or destructive.
    CHECK(kRouteCorridorToleranceKm > corridorDistanceKm(kJfk, kSan, kDenver));
    CHECK(kRouteCorridorToleranceKm < corridorDistanceKm(kJfk, kSan, kTampa));
}

// Mid-country on the JFK->SAN corridor. Bearing to SAN is ~246 deg, to JFK
// ~66 deg, so the two legs are unambiguous.
static constexpr GeoPoint kKansas{38.5f, -98.0f};

static RouteSample enRouteSample(GeoPoint position, float trackDeg) {
    RouteSample sample;
    sample.position = position;
    sample.trackDeg = trackDeg;
    sample.gsKnots = 450.0f;
    sample.hasTrack = true;
    return sample;
}

// A route can pass the corridor check while the aircraft flies the opposite
// leg; the track exposes that.
static void testReversedLegContradictsTrack() {
    CHECK(routeDirectionIsPlausible(kJfk, kSan, kKansas, 250.0f));
    CHECK(routeDirectionIsPlausible(kJfk, kSan, kKansas, 180.0f));
    CHECK(!routeDirectionIsPlausible(kJfk, kSan, kKansas, 66.0f));
    CHECK(!routeDirectionIsPlausible(kJfk, kSan, kKansas, 90.0f));
}

// Terminal maneuvering points anywhere, so the track says nothing near the
// airports.
static void testDirectionSkippedNearEndpoints() {
    constexpr GeoPoint nearSan{33.3f, -116.2f};
    CHECK(corridorDistanceKm(kSan, kSan, nearSan) < 150.0f);
    CHECK(routeDirectionIsPlausible(kJfk, kSan, nearSan, 66.0f));
    constexpr GeoPoint nearJfk{40.9f, -74.5f};
    CHECK(routeDirectionIsPlausible(kJfk, kSan, nearJfk, 66.0f));
}

static void testCorridorBreachRejectsImmediately() {
    RouteVerdict verdict;
    CHECK(updateRouteVerdict(verdict, kJfk, kSan, enRouteSample(kTampa, 200.0f)));
    CHECK(verdict.rejected);
}

static void testContradictingTrackNeedsConsecutiveStrikes() {
    RouteVerdict verdict;
    for (uint8_t i = 1; i < kRouteDirectionStrikeLimit; i++) {
        CHECK(!updateRouteVerdict(verdict, kJfk, kSan, enRouteSample(kKansas, 66.0f)));
        CHECK(!verdict.rejected);
    }
    CHECK(updateRouteVerdict(verdict, kJfk, kSan, enRouteSample(kKansas, 66.0f)));
    CHECK(verdict.rejected);
}

// One toward-destination sample resets the strikes: turns and vectors are
// transient.
static void testTowardSampleResetsStrikes() {
    RouteVerdict verdict;
    for (uint8_t i = 1; i < kRouteDirectionStrikeLimit; i++) {
        updateRouteVerdict(verdict, kJfk, kSan, enRouteSample(kKansas, 66.0f));
    }
    CHECK(!updateRouteVerdict(verdict, kJfk, kSan, enRouteSample(kKansas, 250.0f)));
    for (uint8_t i = 1; i < kRouteDirectionStrikeLimit; i++) {
        CHECK(!updateRouteVerdict(verdict, kJfk, kSan, enRouteSample(kKansas, 66.0f)));
        CHECK(!verdict.rejected);
    }
}

// Slow or trackless samples carry no directional evidence.
static void testDirectionNeedsSpeedAndTrack() {
    RouteVerdict verdict;
    RouteSample slow = enRouteSample(kKansas, 66.0f);
    slow.gsKnots = 40.0f;
    RouteSample trackless = enRouteSample(kKansas, 66.0f);
    trackless.hasTrack = false;
    for (int i = 0; i < 5; i++) {
        CHECK(!updateRouteVerdict(verdict, kJfk, kSan, slow));
        CHECK(!updateRouteVerdict(verdict, kJfk, kSan, trackless));
    }
    CHECK(!verdict.rejected);
}

// A glitched position must not suppress a good route for the whole flight:
// consecutive samples well inside the corridor lift the rejection.
static void testRejectionRecoversOnConsistentGoodSamples() {
    RouteVerdict verdict;
    updateRouteVerdict(verdict, kJfk, kSan, enRouteSample(kTampa, 250.0f));
    CHECK(verdict.rejected);
    for (uint8_t i = 1; i < kRouteRecoverySampleLimit; i++) {
        CHECK(!updateRouteVerdict(verdict, kJfk, kSan, enRouteSample(kDenver, 250.0f)));
        CHECK(verdict.rejected);
    }
    CHECK(updateRouteVerdict(verdict, kJfk, kSan, enRouteSample(kDenver, 250.0f)));
    CHECK(!verdict.rejected);
}

// An unusable sample is not evidence in either direction.
static void testUnusableSampleLeavesVerdictAlone() {
    RouteVerdict verdict;
    updateRouteVerdict(verdict, kJfk, kSan, enRouteSample(kTampa, 250.0f));
    CHECK(verdict.rejected);
    CHECK(!updateRouteVerdict(verdict, kJfk, kSan, enRouteSample(GeoPoint{0.0f, 0.0f}, 250.0f)));
    CHECK(verdict.rejected);
    RouteVerdict fresh;
    CHECK(!updateRouteVerdict(fresh, GeoPoint{0.0f, 0.0f}, kSan, enRouteSample(kTampa, 250.0f)));
    CHECK(!fresh.rejected);
}

int main() {
    testPointOnCorridorIsClose();
    testStaleRouteIsRejected();
    testActualRouteIsAccepted();
    testOrdinaryRoutingDeviationIsAccepted();
    testDistanceClampsToEndpoints();
    testDegenerateRouteMeasuresFromSinglePoint();
    testUnknownCoordinatesStayPlausible();
    testNonPositiveToleranceDisablesGate();
    testToleranceSeparatesRealFromStale();
    testReversedLegContradictsTrack();
    testDirectionSkippedNearEndpoints();
    testCorridorBreachRejectsImmediately();
    testContradictingTrackNeedsConsecutiveStrikes();
    testTowardSampleResetsStrikes();
    testDirectionNeedsSpeedAndTrack();
    testRejectionRecoversOnConsistentGoodSamples();
    testUnusableSampleLeavesVerdictAlone();
    printf("route plausibility tests passed\n");
    return 0;
}
