#pragma once

#include <math.h>
#include <stdint.h>

namespace RadarRoute {

// Callsign route lookups resolve against a static schedule table, so a recycled
// flight number can return a route the aircraft is not flying. Comparing the
// live position against the origin-destination corridor rejects those.
static constexpr float kRouteCorridorToleranceKm = 500.0f;

// A rejected route recovers only well inside the corridor, so a position
// oscillating around the tolerance cannot flap the verdict.
static constexpr float kRouteRecoveryMarginKm = 100.0f;

// The track direction says nothing near the airports, where terminal
// maneuvering points anywhere, or at low speed.
static constexpr float kRouteEndpointSlackKm = 150.0f;
static constexpr float kRouteMinDirectionSpeedKnots = 80.0f;

// cos(120 deg): beyond this the aircraft is flying away from its claimed
// destination. En-route weather deviation stays inside 90 deg; a reversed or
// wrong leg measures near 180 deg.
static constexpr float kRouteAwayCosineLimit = -0.5f;

// Turns and ATC vectors are transient, so a contradicting track must persist
// across consecutive samples; recovery needs the same persistence.
static constexpr uint8_t kRouteDirectionStrikeLimit = 3;
static constexpr uint8_t kRouteRecoverySampleLimit = 3;

static constexpr float kEarthRadiusKm = 6371.0088f;

struct GeoPoint {
    float lat = 0;
    float lon = 0;
};

inline bool isUsableCoordinate(GeoPoint point) {
    if (isnan(point.lat) || isnan(point.lon)) return false;
    if (point.lat < -90.0f || point.lat > 90.0f) return false;
    if (point.lon < -180.0f || point.lon > 180.0f) return false;
    // Missing coordinates arrive as zero rather than as absent fields.
    return point.lat != 0.0f || point.lon != 0.0f;
}

namespace detail {

inline float toRadians(float degrees) {
    return degrees * 0.017453292519943295f;
}

inline float clampUnit(float value) {
    if (isnan(value)) return 0.0f;
    if (value > 1.0f) return 1.0f;
    if (value < -1.0f) return -1.0f;
    return value;
}

// Angular separation in radians.
inline float angularDistance(GeoPoint a, GeoPoint b) {
    float lat1 = toRadians(a.lat);
    float lat2 = toRadians(b.lat);
    float halfDLat = toRadians(b.lat - a.lat) * 0.5f;
    float halfDLon = toRadians(b.lon - a.lon) * 0.5f;
    float sinHalfDLat = sinf(halfDLat);
    float sinHalfDLon = sinf(halfDLon);
    float h = sinHalfDLat * sinHalfDLat +
              cosf(lat1) * cosf(lat2) * sinHalfDLon * sinHalfDLon;
    return 2.0f * asinf(clampUnit(sqrtf(h)));
}

inline float initialBearing(GeoPoint from, GeoPoint to) {
    float lat1 = toRadians(from.lat);
    float lat2 = toRadians(to.lat);
    float dLon = toRadians(to.lon - from.lon);
    float y = sinf(dLon) * cosf(lat2);
    float x = cosf(lat1) * sinf(lat2) - sinf(lat1) * cosf(lat2) * cosf(dLon);
    return atan2f(y, x);
}

}  // namespace detail

// Distance from `point` to the great-circle segment origin->destination,
// clamped to the endpoints so positions short of the origin or past the
// destination measure to that airport rather than to the extended track.
inline float corridorDistanceKm(
    GeoPoint origin,
    GeoPoint destination,
    GeoPoint point
) {
    float routeAngle = detail::angularDistance(origin, destination);
    float originAngle = detail::angularDistance(origin, point);
    if (routeAngle < 1e-7f) {
        return originAngle * kEarthRadiusKm;
    }

    float deltaBearing = detail::initialBearing(origin, point) -
                         detail::initialBearing(origin, destination);
    if (cosf(deltaBearing) < 0.0f) {
        return originAngle * kEarthRadiusKm;
    }

    float crossTrack = asinf(
        detail::clampUnit(sinf(originAngle) * sinf(deltaBearing))
    );
    float cosCrossTrack = cosf(crossTrack);
    if (fabsf(cosCrossTrack) < 1e-7f) {
        return fabsf(crossTrack) * kEarthRadiusKm;
    }

    float alongTrack = acosf(
        detail::clampUnit(cosf(originAngle) / cosCrossTrack)
    );
    if (alongTrack > routeAngle) {
        return detail::angularDistance(destination, point) * kEarthRadiusKm;
    }
    return fabsf(crossTrack) * kEarthRadiusKm;
}

// Reject a route only when the corridor is measurable and the aircraft is
// clearly not on it. Unknown coordinates and a non-positive tolerance leave the
// route untouched, so missing data never hides a good route.
inline bool routeIsPlausible(
    GeoPoint origin,
    GeoPoint destination,
    GeoPoint point,
    float toleranceKm
) {
    if (toleranceKm <= 0.0f) return true;
    if (!isUsableCoordinate(origin)) return true;
    if (!isUsableCoordinate(destination)) return true;
    if (!isUsableCoordinate(point)) return true;
    return corridorDistanceKm(origin, destination, point) <= toleranceKm;
}

// True when the track does not contradict the claimed destination. A route can
// pass the corridor check while the aircraft flies the opposite leg; the track
// exposes that. Not applicable near either airport.
inline bool routeDirectionIsPlausible(
    GeoPoint origin,
    GeoPoint destination,
    GeoPoint point,
    float trackDeg
) {
    if (!isUsableCoordinate(origin)) return true;
    if (!isUsableCoordinate(destination)) return true;
    if (!isUsableCoordinate(point)) return true;
    if (detail::angularDistance(origin, point) * kEarthRadiusKm <
        kRouteEndpointSlackKm) {
        return true;
    }
    if (detail::angularDistance(destination, point) * kEarthRadiusKm <
        kRouteEndpointSlackKm) {
        return true;
    }
    float away = detail::toRadians(trackDeg) -
                 detail::initialBearing(point, destination);
    return cosf(away) > kRouteAwayCosineLimit;
}

struct RouteSample {
    GeoPoint position;
    float trackDeg = 0;
    float gsKnots = 0;
    bool hasTrack = false;
};

struct RouteVerdict {
    uint8_t awayStrikes = 0;
    uint8_t recoveryStreak = 0;
    bool rejected = false;
};

// Accumulates evidence across position samples. A corridor breach rejects
// immediately; a contradicting track needs kRouteDirectionStrikeLimit
// consecutive samples; kRouteRecoverySampleLimit consecutive good samples well
// inside the corridor lift a rejection, so one glitched position cannot
// suppress a good route for the whole flight. Returns true when the rejected
// flag changed.
inline bool updateRouteVerdict(
    RouteVerdict &verdict,
    GeoPoint origin,
    GeoPoint destination,
    const RouteSample &sample
) {
    if (!isUsableCoordinate(origin)) return false;
    if (!isUsableCoordinate(destination)) return false;
    if (!isUsableCoordinate(sample.position)) return false;

    bool wasRejected = verdict.rejected;
    float corridorKm = corridorDistanceKm(origin, destination, sample.position);
    if (corridorKm > kRouteCorridorToleranceKm) {
        verdict.rejected = true;
        verdict.awayStrikes = 0;
        verdict.recoveryStreak = 0;
        return !wasRejected;
    }

    bool directionOk =
        !sample.hasTrack ||
        sample.gsKnots < kRouteMinDirectionSpeedKnots ||
        routeDirectionIsPlausible(origin, destination, sample.position, sample.trackDeg);
    if (!directionOk) {
        verdict.recoveryStreak = 0;
        if (verdict.awayStrikes < kRouteDirectionStrikeLimit) {
            verdict.awayStrikes++;
        }
        if (verdict.awayStrikes >= kRouteDirectionStrikeLimit) {
            verdict.rejected = true;
        }
    } else {
        verdict.awayStrikes = 0;
        if (verdict.rejected &&
            corridorKm <= kRouteCorridorToleranceKm - kRouteRecoveryMarginKm) {
            verdict.recoveryStreak++;
            if (verdict.recoveryStreak >= kRouteRecoverySampleLimit) {
                verdict.rejected = false;
                verdict.recoveryStreak = 0;
            }
        }
    }
    return verdict.rejected != wasRejected;
}

}  // namespace RadarRoute
