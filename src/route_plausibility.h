#pragma once

#include <math.h>

namespace RadarRoute {

// Callsign route lookups resolve against a static schedule table, so a recycled
// flight number can return a route the aircraft is not flying. Comparing the
// live position against the origin-destination corridor rejects those.
static constexpr float kRouteCorridorToleranceKm = 500.0f;

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

}  // namespace RadarRoute
