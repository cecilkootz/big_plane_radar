#pragma once

#include <ArduinoJson.h>

#include <initializer_list>

#include "route_plausibility.h"

namespace RadarRoute {

// Keys mirror the ADSBdb callsign response. Coordinates are requested so the
// route can be checked against the aircraft position; dropping them here
// silently disables that check, so the filter is covered by a native test.
inline void buildFlightRouteFilter(JsonDocument &filter) {
    for (const char *end : {"origin", "destination"}) {
        JsonObject airport = filter["response"]["flightroute"][end].to<JsonObject>();
        airport["iata_code"] = true;
        airport["iata"] = true;
        airport["iataCode"] = true;
        airport["municipality"] = true;
        airport["name"] = true;
        airport["latitude"] = true;
        airport["longitude"] = true;
    }
}

inline GeoPoint readAirportCoordinate(const JsonObject &airport) {
    GeoPoint point;
    if (airport["latitude"].is<float>() && airport["longitude"].is<float>()) {
        point.lat = airport["latitude"].as<float>();
        point.lon = airport["longitude"].as<float>();
    }
    return point;
}

}  // namespace RadarRoute
