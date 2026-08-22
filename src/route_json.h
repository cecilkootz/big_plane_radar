#pragma once

#include <ArduinoJson.h>
#include <ctype.h>

#include <initializer_list>

#include "airport_lookup.h"
#include "route_plausibility.h"

namespace RadarRoute {

// Keys mirror the ADSBdb callsign response. Coordinates and airport codes are
// requested so the route can be checked against the aircraft position;
// dropping them here silently disables that check, so the filter is covered by
// a native test.
inline void buildFlightRouteFilter(JsonDocument &filter) {
    for (const char *end : {"origin", "destination"}) {
        JsonObject airport = filter["response"]["flightroute"][end].to<JsonObject>();
        airport["icao_code"] = true;
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

inline bool readAirportCode(
    const JsonObject &airport,
    const char *key,
    size_t codeLen,
    char *out,
    size_t outLen
) {
    if (outLen <= codeLen) return false;
    out[0] = '\0';
    if (!airport[key].is<const char *>()) return false;
    const char *value = airport[key].as<const char *>();
    size_t len = 0;
    for (size_t i = 0; value[i] != '\0' && len < codeLen; i++) {
        unsigned char c = static_cast<unsigned char>(value[i]);
        if (!isalnum(c)) continue;
        out[len++] = static_cast<char>(toupper(c));
    }
    out[len] = '\0';
    return len == codeLen;
}

// ADSBdb airport rows sometimes omit or misstate coordinates. The compiled-in
// OurAirports catalog is authoritative when it knows the airport; the response
// coordinates only cover airports outside the catalog.
inline GeoPoint resolveAirportCoordinate(const JsonObject &airport) {
    char code[8];
    const AirportCatalogEntry *entry = nullptr;
    if (readAirportCode(airport, "icao_code", 4, code, sizeof(code))) {
        entry = RadarAirports::findByIcao(code);
    }
    if (entry == nullptr) {
        for (const char *key : {"iata_code", "iata", "iataCode"}) {
            if (readAirportCode(airport, key, 3, code, sizeof(code))) {
                entry = RadarAirports::findByIata(code);
                if (entry != nullptr) break;
            }
        }
    }
    if (entry != nullptr) {
        GeoPoint point;
        point.lat = RadarAirports::degreesFromE5(entry->latE5);
        point.lon = RadarAirports::degreesFromE5(entry->lonE5);
        return point;
    }
    return readAirportCoordinate(airport);
}

}  // namespace RadarRoute
