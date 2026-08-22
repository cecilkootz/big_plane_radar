#include "route_json.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using RadarRoute::buildFlightRouteFilter;
using RadarRoute::GeoPoint;
using RadarRoute::kRouteCorridorToleranceKm;
using RadarRoute::readAirportCoordinate;
using RadarRoute::resolveAirportCoordinate;
using RadarRoute::routeIsPlausible;

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        exit(1); \
    } \
} while (0)

// Verbatim ADSBdb response for DAL338, captured while the aircraft was actually
// descending into south Florida.
static const char kDal338Response[] =
    "{\"response\":{\"flightroute\":{\"callsign\":\"DAL338\",\"callsign_icao\":\"DAL338\","
    "\"callsign_iata\":\"DL338\",\"airline\":{\"name\":\"Delta Air Lines\",\"icao\":\"DAL\","
    "\"iata\":\"DL\",\"country\":\"United States\",\"country_iso\":\"US\",\"callsign\":\"DELTA\"},"
    "\"origin\":{\"country_iso_name\":\"US\",\"country_name\":\"United States\",\"elevation\":13,"
    "\"iata_code\":\"JFK\",\"icao_code\":\"KJFK\",\"latitude\":40.639801,\"longitude\":-73.7789,"
    "\"municipality\":\"New York\",\"name\":\"John F Kennedy International Airport\"},"
    "\"destination\":{\"country_iso_name\":\"US\",\"country_name\":\"United States\","
    "\"elevation\":17,\"iata_code\":\"SAN\",\"icao_code\":\"KSAN\",\"latitude\":32.7336006165,"
    "\"longitude\":-117.190002441,\"municipality\":\"San Diego\","
    "\"name\":\"San Diego International Airport\"}}}}";

// Live position of DAL338 from the same feed the firmware polls.
static constexpr GeoPoint kLivePosition{27.733658f, -82.389118f};

static JsonObject parseEndpoint(JsonDocument &doc, const char *endpoint) {
    JsonDocument filter;
    buildFlightRouteFilter(filter);
    DeserializationError err = deserializeJson(
        doc,
        kDal338Response,
        DeserializationOption::Filter(filter)
    );
    CHECK(!err);
    return doc["response"]["flightroute"][endpoint].as<JsonObject>();
}

// A filter that drops the coordinates would disable the corridor check without
// any other visible symptom.
static void testFilterKeepsCoordinates() {
    JsonDocument originDoc;
    JsonObject origin = parseEndpoint(originDoc, "origin");
    CHECK(!origin.isNull());
    CHECK(strcmp(origin["iata_code"].as<const char *>(), "JFK") == 0);
    CHECK(strcmp(origin["municipality"].as<const char *>(), "New York") == 0);

    GeoPoint originPoint = readAirportCoordinate(origin);
    CHECK(RadarRoute::isUsableCoordinate(originPoint));
    CHECK(fabsf(originPoint.lat - 40.6398f) < 0.01f);
    CHECK(fabsf(originPoint.lon - (-73.7789f)) < 0.01f);

    JsonDocument destinationDoc;
    JsonObject destination = parseEndpoint(destinationDoc, "destination");
    GeoPoint destinationPoint = readAirportCoordinate(destination);
    CHECK(RadarRoute::isUsableCoordinate(destinationPoint));
    CHECK(fabsf(destinationPoint.lat - 32.7336f) < 0.01f);
    CHECK(fabsf(destinationPoint.lon - (-117.1900f)) < 0.01f);
}

// The whole point: this exact payload plus this exact position must be refused.
static void testReportedDefectIsRejectedEndToEnd() {
    JsonDocument originDoc;
    JsonDocument destinationDoc;
    GeoPoint origin = readAirportCoordinate(parseEndpoint(originDoc, "origin"));
    GeoPoint destination =
        readAirportCoordinate(parseEndpoint(destinationDoc, "destination"));

    CHECK(!routeIsPlausible(origin, destination, kLivePosition, kRouteCorridorToleranceKm));
}

// An airport object without coordinates must not be treated as null island.
static void testMissingCoordinatesAreNotUsable() {
    JsonDocument doc;
    DeserializationError err = deserializeJson(
        doc,
        "{\"iata_code\":\"JFK\",\"municipality\":\"New York\"}"
    );
    CHECK(!err);
    GeoPoint point = readAirportCoordinate(doc.as<JsonObject>());
    CHECK(!RadarRoute::isUsableCoordinate(point));
    CHECK(routeIsPlausible(point, point, kLivePosition, kRouteCorridorToleranceKm));
}

static JsonObject parseAirport(JsonDocument &doc, const char *json) {
    CHECK(!deserializeJson(doc, json));
    return doc.as<JsonObject>();
}

// The corridor check must survive an ADSBdb response that omits coordinates:
// the compiled-in catalog resolves them from the airport codes.
static void testCatalogResolvesMissingCoordinates() {
    JsonDocument icaoDoc;
    GeoPoint viaIcao = resolveAirportCoordinate(
        parseAirport(icaoDoc, "{\"icao_code\":\"KJFK\",\"iata_code\":\"JFK\"}")
    );
    CHECK(RadarRoute::isUsableCoordinate(viaIcao));
    CHECK(fabsf(viaIcao.lat - 40.6394f) < 0.01f);
    CHECK(fabsf(viaIcao.lon - (-73.7793f)) < 0.01f);

    JsonDocument iataDoc;
    GeoPoint viaIata = resolveAirportCoordinate(
        parseAirport(iataDoc, "{\"iata_code\":\"SAN\"}")
    );
    CHECK(RadarRoute::isUsableCoordinate(viaIata));
    CHECK(fabsf(viaIata.lat - 32.7336f) < 0.01f);
    CHECK(fabsf(viaIata.lon - (-117.19f)) < 0.01f);
}

// The catalog also overrides response coordinates when it knows the airport,
// so a wrong ADSBdb airport row cannot poison the check.
static void testCatalogOverridesResponseCoordinates() {
    JsonDocument doc;
    GeoPoint point = resolveAirportCoordinate(parseAirport(
        doc,
        "{\"icao_code\":\"KJFK\",\"latitude\":12.0,\"longitude\":34.0}"
    ));
    CHECK(fabsf(point.lat - 40.6394f) < 0.01f);
    CHECK(fabsf(point.lon - (-73.7793f)) < 0.01f);
}

// Airports outside the catalog (small fields) keep using response coordinates.
static void testUnknownAirportFallsBackToResponse() {
    JsonDocument doc;
    GeoPoint point = resolveAirportCoordinate(parseAirport(
        doc,
        "{\"icao_code\":\"XXXX\",\"iata_code\":\"XXX\","
        "\"latitude\":10.5,\"longitude\":20.25}"
    ));
    CHECK(RadarRoute::isUsableCoordinate(point));
    CHECK(fabsf(point.lat - 10.5f) < 0.001f);
    CHECK(fabsf(point.lon - 20.25f) < 0.001f);

    JsonDocument bareDoc;
    GeoPoint bare = resolveAirportCoordinate(
        parseAirport(bareDoc, "{\"icao_code\":\"XXXX\"}")
    );
    CHECK(!RadarRoute::isUsableCoordinate(bare));
}

// Dropping icao_code from the filter would silently defeat catalog resolution.
static void testFilterKeepsIcaoCode() {
    JsonDocument originDoc;
    JsonObject origin = parseEndpoint(originDoc, "origin");
    CHECK(strcmp(origin["icao_code"].as<const char *>(), "KJFK") == 0);
    GeoPoint point = resolveAirportCoordinate(origin);
    CHECK(fabsf(point.lat - 40.6394f) < 0.01f);
}

int main() {
    testFilterKeepsCoordinates();
    testReportedDefectIsRejectedEndToEnd();
    testMissingCoordinatesAreNotUsable();
    testCatalogResolvesMissingCoordinates();
    testCatalogOverridesResponseCoordinates();
    testUnknownAirportFallsBackToResponse();
    testFilterKeepsIcaoCode();
    printf("route json tests passed\n");
    return 0;
}
