#include "ha_mqtt_payloads.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace HaMqttPayloads;

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        exit(1); \
    } \
} while (0)

static const char *kRangeOptions[] = {"5km", "10km", "15km", "25km"};

static Identity makeTestIdentity() {
    Identity identity;
    // efuse MAC packs mac[0] in the low byte; tail bytes are a1 b2 c3.
    makeIdentity(0x0000C3B2A1000000ULL, identity);
    return identity;
}

static EntityLimits makeTestLimits() {
    EntityLimits limits;
    limits.rangeOptions = kRangeOptions;
    limits.rangeOptionCount = 4;
    limits.brightnessMin = 20;
    limits.brightnessMax = 100;
    limits.brightnessStep = 5;
    return limits;
}

static void testIdentityFormatting() {
    Identity identity = makeTestIdentity();
    CHECK(strcmp(identity.deviceId, "plane_radar_a1b2c3") == 0);
    CHECK(strcmp(identity.clientId, "plane-radar-a1b2c3") == 0);
    CHECK(strcmp(identity.baseTopic, "plane-radar/a1b2c3") == 0);

    char topic[TOPIC_MAX];
    joinTopic(identity, RANGE_COMMAND_SUFFIX, topic, sizeof(topic));
    CHECK(strcmp(topic, "plane-radar/a1b2c3/cmd/range") == 0);
}

static void testAircraftJsonWithNearest() {
    AircraftSummary summary;
    summary.count = 7;
    summary.hasNearest = true;
    strcpy(summary.callsign, "BAW123");
    strcpy(summary.hex, "400abc");
    strcpy(summary.type, "A320");
    summary.distanceKm = 4.234f;
    summary.hasAltitude = true;
    summary.altitudeFt = 11987.4f;
    summary.speedKnots = 320.6f;
    summary.verticalRateFpm = -639.5f;

    JsonDocument doc;
    buildAircraftJson(summary, doc);
    CHECK(doc["count"].as<size_t>() == 7);
    CHECK(strcmp(doc["callsign"].as<const char *>(), "BAW123") == 0);
    CHECK(strcmp(doc["hex"].as<const char *>(), "400abc") == 0);
    CHECK(strcmp(doc["type"].as<const char *>(), "A320") == 0);
    CHECK(fabs(doc["distance_km"].as<double>() - 4.2) < 1e-9);
    CHECK(doc["altitude_ft"].as<int>() == 11987);
    CHECK(doc["speed_kt"].as<int>() == 321);
    CHECK(doc["vertical_rate_fpm"].as<int>() == -640 ||
          doc["vertical_rate_fpm"].as<int>() == -639);

    // Rounded distance must serialize cleanly, not as 4.19999...
    char payload[256];
    serializeJson(doc, payload, sizeof(payload));
    CHECK(strstr(payload, "4.2") != nullptr);
    CHECK(strstr(payload, "4.19") == nullptr);
}

// Empty sky: every key the discovery templates reference must exist as null
// so no template ever hits an undefined variable.
static void testAircraftJsonEmptySky() {
    AircraftSummary summary;
    JsonDocument doc;
    buildAircraftJson(summary, doc);
    CHECK(doc["count"].as<size_t>() == 0);
    for (const char *key : {
             "callsign", "hex", "type", "distance_km", "altitude_ft",
             "speed_kt", "vertical_rate_fpm"
         }) {
        CHECK(!doc[key].isUnbound());
        CHECK(doc[key].isNull());
    }
}

static void testStatusJson() {
    StatusSummary status;
    status.rssi = -52;
    strcpy(status.ip, "192.168.1.23");
    status.uptimeS = 3600;
    status.rangeLabel = "10km";
    status.mapBrightness = 80;
    status.displayOn = false;
    status.batteryPercent = 65;
    status.onBattery = true;

    JsonDocument doc;
    buildStatusJson(status, doc);
    CHECK(doc["rssi"].as<int>() == -52);
    CHECK(strcmp(doc["ip"].as<const char *>(), "192.168.1.23") == 0);
    CHECK(doc["uptime_s"].as<uint32_t>() == 3600);
    CHECK(strcmp(doc["range"].as<const char *>(), "10km") == 0);
    CHECK(doc["map_brightness"].as<int>() == 80);
    CHECK(strcmp(doc["display"].as<const char *>(), "OFF") == 0);
    CHECK(doc["battery_percent"].as<int>() == 65);
    CHECK(strcmp(doc["on_battery"].as<const char *>(), "ON") == 0);
}

// No battery reading yet (or no battery hardware): the keys must exist as
// null so the battery templates never hit an undefined variable.
static void testStatusJsonBatteryUnknown() {
    StatusSummary status;
    JsonDocument doc;
    buildStatusJson(status, doc);
    for (const char *key : {"battery_percent", "on_battery"}) {
        CHECK(!doc[key].isUnbound());
        CHECK(doc[key].isNull());
    }
}

static void testDiscoveryCommonFields() {
    Identity identity = makeTestIdentity();
    DeviceMeta meta;
    meta.model = "ESP32-S3-Touch-LCD-7B";
    meta.configurationUrl = "http://plane-radar.local/";
    meta.hasBatteryTelemetry = true;
    EntityLimits limits = makeTestLimits();

    for (size_t i = 0; i < DISCOVERY_ENTITY_COUNT; i++) {
        JsonDocument doc;
        char topic[TOPIC_MAX];
        CHECK(buildDiscovery(identity, meta, limits, i, topic, sizeof(topic), doc));

        char expectedPrefix[TOPIC_MAX];
        snprintf(expectedPrefix, sizeof(expectedPrefix), "homeassistant/%s/plane_radar_a1b2c3/%s/config",
                 DISCOVERY_ENTITIES[i].component, DISCOVERY_ENTITIES[i].key);
        CHECK(strcmp(topic, expectedPrefix) == 0);

        CHECK(!doc["name"].isNull());
        const char *uniqueId = doc["unique_id"].as<const char *>();
        CHECK(uniqueId != nullptr);
        CHECK(strncmp(uniqueId, "plane_radar_a1b2c3_", 19) == 0);
        CHECK(strcmp(doc["availability_topic"].as<const char *>(),
                     "plane-radar/a1b2c3/availability") == 0);
        CHECK(strcmp(doc["device"]["identifiers"][0].as<const char *>(),
                     "plane_radar_a1b2c3") == 0);
        CHECK(strcmp(doc["device"]["model"].as<const char *>(),
                     "ESP32-S3-Touch-LCD-7B") == 0);
        CHECK(!doc["origin"]["name"].isNull());
        CHECK(!doc["state_topic"].isNull());

        // Discovery payloads must fit the transport's serialization buffer.
        char payload[1280];
        size_t len = serializeJson(doc, payload, sizeof(payload));
        CHECK(len > 0 && len < sizeof(payload) - 1);
    }

    JsonDocument doc;
    char topic[TOPIC_MAX];
    CHECK(!buildDiscovery(identity, meta, limits, DISCOVERY_ENTITY_COUNT,
                          topic, sizeof(topic), doc));
}

static void testDiscoveryControls() {
    Identity identity = makeTestIdentity();
    DeviceMeta meta;
    meta.model = "ESP32-S3-Touch-LCD-7";
    EntityLimits limits = makeTestLimits();

    JsonDocument select;
    char topic[TOPIC_MAX];
    CHECK(buildDiscovery(identity, meta, limits, 6, topic, sizeof(topic), select));
    CHECK(strcmp(select["command_topic"].as<const char *>(),
                 "plane-radar/a1b2c3/cmd/range") == 0);
    CHECK(select["options"].size() == 4);
    CHECK(strcmp(select["options"][1].as<const char *>(), "10km") == 0);

    JsonDocument number;
    CHECK(buildDiscovery(identity, meta, limits, 7, topic, sizeof(topic), number));
    CHECK(strcmp(number["command_topic"].as<const char *>(),
                 "plane-radar/a1b2c3/cmd/brightness") == 0);
    CHECK(number["min"].as<int>() == 20);
    CHECK(number["max"].as<int>() == 100);
    CHECK(number["step"].as<int>() == 5);

    JsonDocument sw;
    CHECK(buildDiscovery(identity, meta, limits, 8, topic, sizeof(topic), sw));
    CHECK(strcmp(sw["command_topic"].as<const char *>(),
                 "plane-radar/a1b2c3/cmd/display") == 0);
    CHECK(strcmp(sw["payload_on"].as<const char *>(), "ON") == 0);
    CHECK(strcmp(sw["payload_off"].as<const char *>(), "OFF") == 0);
    // The switch state template must match what buildStatusJson publishes.
    CHECK(strcmp(sw["value_template"].as<const char *>(),
                 "{{ value_json.display }}") == 0);
}

static void testDiscoveryBattery() {
    Identity identity = makeTestIdentity();
    DeviceMeta meta;
    meta.model = "ESP32-S3-Touch-LCD-7B";
    meta.hasBatteryTelemetry = true;
    EntityLimits limits = makeTestLimits();

    JsonDocument percent;
    char topic[TOPIC_MAX];
    CHECK(buildDiscovery(identity, meta, limits, 9, topic, sizeof(topic), percent));
    CHECK(strcmp(topic,
                 "homeassistant/sensor/plane_radar_a1b2c3/battery/config") == 0);
    CHECK(strcmp(percent["value_template"].as<const char *>(),
                 "{{ value_json.battery_percent }}") == 0);
    CHECK(strcmp(percent["unit_of_measurement"].as<const char *>(), "%") == 0);
    CHECK(strcmp(percent["device_class"].as<const char *>(), "battery") == 0);
    CHECK(strcmp(percent["entity_category"].as<const char *>(), "diagnostic") == 0);

    JsonDocument onBattery;
    CHECK(buildDiscovery(identity, meta, limits, 10, topic, sizeof(topic), onBattery));
    CHECK(strcmp(topic,
                 "homeassistant/binary_sensor/plane_radar_a1b2c3/on_battery/config") == 0);
    CHECK(strcmp(onBattery["value_template"].as<const char *>(),
                 "{{ value_json.on_battery }}") == 0);
    // Payloads must match what buildStatusJson publishes.
    CHECK(strcmp(onBattery["payload_on"].as<const char *>(), "ON") == 0);
    CHECK(strcmp(onBattery["payload_off"].as<const char *>(), "OFF") == 0);
    CHECK(strcmp(onBattery["entity_category"].as<const char *>(), "diagnostic") == 0);
}

// LCD-7 has no battery telemetry: battery entities must not be discovered,
// while every other entity still is.
static void testDiscoveryBatteryGating() {
    Identity identity = makeTestIdentity();
    DeviceMeta meta;
    meta.model = "ESP32-S3-Touch-LCD-7";
    EntityLimits limits = makeTestLimits();

    for (size_t i = 0; i < DISCOVERY_ENTITY_COUNT; i++) {
        JsonDocument doc;
        char topic[TOPIC_MAX];
        bool built = buildDiscovery(identity, meta, limits, i, topic, sizeof(topic), doc);
        CHECK(built == !DISCOVERY_ENTITIES[i].requiresBatteryTelemetry);
    }
    CHECK(DISCOVERY_ENTITIES[9].requiresBatteryTelemetry);
    CHECK(DISCOVERY_ENTITIES[10].requiresBatteryTelemetry);
}

int main() {
    testIdentityFormatting();
    testAircraftJsonWithNearest();
    testAircraftJsonEmptySky();
    testStatusJson();
    testStatusJsonBatteryUnknown();
    testDiscoveryCommonFields();
    testDiscoveryControls();
    testDiscoveryBattery();
    testDiscoveryBatteryGating();
    printf("ha mqtt payload tests passed\n");
    return 0;
}
