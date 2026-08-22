#pragma once

#include <ArduinoJson.h>

#include <initializer_list>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Topics and JSON payloads for the Home Assistant MQTT integration. Pure
// header so the discovery contract is covered by a native test; the esp-mqtt
// transport lives in ha_mqtt.cpp.
namespace HaMqttPayloads {

constexpr size_t TOPIC_MAX = 96;

constexpr const char *AVAILABILITY_SUFFIX = "availability";
constexpr const char *AIRCRAFT_SUFFIX = "aircraft";
constexpr const char *STATUS_SUFFIX = "status";
constexpr const char *RANGE_COMMAND_SUFFIX = "cmd/range";
constexpr const char *BRIGHTNESS_COMMAND_SUFFIX = "cmd/brightness";
constexpr const char *DISPLAY_COMMAND_SUFFIX = "cmd/display";

struct Identity {
    char deviceId[24] = {};   // plane_radar_a1b2c3, unique_id prefix
    char clientId[24] = {};   // plane-radar-a1b2c3
    char baseTopic[24] = {};  // plane-radar/a1b2c3
};

struct DeviceMeta {
    const char *model = "";
    const char *configurationUrl = nullptr;
    bool hasBatteryTelemetry = false;
};

struct EntityLimits {
    const char *const *rangeOptions = nullptr;
    size_t rangeOptionCount = 0;
    uint8_t brightnessMin = 20;
    uint8_t brightnessMax = 100;
    uint8_t brightnessStep = 5;
};

struct AircraftSummary {
    size_t count = 0;
    bool hasNearest = false;
    char callsign[10] = {};
    char hex[7] = {};
    char type[8] = {};
    float distanceKm = 0;
    bool hasAltitude = false;
    float altitudeFt = 0;
    float speedKnots = 0;
    float verticalRateFpm = 0;
};

struct StatusSummary {
    int rssi = 0;
    char ip[16] = {};
    uint32_t uptimeS = 0;
    const char *rangeLabel = "";
    uint8_t mapBrightness = 100;
    bool displayOn = true;
    int batteryPercent = -1;  // < 0: no battery reading yet (or unsupported)
    bool onBattery = false;
};

// The efuse MAC packs mac[0] in the low byte; bytes 3..5 are the
// device-unique tail.
inline void makeIdentity(uint64_t efuseMac, Identity &out) {
    unsigned tail[3] = {
        static_cast<unsigned>((efuseMac >> 24) & 0xFF),
        static_cast<unsigned>((efuseMac >> 32) & 0xFF),
        static_cast<unsigned>((efuseMac >> 40) & 0xFF),
    };
    snprintf(out.deviceId, sizeof(out.deviceId),
             "plane_radar_%02x%02x%02x", tail[0], tail[1], tail[2]);
    snprintf(out.clientId, sizeof(out.clientId),
             "plane-radar-%02x%02x%02x", tail[0], tail[1], tail[2]);
    snprintf(out.baseTopic, sizeof(out.baseTopic),
             "plane-radar/%02x%02x%02x", tail[0], tail[1], tail[2]);
}

inline void joinTopic(
    const Identity &identity,
    const char *suffix,
    char *out,
    size_t outLen
) {
    snprintf(out, outLen, "%s/%s", identity.baseTopic, suffix);
}

inline void buildAircraftJson(const AircraftSummary &summary, JsonDocument &doc) {
    doc["count"] = summary.count;
    if (!summary.hasNearest) {
        // Explicit nulls keep every value_template defined; the MQTT sensors
        // render null as unknown.
        for (const char *key : {
                 "callsign", "hex", "type", "distance_km", "altitude_ft",
                 "speed_kt", "vertical_rate_fpm"
             }) {
            doc[key] = nullptr;
        }
        return;
    }
    doc["callsign"] = summary.callsign;
    doc["hex"] = summary.hex;
    doc["type"] = summary.type;
    doc["distance_km"] =
        static_cast<double>(lroundf(summary.distanceKm * 10.0f)) / 10.0;
    if (summary.hasAltitude) {
        doc["altitude_ft"] = static_cast<int>(lroundf(summary.altitudeFt));
    } else {
        doc["altitude_ft"] = nullptr;
    }
    doc["speed_kt"] = static_cast<int>(lroundf(summary.speedKnots));
    doc["vertical_rate_fpm"] = static_cast<int>(lroundf(summary.verticalRateFpm));
}

inline void buildStatusJson(const StatusSummary &status, JsonDocument &doc) {
    doc["rssi"] = status.rssi;
    doc["ip"] = status.ip;
    doc["uptime_s"] = status.uptimeS;
    doc["range"] = status.rangeLabel;
    doc["map_brightness"] = status.mapBrightness;
    doc["display"] = status.displayOn ? "ON" : "OFF";
    if (status.batteryPercent >= 0) {
        doc["battery_percent"] = status.batteryPercent;
        doc["on_battery"] = status.onBattery ? "ON" : "OFF";
    } else {
        doc["battery_percent"] = nullptr;
        doc["on_battery"] = nullptr;
    }
}

struct DiscoveryEntityDef {
    const char *component;
    const char *key;
    bool requiresBatteryTelemetry = false;
};

inline constexpr DiscoveryEntityDef DISCOVERY_ENTITIES[] = {
    {"sensor", "aircraft_count"},
    {"sensor", "nearest_callsign"},
    {"sensor", "nearest_distance"},
    {"sensor", "nearest_altitude"},
    {"sensor", "wifi_rssi"},
    {"sensor", "ip_address"},
    {"select", "radar_range"},
    {"number", "map_brightness"},
    {"switch", "display"},
    {"sensor", "battery", true},
    {"binary_sensor", "on_battery", true},
};
inline constexpr size_t DISCOVERY_ENTITY_COUNT =
    sizeof(DISCOVERY_ENTITIES) / sizeof(DISCOVERY_ENTITIES[0]);

inline bool buildDiscovery(
    const Identity &identity,
    const DeviceMeta &meta,
    const EntityLimits &limits,
    size_t index,
    char *topicOut,
    size_t topicOutLen,
    JsonDocument &doc
) {
    if (index >= DISCOVERY_ENTITY_COUNT) return false;
    const DiscoveryEntityDef &def = DISCOVERY_ENTITIES[index];
    if (def.requiresBatteryTelemetry && !meta.hasBatteryTelemetry) return false;
    snprintf(topicOut, topicOutLen, "homeassistant/%s/%s/%s/config",
             def.component, identity.deviceId, def.key);

    char buffer[TOPIC_MAX];
    snprintf(buffer, sizeof(buffer), "%s_%s", identity.deviceId, def.key);
    doc["unique_id"] = buffer;
    joinTopic(identity, AVAILABILITY_SUFFIX, buffer, sizeof(buffer));
    doc["availability_topic"] = buffer;

    JsonObject device = doc["device"].to<JsonObject>();
    device["identifiers"][0] = identity.deviceId;
    device["name"] = "Big Plane Radar";
    device["manufacturer"] = "Waveshare";
    device["model"] = meta.model;
    if (meta.configurationUrl != nullptr) {
        device["configuration_url"] = meta.configurationUrl;
    }
    JsonObject origin = doc["origin"].to<JsonObject>();
    origin["name"] = "big_plane_radar";
    origin["url"] = "https://github.com/k4m454k/big_plane_radar";

    char aircraftTopic[TOPIC_MAX];
    char statusTopic[TOPIC_MAX];
    joinTopic(identity, AIRCRAFT_SUFFIX, aircraftTopic, sizeof(aircraftTopic));
    joinTopic(identity, STATUS_SUFFIX, statusTopic, sizeof(statusTopic));

    switch (index) {
    case 0:
        doc["name"] = "Aircraft count";
        doc["state_topic"] = aircraftTopic;
        doc["value_template"] = "{{ value_json.count }}";
        doc["state_class"] = "measurement";
        doc["icon"] = "mdi:radar";
        break;
    case 1:
        doc["name"] = "Nearest aircraft";
        doc["state_topic"] = aircraftTopic;
        doc["value_template"] = "{{ value_json.callsign }}";
        doc["json_attributes_topic"] = aircraftTopic;
        doc["json_attributes_template"] =
            "{{ {'hex': value_json.hex, 'type': value_json.type,"
            " 'speed_kt': value_json.speed_kt,"
            " 'vertical_rate_fpm': value_json.vertical_rate_fpm} | tojson }}";
        doc["icon"] = "mdi:airplane";
        break;
    case 2:
        doc["name"] = "Nearest distance";
        doc["state_topic"] = aircraftTopic;
        doc["value_template"] = "{{ value_json.distance_km }}";
        doc["unit_of_measurement"] = "km";
        doc["device_class"] = "distance";
        doc["state_class"] = "measurement";
        doc["suggested_display_precision"] = 1;
        break;
    case 3:
        doc["name"] = "Nearest altitude";
        doc["state_topic"] = aircraftTopic;
        doc["value_template"] = "{{ value_json.altitude_ft }}";
        doc["unit_of_measurement"] = "ft";
        doc["device_class"] = "distance";
        doc["state_class"] = "measurement";
        doc["suggested_display_precision"] = 0;
        break;
    case 4:
        doc["name"] = "Wi-Fi RSSI";
        doc["state_topic"] = statusTopic;
        doc["value_template"] = "{{ value_json.rssi }}";
        doc["unit_of_measurement"] = "dBm";
        doc["device_class"] = "signal_strength";
        doc["state_class"] = "measurement";
        doc["entity_category"] = "diagnostic";
        break;
    case 5:
        doc["name"] = "IP address";
        doc["state_topic"] = statusTopic;
        doc["value_template"] = "{{ value_json.ip }}";
        doc["entity_category"] = "diagnostic";
        doc["icon"] = "mdi:ip-network";
        break;
    case 6: {
        doc["name"] = "Radar range";
        doc["state_topic"] = statusTopic;
        doc["value_template"] = "{{ value_json.range }}";
        joinTopic(identity, RANGE_COMMAND_SUFFIX, buffer, sizeof(buffer));
        doc["command_topic"] = buffer;
        JsonArray options = doc["options"].to<JsonArray>();
        for (size_t i = 0; i < limits.rangeOptionCount; i++) {
            options.add(limits.rangeOptions[i]);
        }
        doc["icon"] = "mdi:map-marker-radius";
        break;
    }
    case 7:
        doc["name"] = "Map brightness";
        doc["state_topic"] = statusTopic;
        doc["value_template"] = "{{ value_json.map_brightness }}";
        joinTopic(identity, BRIGHTNESS_COMMAND_SUFFIX, buffer, sizeof(buffer));
        doc["command_topic"] = buffer;
        doc["min"] = limits.brightnessMin;
        doc["max"] = limits.brightnessMax;
        doc["step"] = limits.brightnessStep;
        doc["unit_of_measurement"] = "%";
        doc["mode"] = "slider";
        doc["icon"] = "mdi:brightness-6";
        break;
    case 8:
        doc["name"] = "Display";
        doc["state_topic"] = statusTopic;
        doc["value_template"] = "{{ value_json.display }}";
        joinTopic(identity, DISPLAY_COMMAND_SUFFIX, buffer, sizeof(buffer));
        doc["command_topic"] = buffer;
        doc["payload_on"] = "ON";
        doc["payload_off"] = "OFF";
        doc["icon"] = "mdi:monitor";
        break;
    case 9:
        doc["name"] = "Battery";
        doc["state_topic"] = statusTopic;
        doc["value_template"] = "{{ value_json.battery_percent }}";
        doc["unit_of_measurement"] = "%";
        doc["device_class"] = "battery";
        doc["state_class"] = "measurement";
        doc["entity_category"] = "diagnostic";
        break;
    case 10:
        doc["name"] = "On battery";
        doc["state_topic"] = statusTopic;
        doc["value_template"] = "{{ value_json.on_battery }}";
        doc["payload_on"] = "ON";
        doc["payload_off"] = "OFF";
        doc["entity_category"] = "diagnostic";
        doc["icon"] = "mdi:power-plug-off";
        break;
    default:
        return false;
    }
    return true;
}

}  // namespace HaMqttPayloads
