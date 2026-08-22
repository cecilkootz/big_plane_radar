#pragma once

#include <Arduino.h>

#include "ha_mqtt_payloads.h"

// Home Assistant MQTT integration on top of ESP-IDF's esp-mqtt client, which
// runs its own task and reconnects on its own. Command callbacks fire on the
// MQTT task; publish calls are safe from any task.
namespace HaMqtt {

enum class Command : uint8_t {
    Range,
    MapBrightness,
    Display,
};

using CommandHandler = void (*)(Command command, const char *value);

struct Settings {
    bool enabled = false;
    String host;
    uint16_t port = 1883;
    String username;
    String password;
};

void configure(
    const Settings &settings,
    const HaMqttPayloads::DeviceMeta &meta,
    const HaMqttPayloads::EntityLimits &limits,
    CommandHandler handler
);

// Starts the client once Wi-Fi is up; later reconnects are esp-mqtt's job.
void ensureStarted();
bool connected();

// True once after each (re)connect so the caller republishes a fresh status.
bool consumeStatusRequest();

void publishAircraft(const HaMqttPayloads::AircraftSummary &summary);
void publishStatus(const HaMqttPayloads::StatusSummary &summary);

}  // namespace HaMqtt
