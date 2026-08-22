#include "ha_mqtt.h"

#include <ESPmDNS.h>
#include <WiFi.h>
#include <mqtt_client.h>

#include "app_log.h"

namespace HaMqtt {
namespace {

constexpr uint32_t START_RETRY_INTERVAL_MS = 15000;

Settings settings;
String resolvedHost;
uint32_t lastStartAttemptMs = 0;
HaMqttPayloads::DeviceMeta deviceMeta;
HaMqttPayloads::EntityLimits entityLimits;
HaMqttPayloads::Identity identity;
CommandHandler commandHandler = nullptr;
esp_mqtt_client_handle_t client = nullptr;
volatile bool clientConnected = false;
volatile bool statusRequested = false;

char availabilityTopic[HaMqttPayloads::TOPIC_MAX];
char aircraftTopic[HaMqttPayloads::TOPIC_MAX];
char statusTopic[HaMqttPayloads::TOPIC_MAX];
char rangeCommandTopic[HaMqttPayloads::TOPIC_MAX];
char brightnessCommandTopic[HaMqttPayloads::TOPIC_MAX];
char displayCommandTopic[HaMqttPayloads::TOPIC_MAX];

bool topicMatches(const esp_mqtt_event_handle_t event, const char *topic) {
    size_t len = strlen(topic);
    return event->topic_len == static_cast<int>(len) &&
           memcmp(event->topic, topic, len) == 0;
}

// MQTT task context.
void publishDiscovery() {
    JsonDocument doc;
    char topic[HaMqttPayloads::TOPIC_MAX];
    static char payload[1280];
    for (size_t i = 0; i < HaMqttPayloads::DISCOVERY_ENTITY_COUNT; i++) {
        doc.clear();
        if (!HaMqttPayloads::buildDiscovery(
                identity, deviceMeta, entityLimits, i,
                topic, sizeof(topic), doc)) {
            continue;
        }
        size_t len = serializeJson(doc, payload, sizeof(payload));
        esp_mqtt_client_publish(client, topic, payload, len, 1, true);
    }
}

void onMqttEvent(void *, esp_event_base_t, int32_t eventId, void *eventData) {
    auto *event = static_cast<esp_mqtt_event_handle_t>(eventData);
    switch (static_cast<esp_mqtt_event_id_t>(eventId)) {
    case MQTT_EVENT_CONNECTED:
        clientConnected = true;
        esp_mqtt_client_publish(client, availabilityTopic, "online", 0, 1, true);
        publishDiscovery();
        esp_mqtt_client_subscribe(client, rangeCommandTopic, 1);
        esp_mqtt_client_subscribe(client, brightnessCommandTopic, 1);
        esp_mqtt_client_subscribe(client, displayCommandTopic, 1);
        statusRequested = true;
        RADAR_LOGI("[mqtt] connected, discovery published\n");
        break;
    case MQTT_EVENT_DISCONNECTED:
        clientConnected = false;
        RADAR_LOGI("[mqtt] disconnected\n");
        break;
    case MQTT_EVENT_DATA: {
        // topic_len == 0 marks a continuation fragment of an oversized
        // payload; commands are always tiny.
        if (event->topic_len == 0 || commandHandler == nullptr) break;
        char value[16];
        size_t len = static_cast<size_t>(event->data_len);
        if (len >= sizeof(value)) len = sizeof(value) - 1;
        memcpy(value, event->data, len);
        value[len] = '\0';
        if (topicMatches(event, rangeCommandTopic)) {
            commandHandler(Command::Range, value);
        } else if (topicMatches(event, brightnessCommandTopic)) {
            commandHandler(Command::MapBrightness, value);
        } else if (topicMatches(event, displayCommandTopic)) {
            commandHandler(Command::Display, value);
        }
        break;
    }
    case MQTT_EVENT_ERROR:
        RADAR_LOGD("[mqtt] transport error\n");
        break;
    default:
        break;
    }
}

}  // namespace

void configure(
    const Settings &newSettings,
    const HaMqttPayloads::DeviceMeta &meta,
    const HaMqttPayloads::EntityLimits &limits,
    CommandHandler handler
) {
    settings = newSettings;
    deviceMeta = meta;
    entityLimits = limits;
    commandHandler = handler;
    HaMqttPayloads::makeIdentity(ESP.getEfuseMac(), identity);
    HaMqttPayloads::joinTopic(
        identity, HaMqttPayloads::AVAILABILITY_SUFFIX,
        availabilityTopic, sizeof(availabilityTopic));
    HaMqttPayloads::joinTopic(
        identity, HaMqttPayloads::AIRCRAFT_SUFFIX,
        aircraftTopic, sizeof(aircraftTopic));
    HaMqttPayloads::joinTopic(
        identity, HaMqttPayloads::STATUS_SUFFIX,
        statusTopic, sizeof(statusTopic));
    HaMqttPayloads::joinTopic(
        identity, HaMqttPayloads::RANGE_COMMAND_SUFFIX,
        rangeCommandTopic, sizeof(rangeCommandTopic));
    HaMqttPayloads::joinTopic(
        identity, HaMqttPayloads::BRIGHTNESS_COMMAND_SUFFIX,
        brightnessCommandTopic, sizeof(brightnessCommandTopic));
    HaMqttPayloads::joinTopic(
        identity, HaMqttPayloads::DISPLAY_COMMAND_SUFFIX,
        displayCommandTopic, sizeof(displayCommandTopic));
}

void ensureStarted() {
    if (client != nullptr || !settings.enabled || settings.host.isEmpty()) {
        return;
    }
    if (WiFi.status() != WL_CONNECTED) {
        return;
    }
    uint32_t now = millis();
    if (lastStartAttemptMs != 0 &&
        now - lastStartAttemptMs < START_RETRY_INTERVAL_MS) {
        return;
    }
    lastStartAttemptMs = now;

    // esp-mqtt's resolver cannot answer mDNS names, so resolve .local hosts
    // through the already-running responder.
    resolvedHost = settings.host;
    if (resolvedHost.endsWith(".local")) {
        IPAddress ip = MDNS.queryHost(
            resolvedHost.substring(0, resolvedHost.length() - 6));
        if (static_cast<uint32_t>(ip) == 0) {
            RADAR_LOGI("[mqtt] mdns lookup failed host=%s, retrying later\n",
                       settings.host.c_str());
            return;
        }
        resolvedHost = ip.toString();
    }

    esp_mqtt_client_config_t config = {};
    config.broker.address.hostname = resolvedHost.c_str();
    config.broker.address.port = settings.port;
    config.broker.address.transport = MQTT_TRANSPORT_OVER_TCP;
    if (settings.username.length() > 0) {
        config.credentials.username = settings.username.c_str();
    }
    if (settings.password.length() > 0) {
        config.credentials.authentication.password = settings.password.c_str();
    }
    config.credentials.client_id = identity.clientId;
    config.session.last_will.topic = availabilityTopic;
    config.session.last_will.msg = "offline";
    config.session.last_will.msg_len = 7;
    config.session.last_will.qos = 1;
    config.session.last_will.retain = true;
    config.session.keepalive = 30;
    config.buffer.size = 2048;
    config.task.stack_size = 8192;

    client = esp_mqtt_client_init(&config);
    if (client == nullptr) {
        RADAR_LOGE("[mqtt] client init failed\n");
        settings.enabled = false;
        return;
    }
    esp_mqtt_client_register_event(
        client, MQTT_EVENT_ANY, onMqttEvent, nullptr);
    if (esp_mqtt_client_start(client) != ESP_OK) {
        RADAR_LOGE("[mqtt] client start failed\n");
        esp_mqtt_client_destroy(client);
        client = nullptr;
        settings.enabled = false;
        return;
    }
    RADAR_LOGI("[mqtt] client started host=%s port=%u id=%s\n",
               settings.host.c_str(),
               static_cast<unsigned>(settings.port),
               identity.clientId);
}

bool connected() {
    return client != nullptr && clientConnected;
}

bool consumeStatusRequest() {
    if (!statusRequested) return false;
    statusRequested = false;
    return true;
}

void publishAircraft(const HaMqttPayloads::AircraftSummary &summary) {
    if (!connected()) return;
    JsonDocument doc;
    HaMqttPayloads::buildAircraftJson(summary, doc);
    static char payload[256];
    size_t len = serializeJson(doc, payload, sizeof(payload));
    esp_mqtt_client_publish(client, aircraftTopic, payload, len, 0, false);
}

void publishStatus(const HaMqttPayloads::StatusSummary &summary) {
    if (!connected()) return;
    JsonDocument doc;
    HaMqttPayloads::buildStatusJson(summary, doc);
    static char payload[256];
    size_t len = serializeJson(doc, payload, sizeof(payload));
    esp_mqtt_client_publish(client, statusTopic, payload, len, 0, false);
}

}  // namespace HaMqtt
