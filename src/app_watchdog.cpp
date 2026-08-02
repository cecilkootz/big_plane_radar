#include "app_watchdog.h"

#include <esp_err.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

#include "app_log.h"

namespace AppWatchdog {
namespace {

static constexpr uint32_t WATCHDOG_TIMEOUT_MS = 30000;
static constexpr uint32_t WATCHED_IDLE_CORES = 1U << 0;
static bool ready = false;

const char *resetReasonName(esp_reset_reason_t reason) {
    switch (reason) {
    case ESP_RST_POWERON: return "POWER ON";
    case ESP_RST_EXT: return "EXTERNAL PIN";
    case ESP_RST_SW: return "SOFTWARE";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INTERRUPT WATCHDOG";
    case ESP_RST_TASK_WDT: return "TASK WATCHDOG";
    case ESP_RST_WDT: return "OTHER WATCHDOG";
    case ESP_RST_DEEPSLEEP: return "DEEP SLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    case ESP_RST_USB: return "USB";
    case ESP_RST_JTAG: return "JTAG";
    case ESP_RST_EFUSE: return "EFUSE";
    case ESP_RST_PWR_GLITCH: return "POWER GLITCH";
    case ESP_RST_CPU_LOCKUP: return "CPU LOCKUP";
    case ESP_RST_UNKNOWN:
    default:
        return "UNKNOWN";
    }
}

}  // namespace

void logResetReason() {
    esp_reset_reason_t reason = esp_reset_reason();
    RADAR_LOGI(
        "[system] reset reason=%s (%d)\n",
        resetReasonName(reason),
        static_cast<int>(reason)
    );
}

bool begin() {
    esp_task_wdt_config_t config = {
        .timeout_ms = WATCHDOG_TIMEOUT_MS,
        .idle_core_mask = WATCHED_IDLE_CORES,
        .trigger_panic = true,
    };

    esp_err_t result = esp_task_wdt_reconfigure(&config);
    if (result == ESP_ERR_INVALID_STATE) {
        result = esp_task_wdt_init(&config);
    }
    if (result != ESP_OK) {
        RADAR_LOGE(
            "[watchdog] initialization failed: %s\n",
            esp_err_to_name(result)
        );
        return false;
    }

    ready = true;
    RADAR_LOGI(
        "[watchdog] ready timeout=%lums idle_mask=0x%lx\n",
        static_cast<unsigned long>(WATCHDOG_TIMEOUT_MS),
        static_cast<unsigned long>(WATCHED_IDLE_CORES)
    );
    return true;
}

bool subscribeCurrentTask(const char *name) {
    if (!ready) {
        RADAR_LOGE("[watchdog] cannot subscribe %s before initialization\n", name);
        return false;
    }

    esp_err_t result = esp_task_wdt_status(nullptr);
    if (result != ESP_OK) {
        result = esp_task_wdt_add(nullptr);
    }
    if (result != ESP_OK) {
        RADAR_LOGE(
            "[watchdog] task %s subscription failed: %s\n",
            name,
            esp_err_to_name(result)
        );
        return false;
    }

    RADAR_LOGI("[watchdog] task %s subscribed\n", name);
    esp_task_wdt_reset();
    return true;
}

void feed() {
    if (ready) {
        esp_task_wdt_reset();
    }
}

}  // namespace AppWatchdog
