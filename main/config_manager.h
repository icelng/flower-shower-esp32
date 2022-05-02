#pragma once

#include "common.h"

#include "esp_err.h"
#include "nvs_handle.hpp"

#include <functional>
#include <string>
#include <unordered_map>

namespace sd {

static const std::string kConfigNameDeviceName = "device-name";
static const std::string kConfigNameWaterSpeed = "water-speed";
static const std::string kConfigNameLoginPassword = "password";
static const std::string kConfigNameWaterMLPerSecond = "ml-per-second";
static const std::string kConfigNameLedLevel = "led-level";

class GATTServer;

using ValueChangeCallback = std::function<void(const std::string&)>;

class ConfigManager {
  public:
    ConfigManager();
    ~ConfigManager();

    esp_err_t Init();
    esp_err_t SetGATTServer(GATTServer* gatt_server);

    esp_err_t Set(const std::string& name, const std::string& value);
    esp_err_t Get(const std::string& name, std::string* value);
    esp_err_t GetOrSetDefault(const std::string& name, std::string* value, const std::string& default_value);
    void ListenValueChange(const std::string& name, const ValueChangeCallback callback);

private:
    void NotifyConfig(const std::string& name, const std::string& value);

    std::unordered_map<std::string, std::string> configs_;
    // callback may be called from any task, which is determine by the caller of Set()
    std::unordered_map<std::string, ValueChangeCallback> value_change_cbs_;
    GATTServer* gatt_server_;
    uint16_t char_handle_;
    nvs_handle_t nvs_handle_;
    Mutex config_mutex_;
    Mutex cb_mutex_;
    bool is_shutdown_ = false;
    TaskHandle_t notification_task_handle_;
    std::pair<std::string, std::string> config_on_standby_;

    static const uint32_t kMaxKeyLen = 32;
    static const uint32_t kMaxValueLen = 128;

    const static uint16_t kSIDConfigManager = 0x000C;
    const static uint16_t kCIDConfigManager = 0x0C01;

};  // class Configuration

}  // namespace sd
