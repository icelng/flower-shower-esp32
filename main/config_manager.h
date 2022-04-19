#pragma once

#include "common.h"

#include "esp_err.h"
#include "freertos/event_groups.h"
#include "nvs_handle.hpp"

#include <functional>
#include <string>
#include <unordered_map>

namespace sd {

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
    void NotificationTask();
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
    EventGroupHandle_t event_group_;

    static const TickType_t kEGTimeout = 3000 / portTICK_PERIOD_MS;
    static const EventBits_t kEGNotifyConfig = (1 << 0);
    static const EventBits_t kEGNotificationTaskJoin = (1 << 1);

    static const uint32_t kMaxKeyLen = 32;
    static const uint32_t kMaxValueLen = 128;

    const static uint16_t kSIDConfiguration = 0x000C;
    const static uint16_t kCIDConfiguration = 0x0C01;
    
};  // class Configuration

}  // namespace sd
