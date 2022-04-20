#include "config_manager.h"

#include "gatt_server.h"

#include "esp_log.h"

namespace sd {

enum ConfigCharOP {
    SET = 0, GET
};

ConfigManager::ConfigManager() {}

ConfigManager::~ConfigManager() {
    is_shutdown_ = true;
}

esp_err_t ConfigManager::Init() {
    ESP_LOGI(LOG_TAG_CONFIG, "[INIT CONFIGURATION START]\n");

    RETURN_IF_ERROR(nvs_open(NVS_NS_CONFIG_MANAGER, NVS_READWRITE, &nvs_handle_));
    auto it = nvs_entry_find(NVS_DEFAULT_PART_NAME, NVS_NS_CONFIG_MANAGER, NVS_TYPE_BLOB);
    for (; it != nullptr; it = nvs_entry_next(it)) {
        nvs_entry_info_t entry;
        char value[kMaxValueLen + 1];
        size_t len = kMaxValueLen + 1;
        nvs_entry_info(it, &entry);
        ESP_ERROR_CHECK(nvs_get_str(nvs_handle_, entry.key, value, &len));
        configs_.emplace(entry.key, value);
    }

    ESP_LOGI(LOG_TAG_CONFIG, "[INIT CONFIGURATION END] entry num: %d\n", configs_.size());

    return ESP_OK;
}

esp_err_t ConfigManager::SetGATTServer(GATTServer* gatt_server) {
    gatt_server_ = gatt_server;

    uint8_t service_inst_id;
    ESP_ERROR_CHECK(gatt_server->CreateService(kSIDConfigManager, &service_inst_id));
    ESP_ERROR_CHECK(gatt_server_->AddCharateristic(service_inst_id, kCIDConfigManager,
                    [&](BufferPtr* read_buf, size_t* len) {
                        auto [name, value] = config_on_standby_;
                        ESP_LOGI(LOG_TAG_CONFIG, "[GET CONFIG] name: %s, value: %s\n", name.c_str(), value.c_str());
                        *len = 4 + name.size() + value.size();
                        *read_buf = create_unique_buf(*len);
                        auto buf = read_buf->get();
                        *(uint16_t*)&buf[0] = name.size();
                        *(uint16_t*)&buf[2 + name.size()] = value.size();
                        memcpy(&buf[2], name.c_str(), name.size());
                        memcpy(&buf[4 + name.size()], value.c_str(), value.size());
                    },
                    [&](uint16_t char_handle, uint8_t* write_buf, size_t len) {
                        auto op = (ConfigCharOP)write_buf[0];
                        switch(op) {
                            case SET: {
                                uint16_t name_len = *(uint16_t*)&write_buf[1];
                                uint16_t value_len = *(uint16_t*)&write_buf[3 + name_len];
                                std::string name((char*)&write_buf[3], name_len);
                                std::string value((char*)&write_buf[5 + name_len], value_len);
                                if (5 + name_len + value_len != len) {
                                    ESP_LOGE(LOG_TAG_CONFIG, "Bad ble message!\n");
                                    return;
                                }
                                Set(name, value);
                                break;
                            }
                            case GET: {
                                uint16_t name_len = *(uint16_t*)&write_buf[1];
                                std::string name((char*)&write_buf[3], name_len);
                                std::string value;
                                if (Get(name, &value) == ESP_OK) {
                                    config_on_standby_ = {name, value};
                                } else {
                                    config_on_standby_ = {"notfound", ""};
                                }
                                break;
                            }
                            default:
                                ESP_LOGE(LOG_TAG_CONFIG, "Invalid operation code: %d\n", op);
                        }
                    }, &char_handle_));

    return ESP_OK;
}

void ConfigManager::NotifyConfig(const std::string& name, const std::string& value) {
    size_t buf_len = 4 + name.size() + value.size();
    uint8_t* buf = (uint8_t*)malloc(buf_len);
    *(uint16_t*)&buf[0] = name.size();
    *(uint16_t*)&buf[2 + name.size()] = value.size();
    memcpy(&buf[2], name.c_str(), name.size());
    memcpy(&buf[4 + name.size()], value.c_str(), value.size());

    gatt_server_->Notify(char_handle_, buf, buf_len);

    free(buf);
}

esp_err_t ConfigManager::Set(const std::string& name, const std::string& value) {
    {
        MutexGuard g(&config_mutex_);
        RETURN_IF_ERROR(nvs_set_str(nvs_handle_, name.c_str(), value.c_str()));
        configs_[name] = value;
        configs_.emplace(name, value);
    }

    cb_mutex_.Lock();
    auto it = value_change_cbs_.find(name);
    if (it != value_change_cbs_.end()) {
        auto cb = it->second;
        cb_mutex_.Unlock();
        cb(value);
    } else {
        cb_mutex_.Unlock();
    }

    return ESP_OK;
}

esp_err_t ConfigManager::Get(const std::string& name, std::string* value) {
    MutexGuard g(&config_mutex_);

    auto it = configs_.find(name);
    if (it == configs_.end()) {
        return ESP_ERR_NOT_FOUND;
    }

    *value = it->second;

    return ESP_OK;
}

esp_err_t ConfigManager::GetOrSetDefault(const std::string& name, std::string* value,
                                         const std::string& default_value) {
    MutexGuard g(&config_mutex_);

    auto it = configs_.find(name);
    if (it == configs_.end()) {
        RETURN_IF_ERROR(nvs_set_str(nvs_handle_, name.c_str(), default_value.c_str()));
        configs_[name] = default_value;
        *value = default_value;
    } else {
        *value = it->second;
    }

    return ESP_OK;
}

void ConfigManager::ListenValueChange(const std::string& name, const ValueChangeCallback callback) {
    MutexGuard g(&cb_mutex_);
    value_change_cbs_[name] = callback;
}

}  // namespace sd

