#include "water_adjuster.h"

#include "esp_log.h"

namespace sd {

WaterAdjuster::WaterAdjuster(ConfigManager* cfg_mgt, GATTServer* gatt_server, Motor* motor)
    : cfg_mgt_(cfg_mgt), gatt_server_(gatt_server), motor_(motor) {}

WaterAdjuster::~WaterAdjuster() {}

esp_err_t WaterAdjuster::Init() {
    uint8_t service_inst_id;
    RETURN_IF_ERROR(gatt_server_->CreateService(kSIDWaterAdjuster, &service_inst_id));

    RETURN_IF_ERROR(gatt_server_->AddCharateristic(service_inst_id, kCIDWaterControl,
                [&](BufferPtr* read_buf, size_t* len) {
                },
                [&](uint16_t char_handle, uint8_t* buf, size_t len) {
                    auto op = (WaterOP)*buf;
                    switch (op) {
                        case START: {
                            std::string water_speed_str;
                            ESP_ERROR_CHECK(cfg_mgt_->GetOrSetDefault(kConfigNameWaterSpeed,
                                                                      &water_speed_str,
                                                                      std::to_string(kDefaultWaterSpeed)));
                            float water_speed = atof(water_speed_str.c_str());
                            motor_->Start(water_speed);
                            break;
                        }
                        case STOP:
                            motor_->Stop();
                        break;
                        default:
                        ESP_LOGE(LOG_TAG_WATER_ADJUSTER, "Invalid water op: %d\n", op);
                    }
                }));

    RETURN_IF_ERROR(gatt_server_->AddCharateristic(service_inst_id, kCIDWaterSpeed,
                [&](BufferPtr* read_buf, size_t* len) {
                    *len = 4;
                    *read_buf = create_unique_buf(*len);
                    std::string water_speed_str;
                    ESP_ERROR_CHECK(cfg_mgt_->GetOrSetDefault(kConfigNameWaterSpeed,
                                                              &water_speed_str,
                                                              std::to_string(kDefaultWaterSpeed)));
                    *(float*)(read_buf->get()) = atof(water_speed_str.c_str());
                },
                [&](uint16_t char_handle, uint8_t* buf, size_t len) {
                    if (len != 4) {
                        ESP_LOGE(LOG_TAG_WATER_ADJUSTER, "Bad message len: %d", len);
                        return;
                    }
                    ESP_ERROR_CHECK(cfg_mgt_->Set(kConfigNameWaterSpeed, std::to_string(*(float*)buf)));
                }));

    RETURN_IF_ERROR(gatt_server_->AddCharateristic(service_inst_id, kCIDWaterMLPerSecond,
                [&](BufferPtr* read_buf, size_t* len) {
                },
                [&](uint16_t char_handle, uint8_t* buf, size_t len) {
                }));

    return ESP_OK;
}

}
