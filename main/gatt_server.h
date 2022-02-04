#pragma once

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"


#include <atomic>
#include <string>

namespace sd {

static constexpr uint8_t kServiceUUID128[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

class GATTServer {
  public:
    GATTServer(const std::string& device_name);
    ~GATTServer();

    static GATTServer* RegisterServer(const std::string& device_name);

    esp_err_t Init();
    void CreateService();
    void GATTEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    void GAPEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

  private:

    esp_err_t InitBTStack();
    esp_err_t StartAdvertising();

    uint32_t app_id_;
    std::string device_name_;

    // gap
    esp_ble_adv_data_t adv_data_;
    esp_ble_adv_data_t scan_rsp_data_;
    esp_ble_adv_params_t adv_params_;

    // profile
    esp_gatts_cb_t gatts_cb_;
    uint16_t gatts_if_ = ESP_GATT_IF_NONE;
    uint16_t conn_id_;
    uint16_t service_handle_;
    esp_gatt_srvc_id_t service_id_;
    uint16_t char_handle_;
    esp_bt_uuid_t char_uuid_;
    esp_gatt_perm_t perm_;
    esp_gatt_char_prop_t property_;
    uint16_t descr_handle_;
    esp_bt_uuid_t descr_uuid_;

    // event group
    static const TickType_t kEGTimeout = 3000 / portTICK_PERIOD_MS;
    static const EventBits_t kEGAdvConfigDone = (1 << 0);
    static const EventBits_t kEGAdvRspConfigDone = (1 << 1);
    static const EventBits_t kEGAdvStartComplete = (1 << 2);
    static const EventBits_t kEGAppRegisterComplete = (1 << 3);

    EventGroupHandle_t event_group_;

    esp_bt_status_t start_adv_status_;
    esp_gatt_status_t reg_app_status_;
};

}  // namespace sd

