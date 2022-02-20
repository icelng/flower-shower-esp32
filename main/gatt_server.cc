#include "gatt_server.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_debug_helpers.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include <algorithm>
#include <memory>
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <vector>

#define GATTS_TAG "SILICON_DREAMS"

#define GATTS_SERVICE_UUID          0x00FF
#define GATTS_CHAR_UUID_TEST_A      0xFF01
#define GATTS_DESCR_UUID_TEST_A     0x3333
#define GATTS_NUM_HANDLE            16

#define TEST_DEVICE_NAME            "SILICON_DREAMS"
#define TEST_MANUFACTURER_DATA_LEN  17

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE        1024

#define PROFILE_A_APP_ID            0

namespace sd {


static uint8_t char1_str[] = {0x11,0x22,0x33};
static esp_gatt_char_prop_t a_property = 0;

GATTServer* g_gatt_server;

static esp_attr_value_t gatts_demo_char1_val =
{
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char1_str),
    .attr_value   = char1_str,
};

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env;

void prepare_write(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void exec_write(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    assert(g_gatt_server != nullptr);
    g_gatt_server->GAPEventHandler(event, param);
}

void GATTServer::GAPEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        xEventGroupSetBits(event_group_, kEGAdvConfigDone);
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        xEventGroupSetBits(event_group_, kEGAdvRspConfigDone);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        start_adv_status_ = param->adv_start_cmpl.status;
        xEventGroupSetBits(event_group_, kEGAdvStartComplete);
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed\n");
        } else {
            ESP_LOGI(GATTS_TAG, "Stop adv successfully\n");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTS_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

esp_err_t GATTServer::StartAdvertising() {
    esp_err_t ret;

    ret = esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "set device name error, error code = %x", ret);
        return ret;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return ret;
    }

    ret = esp_ble_gap_config_adv_data(&adv_data_);
    if (ret){
        ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
        return ret;
    }
    ret = esp_ble_gap_config_adv_data(&scan_rsp_data_);
    if (ret){
        ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
        return ret;
    }
    auto wait_bits = xEventGroupWaitBits(event_group_, kEGAdvConfigDone | kEGAdvRspConfigDone,
                                    pdTRUE, pdTRUE, kEGTimeout);
    assert(wait_bits == (kEGAdvConfigDone | kEGAdvRspConfigDone));

    ret = esp_ble_gap_start_advertising(&adv_params_);
    if (ret){
        ESP_LOGE(GATTS_TAG, "start advertising failed, error code = %x", ret);
        return ret;
    }
    wait_bits = xEventGroupWaitBits(event_group_, kEGAdvStartComplete, pdTRUE, pdTRUE, kEGTimeout);
    if (wait_bits != kEGAdvStartComplete) {
        ESP_LOGE(GATTS_TAG, "config adv data timeout!!");
        return ESP_ERR_TIMEOUT;
    }
    if (start_adv_status_ != ESP_BT_STATUS_SUCCESS) {
        ESP_LOGE(GATTS_TAG, "Advertising start failed\n");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void prepare_write(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp) {
        if (param->write.is_prep) {
            ESP_LOGE(GATTS_TAG, "prepare write\n");
            if (prepare_write_env->prepare_buf == NULL) {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE*sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL) {
                    ESP_LOGE(GATTS_TAG, "Gatt_server prep no mem\n");
                    status = ESP_GATT_NO_RESOURCES;
                }
            } else {
                if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_OFFSET;
                } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_ATTR_LEN;
                }
            }

            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               ESP_LOGE(GATTS_TAG, "Send response error\n");
            }
            free(gatt_rsp);
            if (status != ESP_GATT_OK){
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;

        } else {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

void exec_write(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param) {
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC) {
        esp_log_buffer_hex(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    } else {
        ESP_LOGI(GATTS_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

void GATTServer::GATTEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT: {
        ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        if (param->reg.status == ESP_GATT_OK) {
            gatts_if_ = gatts_if;
        } else {
            ESP_LOGE(GATTS_TAG, "Reg app failed, app_id %04x, status %d\n",
                    param->reg.app_id,
                    param->reg.status);
        }
        reg_app_status_ = param->reg.status;
        xEventGroupSetBits(event_group_, kEGAppRegisterComplete);
        break;
    }
    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);

        auto it = chars_.find(param->write.handle);
        assert(it != chars_.end());
        auto charateristic = it->second.get();

        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = charateristic->size;
        charateristic->read_cb(rsp.attr_value.value, charateristic->size);

        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        break;
    }
    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
        auto it = chars_.find(param->write.handle);
        assert(it != chars_.end());
        auto charateristic = it->second.get();
        size_t len = std::min(charateristic->size, (size_t)param->write.len);
        charateristic->write_cb(param->write.value, len);
        // TODO(liang), implement notification
        // if (!param->write.is_prep &&
        //     descr_handle_ == param->write.handle &&
        //     param->write.len == 2) {
        //     uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
        //     if (descr_value == 0x0001){
        //         if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
        //             ESP_LOGI(GATTS_TAG, "notify enable");
        //             uint8_t notify_data[15];
        //             for (int i = 0; i < sizeof(notify_data); ++i)
        //             {
        //                 notify_data[i] = i%0xff;
        //             }
        //             //the size of notify_data[] need less than MTU size
        //             esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, char_handle_,
        //                                     sizeof(notify_data), notify_data, false);
        //         }
        //     }else if (descr_value == 0x0002){
        //         if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
        //             ESP_LOGI(GATTS_TAG, "indicate enable");
        //             uint8_t indicate_data[15];
        //             for (int i = 0; i < sizeof(indicate_data); ++i)
        //             {
        //                 indicate_data[i] = i%0xff;
        //             }
        //             //the size of indicate_data[] need less than MTU size
        //             esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, char_handle_,
        //                                     sizeof(indicate_data), indicate_data, true);
        //         }
        //     }
        //     else if (descr_value == 0x0000){
        //         ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
        //     } else {
        //         ESP_LOGE(GATTS_TAG, "unknown descr value");
        //         esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
        //     }
        // }
        if (param->write.need_rsp) {
            esp_gatt_status_t status = ESP_GATT_OK;
            if (param->write.is_prep) {
                // TODO(liang), implement prepare write
                ESP_LOGE(GATTS_TAG, "prepare write is not supported now!\n");
                status = ESP_GATT_REQ_NOT_SUPPORTED;
                // prepare_write(gatts_if, &a_prepare_write_env, param);
            }
            esp_ble_gatts_send_response(gatts_if,
                                        param->write.conn_id,
                                        param->write.trans_id,
                                        status,
                                        NULL);
        }
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(GATTS_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        exec_write(&a_prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT: {
        ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
        auto service = services_[param->create.service_id.id.inst_id].get();
        service->service_handle = param->create.service_handle;
        esp_ble_gatts_start_service(service->service_handle);

        // char_uuid_.len = ESP_UUID_LEN_16;
        // char_uuid_.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;
        // a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        // esp_err_t add_char_ret = esp_ble_gatts_add_char(service_handle_, &char_uuid_,
        //                                                 ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        //                                                 a_property,
        //                                                 &gatts_demo_char1_val, NULL);
        // if (add_char_ret){
        //     ESP_LOGE(GATTS_TAG, "add char failed, error code =%x",add_char_ret);
        // }

        break;
    }
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT: {
        // uint16_t length = 0;
        // const uint8_t *prf_char;

        ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        add_char_status_ = param->add_char.status;
        new_char_handle_ = param->add_char.attr_handle;
        xEventGroupSetBits(event_group_, kEGAddCharComplete);
        // descr_uuid_.len = ESP_UUID_LEN_16;
        // descr_uuid_.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        // esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &length, &prf_char);
        // if (get_attr_ret == ESP_FAIL){
        //     ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
        // }

        // ESP_LOGI(GATTS_TAG, "the gatts demo char length = %x\n", length);
        // for(int i = 0; i < length; i++){
        //     ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x\n",i,prf_char[i]);
        // }
        // esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(service_handle_, &descr_uuid_,
        //                                                         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        // if (add_descr_ret){
        //     ESP_LOGE(GATTS_TAG, "add char descr failed, error code =%x", add_descr_ret);
        // }
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        // descr_handle_ = param->add_char_descr.attr_handle;
        // ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
        //          param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
                 param->start.status, param->start.service_handle);
        xEventGroupSetBits(event_group_, kEGServiceCreateComplete);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT: {
        esp_ble_conn_update_params_t conn_params;
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        conn_id_ = param->connect.conn_id;
        //start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params_);
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d attr_handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK){
            esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
        }
        break;
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    assert(g_gatt_server != nullptr);
    g_gatt_server->GATTEventHandler(event, gatts_if, param);
}

GATTServer* GATTServer::RegisterServer(const std::string& device_name) {
    // just support single instance now
    assert(g_gatt_server == nullptr);
    g_gatt_server = new GATTServer(device_name);
    return g_gatt_server;
}

GATTServer::GATTServer(const std::string& device_name) :
    app_id_(0),
    device_name_(device_name) {

    adv_data_.set_scan_rsp = false;
    adv_data_.include_name = true;
    adv_data_.include_txpower = false;
    adv_data_.min_interval = 0x0006; //slave connection min interval, Time = min_interval * 1.25 msec
    adv_data_.max_interval = 0x0010; //slave connection max interval, Time = max_interval * 1.25 msec
    adv_data_.appearance = 0x00;
    adv_data_.manufacturer_len = 0; //TEST_MANUFACTURER_DATA_LEN,
    adv_data_.p_manufacturer_data = nullptr; //&test_manufacturer[0],
    adv_data_.service_data_len = 0;
    adv_data_.p_service_data = NULL;
    adv_data_.service_uuid_len = sizeof(kServiceUUID128);
    adv_data_.p_service_uuid = (uint8_t*)kServiceUUID128;
    adv_data_.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);

    scan_rsp_data_.set_scan_rsp = true;
    scan_rsp_data_.include_name = true;
    scan_rsp_data_.include_txpower = true;
    // scan_rsp_data_.min_interval = 0x0006;
    // scan_rsp_data_.max_interval = 0x0010;
    scan_rsp_data_.appearance = 0x00;
    scan_rsp_data_.manufacturer_len = 0; //TEST_MANUFACTURER_DATA_LEN,
    scan_rsp_data_.p_manufacturer_data = NULL; //&test_manufacturer[0],
    scan_rsp_data_.service_data_len = 0;
    scan_rsp_data_.p_service_data = NULL;
    scan_rsp_data_.service_uuid_len = sizeof(kServiceUUID128);
    scan_rsp_data_.p_service_uuid = (uint8_t*)kServiceUUID128;
    scan_rsp_data_.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);

    adv_params_.adv_int_min        = 0x20;
    adv_params_.adv_int_max        = 0x40;
    adv_params_.adv_type           = ADV_TYPE_IND;
    adv_params_.own_addr_type      = BLE_ADDR_TYPE_PUBLIC;
    // adv_params_.peer_addr          =
    adv_params_.peer_addr_type     = BLE_ADDR_TYPE_PUBLIC;
    adv_params_.channel_map        = ADV_CHNL_ALL;
    adv_params_.adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;

    event_group_ = xEventGroupCreate();
    assert(event_group_ != nullptr);
}

GATTServer::~GATTServer() {
    vEventGroupDelete(event_group_);
}

esp_err_t GATTServer::AddCharateristic(uint8_t service_inst_id,
                                       uint16_t uuid,
                                       size_t size,
                                       char_rw_cb read_cb,
                                       char_rw_cb write_cb) {
    assert(size <= ESP_GATT_MAX_ATTR_LEN);
    if (service_inst_id >= services_.size()) {
        ESP_LOGE(GATTS_TAG, "failed to add charateristic, invalid service instance id: %d\n", service_inst_id);
        return ESP_ERR_INVALID_ARG;
    }

    auto service = services_[service_inst_id].get();
    esp_bt_uuid_t char_uuid;
    char_uuid.len = ESP_UUID_LEN_16;
    char_uuid.uuid.uuid16 = uuid;
    esp_gatt_char_prop_t property = ESP_GATT_CHAR_PROP_BIT_READ |
                                    ESP_GATT_CHAR_PROP_BIT_WRITE |
                                    ESP_GATT_CHAR_PROP_BIT_NOTIFY;
    auto ret = esp_ble_gatts_add_char(service->service_handle, &char_uuid,
                                      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                      property,
                                      &gatts_demo_char1_val,
                                      NULL);
    if (ret){
        ESP_LOGE(GATTS_TAG, "add char failed, error code =%x", ret);
        return ret;
    }

    auto wait_bits = xEventGroupWaitBits(event_group_, kEGAddCharComplete, pdTRUE, pdTRUE, kEGTimeout);
    if (wait_bits != kEGAddCharComplete) {
        ESP_LOGE(GATTS_TAG, "add charateristic timeout!\n");
        return ESP_ERR_TIMEOUT;
    }
    if (add_char_status_ != ESP_GATT_OK) {
        ESP_LOGE(GATTS_TAG, "failed to add charateristics, status: %d %s\n",
                 add_char_status_, esp_err_to_name(add_char_status_));
        return ESP_FAIL;
    }

    auto new_char = new Charateristic();
    new_char->service_inst_id = service_inst_id;
    new_char->char_handle = new_char_handle_;
    new_char->char_uuid = char_uuid;
    new_char->size = size;
    new_char->read_cb = read_cb;
    new_char->write_cb = write_cb;
    auto it = chars_.find(new_char_handle_);
    assert(it == chars_.end());
    chars_.emplace(new_char_handle_, new_char);

    return ESP_OK;
}

esp_err_t GATTServer::CreateService(uint16_t uuid, uint8_t* inst_id) {
    auto new_service = new Service();
    new_service->service_id.is_primary = true;
    new_service->service_id.id.inst_id = services_.size();
    new_service->service_id.id.uuid.len = ESP_UUID_LEN_16;
    new_service->service_id.id.uuid.uuid.uuid16 = uuid;
    services_.emplace_back(new_service);

    esp_ble_gatts_create_service(gatts_if_, &new_service->service_id, GATTS_NUM_HANDLE);
    auto wait_bits = xEventGroupWaitBits(event_group_, kEGServiceCreateComplete, pdTRUE, pdTRUE, kEGTimeout);
    if (wait_bits != kEGServiceCreateComplete) {
        ESP_LOGE(GATTS_TAG, "create service timeout!\n");
        services_.pop_back();
        return ESP_ERR_TIMEOUT;
    }

    *inst_id = new_service->service_id.id.inst_id;

    return ESP_OK;
}

esp_err_t GATTServer::InitBTStack() {
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t GATTServer::Init() {
    esp_err_t ret;

    ESP_LOGI(GATTS_TAG, "[INIT GATT SERVER START] device_name: %s\n", device_name_.c_str());

    ret = InitBTStack();
    if (ret) return ret;

    ret = StartAdvertising();
    if (ret) return ret;

    // init gatt
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return ret;
    }
    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return ret;
    }

    auto wait_bits = xEventGroupWaitBits(event_group_, kEGAppRegisterComplete, pdTRUE, pdTRUE, kEGTimeout);
    if (wait_bits != kEGAppRegisterComplete) {
        ESP_LOGE(GATTS_TAG, "gatts app register timeout, error code = %x", ret);
        return ret;
    }

    if (reg_app_status_ != ESP_GATT_OK) {
        ret = reg_app_status_;
        return ret;
    }

    ret = esp_ble_gatt_set_local_mtu(500);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", ret);
        return ret;
    }

    ESP_LOGI(GATTS_TAG, "[INIT GATT SERVER SUCCESSFULLY] device_name: %s\n", device_name_.c_str());

    return ESP_OK;
}

}  // namespace sd
