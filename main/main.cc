#include "gatt_server.h"

#include "config_manager.h"
#include "motor.h"
#include "rtc_ds3231.h"
#include "water_adjuster.h"

#include "esp_debug_helpers.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_pm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include <memory>
#include <stdio.h>
#include <string>

namespace sd {

// SID: Service UUID  CID: Charateristic UUID
const static uint16_t kSIDMotorTimer = 0x00FF;
const static uint16_t kCIDMotorTimer = 0xFF01;
const static uint16_t kCIDSystemTim = 0xFF02;

const static uint16_t kSIDMotorAdj = 0x00FF;

enum MotorTimerOP {
    ADD = 1, MOD, DEL, START, STOP
};

void hello_dream(void* arg) {
    printf("Hello silicon dreams!!!\n");

    esp_pm_config_esp32_t pm_config = {.max_freq_mhz = 240, .min_freq_mhz = 40, .light_sleep_enable = true};
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));

    auto cfg_mgt = std::make_unique<ConfigManager>();
    cfg_mgt->Init();

    auto rtc = std::make_unique<RTCDS3231>();
    ESP_ERROR_CHECK(rtc->Init());

    auto motor = std::make_unique<Motor>("motor");
    motor->Init();

    auto gatt_server = GATTServer::RegisterServer(cfg_mgt.get());
    uint8_t service_inst_id;
    ESP_ERROR_CHECK(gatt_server->Init());
    ESP_ERROR_CHECK(cfg_mgt->SetGATTServer(gatt_server));

    auto water_adjuster = std::make_unique<WaterAdjuster>(cfg_mgt.get(), gatt_server, motor.get());
    water_adjuster->Init();

    ESP_ERROR_CHECK(gatt_server->CreateService(kSIDMotorTimer, &service_inst_id));
    ESP_ERROR_CHECK(gatt_server->AddCharateristic(service_inst_id, kCIDMotorTimer,
                [&motor](BufferPtr* read_buf, size_t* len) {
                    ESP_ERROR_CHECK(motor->ListTimersEncoded(read_buf, len));
                    ESP_LOGI(LOG_TAG_MAIN, "[LIST TIMERS]\n");
                },
                [=, &motor](uint16_t char_handle, uint8_t* buf, size_t len) {
                    assert(len >= 1);
                    std::vector<MotorTimerParam> params;
                    auto op = (MotorTimerOP)buf[0];
                    switch(op) {
                        case ADD:
                            Motor::DecodeTimers(&buf[1], len - 1, &params);
                            for (auto& param : params) {
                                Json j;
                                to_json(j, param);
                                ESP_LOGI(LOG_TAG_MAIN, "[CREATE TIMER] %s\n", j.dump().c_str());
                                ESP_ERROR_CHECK(motor->CreateTimer(&param));
                            }
                            break;
                        case MOD:
                            ESP_LOGI(LOG_TAG_MAIN, "[MODIFY TIMER]\n");
                            break;
                        case DEL:
                            ESP_LOGI(LOG_TAG_MAIN, "[DELETE TIMER]\n");
                            ESP_ERROR_CHECK(motor->ClearTimer(buf[1]));
                            break;
                        case STOP:
                            ESP_ERROR_CHECK(motor->Stop());
                            break;
                        default:
                            ESP_LOGE(LOG_TAG_MAIN, "Invalid operation code: %d\n", op);
                    }
                }));

    ESP_ERROR_CHECK(gatt_server->AddCharateristic(service_inst_id, kCIDSystemTim,
                [](BufferPtr* read_buf, size_t* len) {
                    *read_buf = create_unique_buf(*len);
                    *(uint64_t*)(read_buf->get()) = get_curtime_s();
                },
                [&rtc](uint16_t char_handle, uint8_t* write_buf, size_t len) {
                    assert(len == 8);
                    auto timestamp_s = *((uint64_t*)write_buf);
                    set_system_time(timestamp_s);
                    rtc->SetTime(timestamp_s);
                }));

    while (true) {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

}  // namespace sd

extern "C" void app_main(void) {
    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    xTaskCreate(sd::hello_dream, "hello-dream", 4096, NULL, 5, NULL);
}
