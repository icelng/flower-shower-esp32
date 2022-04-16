#include "gatt_server.h"

#include "motor.h"
#include "rtc_ds3231.h"

#include "esp_debug_helpers.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include <memory>
#include <stdio.h>
#include <string>

namespace sd {

// SUID: Service UUID  CID: Charateristic UUID
const static uint16_t kSUIDCommonConfiguration = 0x000C;
const static uint16_t kCIDDeviceName = 0x0C01;

const static uint16_t kSUIDMotorTimer = 0x00FF;
const static uint16_t kCIDMotorTimer = 0xFF01;
const static uint16_t kCIDSystemTim = 0xFF02;

const static uint16_t kSIDMotorAdj = 0x00FF;

enum MotorTimerOP {
    ADD = 1, MOD, DEL, STOP
};

void hello_dream(void* arg) {
    printf("Hello silicon dreams!!!\n");

    auto rtc = std::make_unique<RTCDS3231>();
    rtc->Init();

    auto motor = std::make_unique<Motor>("silicon motor");
    motor->Init();

    uint8_t service_inst_id;
    auto gatt_server = GATTServer::RegisterServer("SILICON DREAMS");
    ESP_ERROR_CHECK(gatt_server->Init());
    ESP_ERROR_CHECK(gatt_server->CreateService(kSUIDMotorTimer, &service_inst_id));
    ESP_ERROR_CHECK(gatt_server->AddCharateristic(service_inst_id, kCIDMotorTimer,
                [&motor](BufferPtr* read_buf, size_t* len) {
                    ESP_ERROR_CHECK(motor->ListTimersEncoded(read_buf, len));
                    ESP_LOGI(LOG_TAG_MAIN, "[LIST TIMERS]\n");
                },
                [=, &motor](uint8_t* buf, size_t len) {
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
                [&rtc](uint8_t* write_buf, size_t len) {
                    assert(len == 8);
                    auto timestamp_s = *((uint64_t*)write_buf);
                    set_system_time(timestamp_s);
                    rtc->SetTime(timestamp_s);
                }));

    RTCDS3231::Time time;
    rtc->GetCurrentTime(&time);
    set_system_time(time.timestamp_s);
    while (true) {
        rtc->GetCurrentTime(&time);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
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
