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

const static uint16_t kGATTServiceUUID = 0x00FF;
const static uint16_t kGATTCharUUIDCreateMotorTimer = 0xFF01;
const static uint16_t kGATTCharUUIDClearMotorTimer = 0xFF02;
const static uint16_t kGATTCharUUIDControlMotor = 0xFF03;
const static uint16_t kGATTCharUUIDListMotorTimers = 0xFF04;

const static uint16_t kGATTServiceSystemTime = 0x00FE;
const static uint16_t kGATTCharUUIDSystemTime = 0xFE01;

void hello_dream(void* arg) {
    printf("Hello silicon dreams!!!\n");

    RTCDS3231* rtc = new RTCDS3231();
    rtc->Init();

    auto motor = std::make_unique<Motor>("silicon motor");
    motor->Init();

    uint8_t service_inst_id;
    auto gatt_server = GATTServer::RegisterServer("SILICON DREAMS");
    ESP_ERROR_CHECK(gatt_server->Init());
    ESP_ERROR_CHECK(gatt_server->CreateService(kGATTServiceUUID, &service_inst_id));
    ESP_ERROR_CHECK(gatt_server->AddCharateristic(service_inst_id, kGATTCharUUIDCreateMotorTimer,
                [](BufferPtr* read_buf, size_t* len) {
                    *len = 1;
                    *read_buf = create_unique_buf(*len);
                    (read_buf->get())[0] = 'c';
                },
                [=, &motor](uint8_t* buf, size_t len) {
                    std::vector<MotorTimerParam> params;
                    Motor::DecodeTimers(buf, len, &params);
                    for (auto& param : params) {
                        Json j;
                        to_json(j, param);
                        ESP_LOGI(LOG_TAG_MAIN, "create timer: %s\n", j.dump().c_str());
                        ESP_ERROR_CHECK(motor->CreateTimer(&param));
                    }
                }));
    ESP_ERROR_CHECK(gatt_server->AddCharateristic(service_inst_id, kGATTCharUUIDClearMotorTimer,
                [](BufferPtr*, size_t*) {},
                [&motor](uint8_t* write_value, size_t len) {
                    if (len != 1) return;
                    ESP_ERROR_CHECK(motor->ClearTimer(write_value[0]));
                }));
    ESP_ERROR_CHECK(gatt_server->AddCharateristic(service_inst_id, kGATTCharUUIDControlMotor,
                [](BufferPtr*, size_t*) {},
                [&motor](uint8_t* write_value, size_t len) {
                    if (len != 1) return;
                    if (write_value[0] == 1) {
                        ESP_ERROR_CHECK(motor->Start(1));
                    } else if (write_value[0] == 0) {
                        ESP_ERROR_CHECK(motor->Stop());
                    }
                }));
    ESP_ERROR_CHECK(gatt_server->AddCharateristic(service_inst_id, kGATTCharUUIDListMotorTimers,
                [&motor](BufferPtr* read_buf, size_t* len) {
                    ESP_ERROR_CHECK(motor->ListTimersEncoded(read_buf, len));
                    ESP_LOGI(LOG_TAG_MAIN, "list timers encoded, len: %d\n", *len);
                },
                [](uint8_t* write_value, size_t len) {
                }));

    uint8_t time_service_inst_id;
    ESP_ERROR_CHECK(gatt_server->CreateService(kGATTServiceSystemTime, &time_service_inst_id));
    ESP_ERROR_CHECK(gatt_server->AddCharateristic(time_service_inst_id, kGATTCharUUIDSystemTime,
                [](BufferPtr* read_buf, size_t* len) {
                    *read_buf = create_unique_buf(*len);
                    *(uint64_t*)(read_buf->get()) = get_curtime_s();
                },
                [=](uint8_t* write_buf, size_t len) {
                    assert(len == 8);
                    auto timestamp_s = *((uint64_t*)write_buf);
                    set_system_time(timestamp_s);
                    rtc->SetTime(timestamp_s);
                }));

    RTCDS3231::Time time;
    rtc->GetCurrentTime(&time);
    set_system_time(time.timestamp_s - 3600 * 8);
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
