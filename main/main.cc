#include "gatt_server.h"

#include "motor.h"

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
const static uint16_t kGATTCharUUIDClearAllMotorTimers = 0xFF03;
const static uint16_t kGATTCharUUIDListMotorTimers = 0xFF04;
// const static uint16_t kGATTCharUUIDStartMotor = 0xFF03;
// const static uint16_t kGATTCharUUIDStopMotor = 0xFF04;

void hello_dream(void* arg) {
    printf("Hello silicon dreams!!!\n");

    auto motor = std::make_unique<Motor>("silicon motor");
    motor->Init();

    MotorTimerParam motor_param;
    motor_param.first_start_timestamp = get_curtime_ms();
    motor_param.period_ms = 8000;
    motor_param.duration_ms = 4000;
    motor_param.speed = 0.5;

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
                [&motor, &motor_param](uint8_t* write_value, size_t len) {
                    ESP_ERROR_CHECK(motor->CreateTimer(&motor_param));
                }));
    ESP_ERROR_CHECK(gatt_server->AddCharateristic(service_inst_id, kGATTCharUUIDClearMotorTimer,
                [](BufferPtr*, size_t*) {},
                [&motor](uint8_t* write_value, size_t len) {
                    ESP_ERROR_CHECK(motor->ClearTimer(write_value[0]));
                }));
    // ESP_ERROR_CHECK(gatt_server->AddCharateristic(service_inst_id, kGATTCharUUIDClearAllMotorTimers, 1,
    //             [](uint8_t* read_value, size_t len) {
    //                 read_value[0] = 0xcc;
    //             },
    //             [&motor](uint8_t* write_value, size_t len) {
    //                 ESP_ERROR_CHECK(motor->ClearAllTimers());
    //             }));
    ESP_ERROR_CHECK(gatt_server->AddCharateristic(service_inst_id, kGATTCharUUIDListMotorTimers,
                [&motor](BufferPtr* read_buf, size_t* len) {
                    Json j;
                    ESP_ERROR_CHECK(motor->ListTimersInJson(&j));
                    auto timers_json_str = j.dump();
                    ESP_LOGI(LOG_TAG_MAIN, "list timers: %s\n", timers_json_str.c_str());
                    *len = timers_json_str.size() + 1;
                    *read_buf = create_unique_buf(*len);
                    memcpy((*read_buf).get(), timers_json_str.c_str(), *len);
                },
                [](uint8_t* write_value, size_t len) {
                }));

    uint32_t cnt = 0;
    while (true) {
        printf("Heart Beat%d\n", cnt++);
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
