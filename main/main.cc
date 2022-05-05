#include "gatt_server.h"

#include "config_manager.h"
#include "motor.h"
#include "rtc_ds3231.h"
#include "water_timer_manager.h"

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
const static uint16_t kSIDSystemTime = 0x00FF;
const static uint16_t kCIDSystemTime = 0xFF01;

const static gpio_num_t kGPIOLED = GPIO_NUM_5;
const static gpio_num_t kGPIOHumidityEn = GPIO_NUM_18;
const static uint32_t kDefaultLEDLevel = 1;

enum MotorTimerOP {
    ADD = 1, MOD, DEL, START, STOP
};

void hello_dream(void* arg) {
    ESP_LOGI(LOG_TAG_MAIN, "Happy Birthday to CC!");

    esp_pm_config_esp32_t pm_config = {.max_freq_mhz = 240, .min_freq_mhz = 40, .light_sleep_enable = true};
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));

    esp_pm_lock_handle_t pm_lock;
    ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "main-init", &pm_lock));
    ESP_ERROR_CHECK(esp_pm_lock_acquire(pm_lock));

    auto cfg_mgt = std::make_unique<ConfigManager>();
    cfg_mgt->Init();

    auto rtc = std::make_unique<RTCDS3231>();
    ESP_ERROR_CHECK(rtc->Init());

    auto motor = std::make_unique<Motor>("motor");
    motor->Init();

    auto gatt_server = GATTServer::RegisterServer(cfg_mgt.get());
    ESP_ERROR_CHECK(gatt_server->Init());
    ESP_ERROR_CHECK(cfg_mgt->SetGATTServer(gatt_server));

    auto water_timer_mgt = std::make_unique<WaterTimerManager>(cfg_mgt.get(), gatt_server, motor.get());
    ESP_ERROR_CHECK(water_timer_mgt->Init());

    uint8_t service_inst_id;
    ESP_ERROR_CHECK(gatt_server->CreateService(kSIDSystemTime, &service_inst_id));
    ESP_ERROR_CHECK(gatt_server->AddCharateristic(service_inst_id, kCIDSystemTime,
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

    // init led and humidity
    gpio_config_t config = {
        .pin_bit_mask = 1 << kGPIOLED | 1 << kGPIOHumidityEn,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));

    std::string led_level_str;
    cfg_mgt->GetOrSetDefault(kConfigNameLedLevel, &led_level_str, std::to_string(kDefaultLEDLevel));
    gpio_set_level(kGPIOLED, atoi(led_level_str.c_str()) > 0? 1 : 0);

    cfg_mgt->ListenValueChange(kConfigNameLedLevel, [&](const std::string& value) {
        gpio_set_level(kGPIOLED, atoi(value.c_str()) > 0? 1 : 0);
    });

    // unsupport humidity now
    gpio_set_level(kGPIOHumidityEn, 0);

    ESP_ERROR_CHECK(esp_pm_lock_release(pm_lock));
    esp_pm_lock_delete(pm_lock);

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
