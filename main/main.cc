#include "gatt_server.h"

#include "esp_debug_helpers.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include <stdio.h>
#include <string>

namespace sd {

void hello_dream(void* arg) {
    printf("Hello silicon dreams!!!\n");

    auto gatt_server = GATTServer::RegisterServer("SILICON DREAMS");
    gatt_server->Init();

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

    xTaskCreate(sd::hello_dream, "hello-dream", 2048, NULL, 5, NULL);
}
