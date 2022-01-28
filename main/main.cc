#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

void hello_dream(void* arg) {
    printf("Hello silicon dreams!!!\n");

    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);

    esp_restart();
}

extern "C" void app_main(void) {
    xTaskCreate(hello_dream, "hello-dream", 2048, NULL, 5, NULL);
}