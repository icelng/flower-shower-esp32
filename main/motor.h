#pragma once

#include "common.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_pm.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "nvs_handle.hpp"

#include <memory>
#include <vector>

namespace sd {

// TODO(liang), abstract it
class Motor {
  public:

    explicit Motor(const std::string motor_name);
    ~Motor();

    esp_err_t Init();
    esp_err_t Start(float speed);
    esp_err_t Start(float speed, uint64_t duration_ms);
    esp_err_t Stop();

  private:
    void TuneSpeedTask();

    bool is_initiated_ = false;
    bool is_shutdown_ = false;
    EventGroupHandle_t event_group_;
    float speed_expected_;
    float speed_now_;
    bool isStby = false;
    std::string motor_name_;
    esp_pm_lock_handle_t pm_lock_;
    uint32_t total_duty_ = 0;

    static const TickType_t kEGTimeout = 3000 / portTICK_PERIOD_MS;
    const static EventBits_t kEGSpeedChanged = (1 << 0);
    const static EventBits_t kEGTuneTaskExited = (1 << 1);

    const static ledc_mode_t      kPWMTimerSpeedMode  = LEDC_LOW_SPEED_MODE;
    const static ledc_timer_bit_t kPWMTimerResolution = LEDC_TIMER_10_BIT;
    const static uint32_t         kPWMTimerFreqHz     = 20000;
    const static ledc_clk_cfg_t   kPWMTimerClkSrc     = LEDC_USE_APB_CLK;  // 40MHz
    const static ledc_timer_t     kPWMTimerNum        = LEDC_TIMER_0;
    const static ledc_channel_t   kPWMTimerChannel    = LEDC_CHANNEL_0;
    const static uint32_t         kPWMDutyTotalCnt    = 1 << kPWMTimerResolution;

    const static gpio_num_t kGPIOMotorIN1  = GPIO_NUM_14;
    const static gpio_num_t kGPIOMotorIN2  = GPIO_NUM_12;
    const static gpio_num_t kGPIOMotorPWM  = GPIO_NUM_13;
    const static gpio_num_t kGPIOMotorStby = GPIO_NUM_27;

};  // class Motor

}  // namespace sd;
