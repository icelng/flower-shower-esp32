#pragma once

#include "gatt_server.h"

#include "common.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
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
    bool is_initiated_ = false;
    std::string motor_name_;
    uint32_t total_duty_ = 0;

    const static ledc_mode_t      kPWMTimerSpeedMode  = LEDC_LOW_SPEED_MODE;
    const static ledc_timer_bit_t kPWMTimerResolution = LEDC_TIMER_13_BIT;
    const static ledc_timer_t     kPWMTimerNum        = LEDC_TIMER_0;
    const static ledc_channel_t   kPWMTimerChannel    = LEDC_CHANNEL_0;
    const static uint32_t         kPWMDutyTotalCnt    = 1 << kPWMTimerResolution;

    const static gpio_num_t kGPIOMotorIN1  = GPIO_NUM_14;
    const static gpio_num_t kGPIOMotorIN2  = GPIO_NUM_12;
    const static gpio_num_t kGPIOMotorPWM  = GPIO_NUM_13;
    const static gpio_num_t kGPIOMotorStby = GPIO_NUM_27;

};  // class Motor

}  // namespace sd;
