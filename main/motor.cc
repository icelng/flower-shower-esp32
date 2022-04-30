#include "motor.h"

#include "esp_log.h"

namespace sd {

Motor::Motor(const std::string motor_name) : motor_name_(motor_name) {}

Motor::~Motor() {}

esp_err_t Motor::Init() {
    if (is_initiated_) return ESP_OK;

    ESP_LOGI(LOG_TAG_MOTOR, "[INIT MOTOR START] motor_name: %s\n", motor_name_.c_str());

    // init gpio before timer
    gpio_config_t config = {
        .pin_bit_mask = (1 << kGPIOMotorIN1) | (1 << kGPIOMotorIN2) | (1 << kGPIOMotorPWM) | (1 << kGPIOMotorStby),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));


    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = kPWMTimerSpeedMode,
        .duty_resolution  = kPWMTimerResolution,
        .timer_num        = kPWMTimerNum,
        .freq_hz          = 5000,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_USE_RTC8M_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = kGPIOMotorPWM,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = kPWMTimerChannel,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = kPWMTimerNum,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    gpio_set_level(kGPIOMotorIN1, 0);
    gpio_set_level(kGPIOMotorIN2, 0);
    gpio_set_level(kGPIOMotorStby, 1);

    ESP_LOGI(LOG_TAG_MOTOR, "[INIT MOTOR END] motor_name: %s\n", motor_name_.c_str());

    is_initiated_ = true;
    return ESP_OK;
}

esp_err_t Motor::Start(float speed) {
    ESP_LOGI(LOG_TAG_MOTOR, "[START MOTOR] motor_name: %s, speed: %f\n", motor_name_.c_str(), speed);

    if (speed > 0) {
        gpio_set_level(kGPIOMotorIN1, 1);
        gpio_set_level(kGPIOMotorIN2, 0);
    } else {
        gpio_set_level(kGPIOMotorIN1, 0);
        gpio_set_level(kGPIOMotorIN2, 1);
    }

    auto duty_cnt = speed * kPWMDutyTotalCnt;
    ESP_ERROR_CHECK(ledc_set_duty(kPWMTimerSpeedMode, kPWMTimerChannel, duty_cnt));
    ESP_ERROR_CHECK(ledc_update_duty(kPWMTimerSpeedMode, kPWMTimerChannel));

    return ESP_OK;
}

esp_err_t Motor::Stop() {
    ESP_LOGI(LOG_TAG_MOTOR, "[STOP MOTOR] motor_name: %s\n", motor_name_.c_str());
    gpio_set_level(kGPIOMotorIN1, 0);
    gpio_set_level(kGPIOMotorIN2, 0);

    ESP_ERROR_CHECK(ledc_set_duty(kPWMTimerSpeedMode, kPWMTimerChannel, 0));
    ESP_ERROR_CHECK(ledc_update_duty(kPWMTimerSpeedMode, kPWMTimerChannel));

    return ESP_OK;
}

esp_err_t Motor::Start(float speed, uint64_t duration_ms) {
    RETURN_IF_ERROR(Start(speed));

    if (duration_ms == 0) { return ESP_OK; }

    TickType_t ticks_to_stop = duration_ms / portTICK_PERIOD_MS;
    ticks_to_stop = ticks_to_stop == 0 ? 1 : ticks_to_stop;
    auto timer_handle = xTimerCreate("motor-timer-stop",
                                     ticks_to_stop,
                                     pdFALSE, this,
                                     [](TimerHandle_t timer_handle) {
                                        auto* motor = (Motor*) pvTimerGetTimerID(timer_handle);
                                        motor->Stop();
                                     });
    if (timer_handle == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    assert(xTimerStart(timer_handle, 0));

    return ESP_OK;
}

}  // namespace sd
