#include "motor.h"

#include "esp_log.h"

namespace sd {

Motor::Motor(const std::string motor_name) : motor_name_(motor_name) {}

Motor::~Motor() {
    is_shutdown_ = true;

    xEventGroupSetBits(event_group_, kEGSpeedChanged);
    xEventGroupWaitBits(event_group_, kEGTuneTaskExited, pdTRUE, pdTRUE, kEGTimeout);
    vEventGroupDelete(event_group_);
    esp_pm_lock_delete(pm_lock_);
}

esp_err_t Motor::Init() {
    if (is_initiated_) return ESP_OK;

    ESP_LOGI(LOG_TAG_MOTOR, "[INIT MOTOR START] motor_name: %s\n", motor_name_.c_str());

    ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "motor", &pm_lock_));

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
        .freq_hz          = kPWMTimerFreqHz,
        .clk_cfg          = kPWMTimerClkSrc
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
    gpio_set_level(kGPIOMotorStby, 0);  // low level is stby
    isStby_ = true;

    speed_now_ = 0;
    event_group_ = xEventGroupCreate();
    xTaskCreate([](void* arg) { ((Motor*)arg)->TuneSpeedTask(); }, "tune-motor-speed", 4096, this, 5, NULL);

    stop_timer_handle_ = xTimerCreate("stop-motor",
                                      1,
                                      pdFALSE, this,
                                      [](TimerHandle_t timer_handle) {
                                         auto* motor = (Motor*) pvTimerGetTimerID(timer_handle);
                                         motor->Stop();
                                      });
    assert(stop_timer_handle_ != nullptr);

    is_initiated_ = true;

    ESP_LOGI(LOG_TAG_MOTOR, "[INIT MOTOR END] motor_name: %s\n", motor_name_.c_str());

    return ESP_OK;
}

void Motor::TuneSpeedTask() {
    while (true) {
        auto wait_bits = xEventGroupWaitBits(event_group_, kEGSpeedChanged, pdTRUE, pdTRUE, kEGTimeout);
        if (is_shutdown_) { break; }
        if (wait_bits != kEGSpeedChanged) { continue; }

        uint32_t num_step = 50;
        float total_dist = speed_expected_ - speed_now_;
        float step_dist = total_dist / num_step;

        for (int i = 0; i < num_step; i++) {
            speed_now_ += step_dist;

            if (speed_now_ > 0) {
                gpio_set_level(kGPIOMotorIN1, 1);
                gpio_set_level(kGPIOMotorIN2, 0);
            } else {
                gpio_set_level(kGPIOMotorIN1, 0);
                gpio_set_level(kGPIOMotorIN2, 1);
            }
            if (isStby_) {
                ESP_LOGI(LOG_TAG_MOTOR, "[ACQUIRE PM LOCK]");
                isStby_ = false;
                gpio_set_level(kGPIOMotorStby, 1);  // low level is stby
                esp_pm_lock_acquire(pm_lock_);
            }

            auto duty_cnt = speed_now_ * kPWMDutyTotalCnt;
            ESP_ERROR_CHECK(ledc_set_duty(kPWMTimerSpeedMode, kPWMTimerChannel, duty_cnt));
            ESP_ERROR_CHECK(ledc_update_duty(kPWMTimerSpeedMode, kPWMTimerChannel));

            vTaskDelay(20 / portTICK_PERIOD_MS);
        }

        if (speed_now_ > -0.01 && speed_now_ < 0.01) {
            gpio_set_level(kGPIOMotorIN1, 0);
            gpio_set_level(kGPIOMotorIN2, 0);
            if (!isStby_) {
                ESP_LOGI(LOG_TAG_MOTOR, "[RELEASE PM LOCK]");
                isStby_ = true;
                gpio_set_level(kGPIOMotorStby, 0);
                esp_pm_lock_release(pm_lock_);
            }

            ESP_ERROR_CHECK(ledc_set_duty(kPWMTimerSpeedMode, kPWMTimerChannel, 0));
            ESP_ERROR_CHECK(ledc_update_duty(kPWMTimerSpeedMode, kPWMTimerChannel));
        }

    }

    xEventGroupSetBits(event_group_, kEGTuneTaskExited);
    vTaskDelete(nullptr);
}

esp_err_t Motor::Start(float speed) {
    ESP_LOGI(LOG_TAG_MOTOR, "[START MOTOR] motor_name: %s, speed: %f\n", motor_name_.c_str(), speed);

    speed_expected_ = speed;
    xEventGroupSetBits(event_group_, kEGSpeedChanged);

    return ESP_OK;
}

esp_err_t Motor::Stop() {
    ESP_LOGI(LOG_TAG_MOTOR, "[STOP MOTOR] motor_name: %s\n", motor_name_.c_str());

    speed_expected_ = 0;
    xEventGroupSetBits(event_group_, kEGSpeedChanged);

    return ESP_OK;
}

esp_err_t Motor::Start(float speed, uint64_t duration_ms) {
    ESP_LOGI(LOG_TAG_MOTOR,
             "[START MOTOR] motor_name: %s, speed: %f, duration_ms: %lld\n",
             motor_name_.c_str(), speed, duration_ms);

    if (duration_ms == 0) { return ESP_OK; }

    speed_expected_ = speed;
    xEventGroupSetBits(event_group_, kEGSpeedChanged);

    TickType_t ticks_to_stop = duration_ms / portTICK_PERIOD_MS;
    ticks_to_stop = ticks_to_stop == 0 ? 1 : ticks_to_stop;
    xTimerChangePeriod(stop_timer_handle_, ticks_to_stop, 0);
    xTimerReset(stop_timer_handle_, 0);

    return ESP_OK;
}

}  // namespace sd
