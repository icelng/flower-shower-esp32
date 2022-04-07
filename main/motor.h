#pragma once

#include "gatt_server.h"

#include "common.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "nvs_handle.hpp"

#include <memory>
#include <vector>

namespace sd {

struct MotorTimerParam {
    uint8_t  timer_no;
    uint64_t first_start_timestamp;
    uint64_t period_ms;
    uint64_t duration_ms;
    float    speed;
};

static const uint64_t kSizeOfEncodedTimer = 29;

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MotorTimerParam, timer_no, first_start_timestamp, period_ms, duration_ms, speed);

// TODO(liang), abstract it
class Motor {
  public:

    explicit Motor(const std::string motor_name);
    ~Motor();

    esp_err_t Init();
    esp_err_t Start(float speed);
    esp_err_t Stop();

    // timer manager
    esp_err_t CreateTimer(MotorTimerParam* timer);
    esp_err_t ListTimers(std::vector<MotorTimerParam>* timers);
    esp_err_t ListTimersEncoded(BufferPtr* buf, size_t* buf_len);
    esp_err_t ListTimersInJson(Json* json);
    esp_err_t ClearTimer(uint8_t timer_no);
    esp_err_t ClearAllTimers();

    static void DecodeTimers(uint8_t* buf, size_t buf_len, std::vector<MotorTimerParam>* timers);

  private:
    enum MotorTimerCMD {
        START_MOTOR,
        STOP_MOTOR
    };

    struct MotorTimerCtx {
        Motor*        motor;
        uint8_t       timer_no;
        TimerHandle_t timer_handle;
        MotorTimerCMD motor_cmd;
    };

    static const uint8_t    kMaxNumTimers   = 16;
    static const TickType_t kTicksPerSecond = 1000 / portTICK_PERIOD_MS;

    static void TimerTaskEntry(TimerHandle_t timer_handle);
    void TimerTask(MotorTimerCtx* ctx);
    esp_err_t InitTimerContext(MotorTimerParam* param);

    bool is_initiated_ = false;
    std::string motor_name_;
    GATTServer* gatt_server_;
    nvs_handle_t nvs_handle_;
    uint8_t num_timers_ = 0;
    std::vector<std::unique_ptr<MotorTimerParam>> timer_params_;
    std::vector<std::unique_ptr<MotorTimerCtx>> timer_ctxs_;
    std::unique_ptr<Mutex> mutex_;

    const static gpio_num_t kGPIOMotorIN1  = GPIO_NUM_27;
    const static gpio_num_t kGPIOMotorIN2  = GPIO_NUM_14;
    const static gpio_num_t kGPIOMotorPWM  = GPIO_NUM_12;
    const static gpio_num_t kGPIOMotorStby = GPIO_NUM_26;


};  // class Motor

}  // namespace sd;
