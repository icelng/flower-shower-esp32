#include "gatt_server.h"

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
    struct MotorTimerParam {
        uint8_t  timer_no;
        uint64_t first_start_timestamp;
        uint64_t period_ms;
        uint64_t duration_ms;
        float    speed;
    };

    explicit Motor(const std::string motor_name);
    ~Motor();

    esp_err_t Init();
    esp_err_t Start(float speed);
    esp_err_t Stop();

    // timer
    esp_err_t CreateTimer(MotorTimerParam* timer);
    esp_err_t ListTimers(std::vector<MotorTimerParam>* timers);
    esp_err_t ClearTimer(uint16_t timer_no);
    esp_err_t ClearAllTimers();

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
    esp_err_t InitMotorTimer(MotorTimerParam* param);

    std::string motor_name_;
    GATTServer* gatt_server_;
    std::unique_ptr<nvs::NVSHandle> nvs_handle_;
    std::vector<std::unique_ptr<MotorTimerParam>> timer_params_;
    std::vector<std::unique_ptr<MotorTimerCtx>> timer_ctxs_;

};  // class Motor

}  // namespace sd;
