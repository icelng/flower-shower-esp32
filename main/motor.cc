#include "motor.h"

#include "common.h"

#include "esp_log.h"

namespace sd {

Motor::Motor(const std::string motor_name) : motor_name_(motor_name),
                                             timer_params_(kMaxNumTimers),
                                             timer_ctxs_(kMaxNumTimers) {}

esp_err_t Motor::Init() {
    return ESP_OK;
}

esp_err_t Motor::Start(float speed) {
    return ESP_OK;
}

esp_err_t Motor::Stop() {
    return ESP_OK;
}

esp_err_t Motor::CreateTimer(MotorTimerParam* param) {
    // TODO(liang), add mutex, ensure task safe
    uint8_t new_timer_no = 0;
    for (new_timer_no = 0; new_timer_no < timer_params_.size(); new_timer_no++) {
        if (timer_params_[new_timer_no].get() == nullptr) break;
    }
    if (new_timer_no >= timer_params_.size()) {
        return ESP_ERR_NO_MEM;
    }
    if (param->period_ms >= portTICK_PERIOD_MS && param->period_ms < param->duration_ms) {
        return ESP_ERR_INVALID_ARG;
    }
    param->timer_no = new_timer_no;
    timer_params_.emplace_back(new MotorTimerParam(*param));
    return InitTimerContext(param);
}

esp_err_t Motor::InitTimerContext(MotorTimerParam* param) {
    // if the timer is not period, and is expired, there is no need to create timer context.
    auto curtime_ms = get_curtime_ms();
    if (param->period_ms < portTICK_PERIOD_MS &&
        (param->first_start_timestamp + param->duration_ms) <= curtime_ms) {
        return ESP_OK;
    }

    // calculate the ticks to start motor
    int64_t offset_from_frist_start = curtime_ms - param->first_start_timestamp;
    TickType_t ticks_to_start;
    if (offset_from_frist_start <= 0) {
        ticks_to_start = (0 - offset_from_frist_start) / portTICK_PERIOD_MS;
    } else {
        uint64_t offset_in_period = offset_from_frist_start % param->period_ms;
        ticks_to_start = (param->period_ms - offset_in_period) / portTICK_PERIOD_MS;
    }
    // because of the timer service implemention, ticks_to_start must be greater than 0
    ticks_to_start = ticks_to_start == 0 ? 1 : ticks_to_start;

    // init timer context
    char timer_name[16];
    sprintf(timer_name, "motor-timer-%d", param->timer_no);
    auto new_ctx = new MotorTimerCtx();
    timer_ctxs_.emplace_back(new_ctx);
    new_ctx->motor = this;
    new_ctx->timer_no = param->timer_no;
    new_ctx->motor_cmd = START_MOTOR;
    new_ctx->timer_handle = xTimerCreate(timer_name, ticks_to_start, pdFALSE, new_ctx, TimerTaskEntry);

    return ESP_OK;
}

void Motor::TimerTaskEntry(TimerHandle_t timer_handle) {
    auto* ctx = (MotorTimerCtx*) pvTimerGetTimerID(timer_handle);
    assert(ctx->timer_handle == timer_handle);
    ctx->motor->TimerTask(ctx);
}

void Motor::TimerTask(MotorTimerCtx* ctx) {
    auto timer_param = timer_params_[ctx->timer_no].get();
    TickType_t ticks_to_next_cmd;
    MotorTimerCMD next_cmd;

    switch(ctx->motor_cmd) {
        case START_MOTOR:
            ESP_LOGI(LOG_TAG_MOTOR, "[TIME START MOTOR] motor_name: %s timer_no: %d cur_time: %lld\n",
                     motor_name_.c_str(), ctx->timer_no, get_curtime_ms());

            Start(timer_param->speed);
            if ((timer_param->period_ms - timer_param->duration_ms) / portTICK_PERIOD_MS == 0) {
                // running forever
                next_cmd = START_MOTOR;
                ticks_to_next_cmd = timer_param->period_ms / portTICK_PERIOD_MS;
            } else {
                next_cmd = STOP_MOTOR;
                ticks_to_next_cmd = timer_param->duration_ms / portTICK_PERIOD_MS;
            }

            break;
        case STOP_MOTOR:
            ESP_LOGI(LOG_TAG_MOTOR, "[TIME STOP MOTOR] motor_name: %s timer_no: %d cur_time: %lld\n",
                     motor_name_.c_str(), ctx->timer_no, get_curtime_ms());

            Stop();
            if ((timer_param->period_ms / portTICK_PERIOD_MS) == 0) {
                // the timer is not period, just running once.
                ticks_to_next_cmd = 0;
                break;
            }
            next_cmd = START_MOTOR;
            ticks_to_next_cmd = (timer_param->period_ms - timer_param->duration_ms) / portTICK_PERIOD_MS;

            break;
    }

    if (ticks_to_next_cmd != 0) {
        ctx->motor_cmd = next_cmd;
        xTimerChangePeriod(ctx->timer_handle, ticks_to_next_cmd, 0);
        xTimerReset(ctx->timer_handle, 0);
    }
}

esp_err_t Motor::ListTimers(std::vector<MotorTimerParam>* times) {
    return ESP_OK;
}

esp_err_t Motor::ClearTimer(uint16_t time_no) {
    return ESP_OK;
}

esp_err_t Motor::ClearAllTimers() {
    return ESP_OK;
}

}  // namespace sd
