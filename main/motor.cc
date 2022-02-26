#include "motor.h"

#include "esp_log.h"

namespace sd {

Motor::Motor(const std::string motor_name) : motor_name_(motor_name),
                                             timer_params_(kMaxNumTimers),
                                             timer_ctxs_(kMaxNumTimers),
                                             mutex_(std::make_unique<Mutex>()) {}
Motor::~Motor() {
    {
        MutexGuard g(mutex_.get());
        is_initiated_ = false;
    }

    for (auto& ctx : timer_ctxs_) {
        if (ctx) xTimerStop(ctx->timer_handle, 0);
    }

    for (auto& ctx : timer_ctxs_) {
        if (ctx) {
            while (xTimerIsTimerActive(ctx->timer_handle)) vTaskDelay(1);
            xTimerDelete(ctx->timer_handle, 0);
        }
    }

    if (nvs_handle_) nvs_close(nvs_handle_);
}

esp_err_t Motor::Init() {
    if (is_initiated_) return ESP_OK;

    ESP_LOGI(LOG_TAG_MOTOR, "[INIT MOTOR START] motor_name: %s\n", motor_name_.c_str());

    RETURN_IF_ERROR(nvs_open(NVS_NS_MOTOR_TIMER, NVS_READWRITE, &nvs_handle_));

    auto it = nvs_entry_find(NVS_DEFAULT_PART_NAME, NVS_NS_MOTOR_TIMER, NVS_TYPE_BLOB);
    for (; it != nullptr; it = nvs_entry_next(it)) {
        nvs_entry_info_t entry;
        size_t len;
        MotorTimerParam* param = new MotorTimerParam;;
        nvs_entry_info(it, &entry);
        ESP_ERROR_CHECK(nvs_get_blob(nvs_handle_, entry.key, param, &len));
        uint8_t timer_no = atoi(entry.key);
        timer_params_[timer_no].reset(param);
    };

    for (auto& param : timer_params_) {
        if (param.get() != nullptr) {
            num_timers_++;
            ESP_ERROR_CHECK(InitTimerContext(param.get()));
        }
    }

    ESP_LOGI(LOG_TAG_MOTOR, "[INIT MOTOR END] motor_name: %s\n", motor_name_.c_str());

    is_initiated_ = true;
    return ESP_OK;
}

esp_err_t Motor::Start(float speed) {
    return ESP_OK;
}

esp_err_t Motor::Stop() {
    return ESP_OK;
}

esp_err_t Motor::CreateTimer(MotorTimerParam* param) {
    MutexGuard g(mutex_.get());

    if (!is_initiated_) return ESP_ERR_INVALID_STATE;

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

    // save to nvs
    char nvs_timer_key[4];
    sprintf(nvs_timer_key, "%d", new_timer_no);
    RETURN_IF_ERROR(nvs_set_blob(nvs_handle_, nvs_timer_key, param, sizeof(MotorTimerParam)));

    num_timers_++;
    timer_params_[new_timer_no].reset(new MotorTimerParam(*param));

    return InitTimerContext(param);
}

esp_err_t Motor::InitTimerContext(MotorTimerParam* param) {
    // if the timer is not period, and is expired, there is no need to create timer context.
    auto curtime_ms = get_curtime_ms();
    if (param->period_ms < portTICK_PERIOD_MS &&
        (param->first_start_timestamp + param->duration_ms) <= curtime_ms) {
        return ESP_OK;
    }

    ESP_LOGI(LOG_TAG_MOTOR,
             "[INIT TIMER CTX START] motor_name: %s timer_no: %d\n",
             motor_name_.c_str(), param->timer_no);

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
    timer_ctxs_[param->timer_no].reset(new_ctx);
    new_ctx->motor = this;
    new_ctx->timer_no = param->timer_no;
    new_ctx->motor_cmd = START_MOTOR;
    new_ctx->timer_handle = xTimerCreate(timer_name, ticks_to_start, pdFALSE, new_ctx, TimerTaskEntry);
    if (new_ctx->timer_handle == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    assert(xTimerStart(new_ctx->timer_handle, 0));

    ESP_LOGI(LOG_TAG_MOTOR,
             "[INIT TIMER CTX SUCCESSFULLY] motor_name: %s timer_no: %d\n",
             motor_name_.c_str(), param->timer_no);

    return ESP_OK;
}

void Motor::TimerTaskEntry(TimerHandle_t timer_handle) {
    auto* ctx = (MotorTimerCtx*) pvTimerGetTimerID(timer_handle);
    assert(ctx->timer_handle == timer_handle);
    ctx->motor->TimerTask(ctx);
}

void Motor::TimerTask(MotorTimerCtx* ctx) {
    auto timer_param = timer_params_[ctx->timer_no].get();
    TickType_t ticks_to_next_cmd = 0;
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

esp_err_t Motor::ListTimers(std::vector<MotorTimerParam>* timers) {
    MutexGuard g(mutex_.get());

    if (!is_initiated_) return ESP_ERR_INVALID_STATE;
    assert(timers != nullptr);

    for (auto& timer : timer_params_) {
        if (timer) timers->push_back(*(timer.get()));
    }

    return ESP_OK;
}

esp_err_t Motor::ListTimersEncoded(BufferPtr* buf, size_t* buf_len) {
    /*|-num_timers(1)-|-timer_no(1)-|-first_start_timestamp(8)-|-period_ms(8)-|-duration_ms(8)-|-speed(4)-|*/

    MutexGuard g(mutex_.get());

    if (!is_initiated_) return ESP_ERR_INVALID_STATE;

    *buf_len = num_timers_ * kSizeOfEncodedTimer + 1;
    *buf = create_unique_buf(*buf_len);

    memcpy(buf->get(), &num_timers_, 1);

    uint64_t offset = 1;
    for (auto& timer : timer_params_) {
        if (timer.get() == nullptr) continue;
        memcpy(buf->get() + offset, &timer->timer_no, 1);
        memcpy(buf->get() + offset + 1, &timer->first_start_timestamp, 8);
        memcpy(buf->get() + offset + 9, &timer->period_ms, 8);
        memcpy(buf->get() + offset + 17, &timer->duration_ms, 8);
        memcpy(buf->get() + offset + 25, &timer->speed, 4);
        offset += kSizeOfEncodedTimer;
    }

    return ESP_OK;
}

void Motor::DecodeTimers(uint8_t* buf, size_t buf_len, std::vector<MotorTimerParam>* timers) {
    /*|-num_timers(1)-|-timer_no(1)-|-first_start_timestamp(8)-|-period_ms(8)-|-duration_ms(8)-|-speed(4)-|*/

    uint8_t num_timers = buf[0];
    assert(num_timers <= kMaxNumTimers);

    uint64_t offset = 1;
    for (uint8_t i = 0; i < num_timers; i++) {
        assert(offset + kSizeOfEncodedTimer <= buf_len);
        MotorTimerParam param;
        param.timer_no = buf[offset];
        param.first_start_timestamp = *((uint64_t*)&buf[offset + 1]);
        param.period_ms = *((uint64_t*)&buf[offset + 9]);
        param.duration_ms = *((uint64_t*)&buf[offset + 17]);
        param.speed = *((float*)&buf[offset + 25]);
        timers->push_back(param);
        offset += kSizeOfEncodedTimer;
    }
}

esp_err_t Motor::ListTimersInJson(Json* json) {
    assert(json != nullptr);
    MutexGuard g(mutex_.get());

    if (!is_initiated_) return ESP_ERR_INVALID_STATE;

    for (auto& timer : timer_params_) {
        if (timer) {
            Json j;
            to_json(j, *timer.get());
            json->emplace_back(j);
        }
    }

    return ESP_OK;
}

esp_err_t Motor::ClearTimer(uint8_t timer_no) {
    MutexGuard g(mutex_.get());

    if (!is_initiated_) return ESP_ERR_INVALID_STATE;
    if (timer_no >= kMaxNumTimers) return ESP_ERR_INVALID_ARG;
    if (timer_params_[timer_no] == nullptr) return ESP_OK;

    // stop timer and destroy context
    auto* ctx = timer_ctxs_[timer_no].get();
    xTimerStop(ctx->timer_handle, 0);
    while (xTimerIsTimerActive(ctx->timer_handle)) vTaskDelay(1);
    xTimerDelete(ctx->timer_handle, 0);

    timer_ctxs_[timer_no].reset();
    timer_params_[timer_no].reset();
    num_timers_--;

    // erase timer param in nvs
    char nvs_timer_key[4];
    sprintf(nvs_timer_key, "%d", timer_no);
    ESP_ERROR_CHECK(nvs_erase_key(nvs_handle_, nvs_timer_key));

    return ESP_OK;
}

esp_err_t Motor::ClearAllTimers() {
    for (uint8_t timer_no = 0; timer_no < kMaxNumTimers; timer_no++) {
        RETURN_IF_ERROR(ClearTimer(timer_no));
    }

    return ESP_OK;
}

}  // namespace sd
