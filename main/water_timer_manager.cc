#include "water_timer_manager.h"

#include "esp_log.h"
#include <vector>

namespace sd {

WaterTimerManager::WaterTimerManager(ConfigManager* cfg_mgt, GATTServer* gatt_server, Motor* motor)
    : cfg_mgt_(cfg_mgt),
      gatt_server_(gatt_server),
      motor_(motor),
      timer_ctxs_(kMaxNumTimers),
      mutex_(std::make_unique<Mutex>()) {}

WaterTimerManager::~WaterTimerManager() {
    xTimerStop(reload_timer_handle_, 0);
    while (xTimerIsTimerActive(reload_timer_handle_)) vTaskDelay(1);
    xTimerDelete(reload_timer_handle_, 0);

    for (auto& ctx : timer_ctxs_) {
        xTimerStop(ctx->timer_handle, 0);
        while (xTimerIsTimerActive(ctx->timer_handle)) vTaskDelay(1);
        xTimerDelete(ctx->timer_handle, 0);
        ctx.reset();
    }

    gatt_server_->UnregisterConnectionStateChangeCb(conn_cb_handle_);
}

esp_err_t WaterTimerManager::Init() {
    ESP_LOGI(LOG_TAG_WATER_TIMER_MANAGER, "[INIT WATER TIMER MANAGER START]");

    RETURN_IF_ERROR(nvs_open(NVS_NS_WATER_TIMER_MANAGER, NVS_READWRITE, &nvs_handle_));
    auto it = nvs_entry_find(NVS_DEFAULT_PART_NAME, NVS_NS_WATER_TIMER_MANAGER, NVS_TYPE_BLOB);


    std::string water_speed_str;
    ESP_ERROR_CHECK(cfg_mgt_->GetOrSetDefault(kConfigNameWaterSpeed,
                                              &water_speed_str,
                                              std::to_string(kDefaultWaterSpeed)));
    water_speed_ = atof(water_speed_str.c_str());

    std::string ml_str;
    ESP_ERROR_CHECK(cfg_mgt_->GetOrSetDefault(kConfigNameWaterMLPerSecond,
                                              &ml_str,
                                              std::to_string(kDefaultWaterMLPerSecond)));
    ml_per_sec_ = atof(ml_str.c_str());

    std::vector<WaterTimer> timers;
    for (; it != nullptr; it = nvs_entry_next(it)) {
        nvs_entry_info_t entry;
        size_t len;
        auto& timer = timers.emplace_back();
        nvs_entry_info(it, &entry);
        ESP_ERROR_CHECK(nvs_get_blob(nvs_handle_, entry.key, &timer, &len));
    };
    for (auto& timer : timers) {
        ESP_ERROR_CHECK(SetupTimer(timer));
    }

    ESP_ERROR_CHECK(SetupGATTService());

    reload_timer_handle_ = xTimerCreate("reload-wt",
                                        60000 / portTICK_PERIOD_MS,
                                        pdTRUE, this,
                                        [](TimerHandle_t timer_handle) {
                                           auto* wtm = (WaterTimerManager*)pvTimerGetTimerID(timer_handle);
                                           wtm->ReloadAllTimers();
                                       });
    assert(reload_timer_handle_ != nullptr);
    assert(xTimerStart(reload_timer_handle_, 0));

    ESP_LOGI(LOG_TAG_WATER_TIMER_MANAGER, "[INIT WATER TIMER MANAGER SUCCEED]");

    return ESP_OK;
}

esp_err_t WaterTimerManager::SetupGATTService() {
    assert(gatt_server_ != nullptr);

    ESP_ERROR_CHECK(gatt_server_->RegisterConnectionStateChangeCb(&conn_cb_handle_, [&](bool is_connected) {
                    if (!is_connected && is_cotrolled_forcelly_) {
                        is_cotrolled_forcelly_ = false;
                        motor_->Stop();
                        ReloadAllTimers();
                    }
                }));

    uint8_t service_inst_id;
    ESP_ERROR_CHECK(gatt_server_->CreateService(kSIDWaterTimer, &service_inst_id));

    ESP_ERROR_CHECK(gatt_server_->AddCharateristic(service_inst_id, kCIDWaterTimer,
                [&](BufferPtr* read_buf, size_t* len) {
                    ESP_ERROR_CHECK(ListTimersEncoded(read_buf, len));
                },
                [&](uint16_t char_handle, uint8_t* write_buf, size_t len) {
                    HandleTimerOperation(write_buf, len);
                }));

    RETURN_IF_ERROR(gatt_server_->AddCharateristic(service_inst_id, kCIDWaterControl,
                [&](BufferPtr* read_buf, size_t* len) {
                },
                [&](uint16_t char_handle, uint8_t* buf, size_t len) {
                    auto op = (WaterOP)*buf;
                    switch (op) {
                        case START: {
                            MutexGuard g(mutex_.get());  // prevent reloading timer
                            // TODO(liang), stop all timers here
                            if (len == 1) {
                                motor_->Start(water_speed_);
                            } else if (len == 5){
                                auto water_speed = *(float*)&buf[1];
                                motor_->Start(water_speed);
                            } else {
                                ESP_LOGE(LOG_TAG_WATER_ADJUSTER, "Bad messge for op: %d\n", op);
                                break;
                            }
                            is_cotrolled_forcelly_ = true;
                            break;
                        }
                        case STOP:
                            motor_->Stop();
                            is_cotrolled_forcelly_ = false;
                            ReloadAllTimers();
                        break;
                        default:
                            ESP_LOGE(LOG_TAG_WATER_ADJUSTER, "Invalid water op: %d\n", op);
                    }
                }));

    RETURN_IF_ERROR(gatt_server_->AddCharateristic(service_inst_id, kCIDWaterSpeed,
                [&](BufferPtr* read_buf, size_t* len) {
                    *len = 4;
                    *read_buf = create_unique_buf(*len);
                    *(float*)(read_buf->get()) = water_speed_;
                },
                [&](uint16_t char_handle, uint8_t* buf, size_t len) {
                    if (len != 4) {
                        ESP_LOGE(LOG_TAG_WATER_ADJUSTER, "Bad message len: %d", len);
                        return;
                    }
                    water_speed_ = *(float*)buf;
                    ESP_ERROR_CHECK(cfg_mgt_->Set(kConfigNameWaterSpeed, std::to_string(water_speed_)));
                }));

    RETURN_IF_ERROR(gatt_server_->AddCharateristic(service_inst_id, kCIDWaterMLPerSecond,
                [&](BufferPtr* read_buf, size_t* len) {
                    *len = 4;
                    *read_buf = create_unique_buf(*len);
                    *(float*)(read_buf->get()) = ml_per_sec_;
                },
                [&](uint16_t char_handle, uint8_t* buf, size_t len) {
                    if (len != 4) {
                        ESP_LOGE(LOG_TAG_WATER_ADJUSTER, "Bad message len: %d", len);
                        return;
                    }
                    ml_per_sec_ = *(float*)buf;
                    ESP_ERROR_CHECK(cfg_mgt_->Set(kConfigNameWaterMLPerSecond, std::to_string(ml_per_sec_)));
                    UpdateAllTimersDuration();
                }));

    return ESP_OK;
}

void WaterTimerManager::HandleTimerOperation(uint8_t* write_buf, size_t len) {
    /*|-op(1)-|-timer_no(1)-|-wdays(1)-|-timestamp(8)-|-ml(4)-|*/
    auto op = (WaterTimerOP)write_buf[0];
    WaterTimer timer;
    esp_err_t ret;
    switch (op) {
        case CREATE:
            DecodeTimer(&write_buf[1], &timer);
            timer.duration_sec = (uint32_t)(timer.volume_ml / ml_per_sec_);
            ret = CreateTimer(timer);
            if (ret != ESP_OK) {
                ESP_LOGE(LOG_TAG_WATER_TIMER_MANAGER, "Failed to create water timer! err code: %d\n", ret);
            }
            break;
        case UPDATE:
            DecodeTimer(&write_buf[1], &timer);
            timer.duration_sec = (uint32_t)(timer.volume_ml / ml_per_sec_);
            UpdateTimer(timer);
            break;
        case DEL:
            /*|-op(1)-|-timer_no(1)-|*/
            if (len != 2) {
                ESP_LOGE(LOG_TAG_WATER_TIMER_MANAGER, "Bad message for op DEL\n");
                return;
            }
            DelTimer(write_buf[1]);
            break;
        case STOP_NOW:
            /*|-op(1)-|-timer_no(1)-|*/
            if (len != 2) {
                ESP_LOGE(LOG_TAG_WATER_TIMER_MANAGER, "Bad message for op STOP_NOW\n");
                return;
            }
            StopTimerNow(write_buf[1]);
            break;
        default:
            ESP_LOGE(LOG_TAG_WATER_TIMER_MANAGER, "Invalid OP: %d\n", op);
    }
}

void WaterTimerManager::StopTimerNow(uint8_t timer_no) {
    if (timer_no > kMaxNumTimers) return;

    MutexGuard g(mutex_.get());

    if (!timer_ctxs_[timer_no]) return;

    auto* ctx = timer_ctxs_[timer_no].get();
    uint64_t secs_to_stop;
    if (!IsTimerRunning(ctx, &secs_to_stop)) return;
    ctx->stopped_until = get_curtime_s() + secs_to_stop;
    motor_->Stop();

    ESP_LOGI(LOG_TAG_WATER_TIMER_MANAGER,
             "[STOP WATER TIMER] timer_no: %d, until: %lld",
             timer_no, ctx->stopped_until);
}

void WaterTimerManager::UpdateAllTimersDuration() {
    std::vector<WaterTimer> timers;
    ListTimers(&timers);
    for (auto& timer : timers) {
        timer.duration_sec = (uint32_t)(timer.volume_ml / ml_per_sec_);
        UpdateTimer(timer);
    }
}

void WaterTimerManager::ReloadAllTimers() {
    // just for avoiding the drift of timer
    MutexGuard g(mutex_.get());

    // motor is being controlled somewhere, timers can not be reloaded now.
    if (is_cotrolled_forcelly_) return;

    ESP_LOGI(LOG_TAG_WATER_TIMER_MANAGER, "[RELOAD ALL WATER TIMERS START]");
    uint32_t num_timers = 0;
    for (auto& ctx : timer_ctxs_) {
        if (ctx.get() != nullptr) {
            num_timers++;
            DoSetupTimer(ctx.get());
        }
    }
    ESP_LOGI(LOG_TAG_WATER_TIMER_MANAGER, "[RELOAD ALL WATER TIMERS END] num timers: %d", num_timers);
}

esp_err_t WaterTimerManager::SetupTimer(WaterTimer& timer) {
    auto* new_ctx = new WaterTimerCtx();
    timer_ctxs_[timer.timer_no].reset(new_ctx);
    new_ctx->timer_handle = nullptr;
    new_ctx->timer_mgt = this;
    memcpy(&(new_ctx->timer), &timer, sizeof(WaterTimer));

    return DoSetupTimer(new_ctx);
}

esp_err_t WaterTimerManager::DoSetupTimer(WaterTimerCtx* ctx) {
    auto& timer = ctx->timer;
    // calculate the ticks to start water
    auto secs_to_start = CalcSecsToStart(timer);
    if (secs_to_start == UINT64_MAX) {
        if (ctx->timer_handle != nullptr) { assert(xTimerStop(ctx->timer_handle, 0)); }
        uint64_t duration_s_left;
        if (IsTimerRunning(ctx, &duration_s_left)) { motor_->Start(water_speed_, duration_s_left * 1000); }
        return ESP_OK;
    }
    TickType_t ticks_to_start = secs_to_start * 1000 / portTICK_PERIOD_MS;
    // because of the timer service implemention, ticks_to_start must be greater than 0
    ticks_to_start = ticks_to_start == 0 ? 1 : ticks_to_start;

    if (ctx->timer_handle != nullptr) {
        // resetup
        xTimerChangePeriod(ctx->timer_handle, ticks_to_start, 0);
        xTimerReset(ctx->timer_handle, 0);
    } else {
        char timer_name[16];
        sprintf(timer_name, "water-timer-%d", timer.timer_no);
        ctx->timer_handle = xTimerCreate(timer_name,
                                         ticks_to_start,
                                         pdFALSE,
                                         ctx,
                                         [](TimerHandle_t timer_handle) {
                                            auto* ctx = (WaterTimerCtx*) pvTimerGetTimerID(timer_handle);
                                            ctx->timer_mgt->StartWaterOnTime(ctx);
                                         });
        if (ctx->timer_handle == nullptr) {
            return ESP_ERR_NO_MEM;
        }
        assert(xTimerStart(ctx->timer_handle, 0));
    }

    uint64_t duration_s_left = 0;
    if (IsTimerRunning(ctx, &duration_s_left)) {
        motor_->Start(water_speed_, duration_s_left * 1000);
    }

    return ESP_OK;
}

void WaterTimerManager::StartWaterOnTime(WaterTimerCtx* ctx) {
    auto& timer = ctx->timer;

    ESP_LOGI(LOG_TAG_WATER_TIMER_MANAGER, "[TIME TO START WATER] timer_no: %d\n", timer.timer_no);

    uint64_t duration_sec = 0;
    if (IsTimerRunning(ctx, &duration_sec)) {
        motor_->Start(water_speed_, duration_sec * 1000);
    }

    auto secs_to_next_start = CalcSecsToStart(timer);
    if (secs_to_next_start == UINT64_MAX) { return; }
    assert(secs_to_next_start != 0);
    xTimerChangePeriod(ctx->timer_handle, secs_to_next_start * 1000 / portTICK_PERIOD_MS, 0);
    xTimerReset(ctx->timer_handle, 0);
}

esp_err_t WaterTimerManager::CreateTimer(WaterTimer& timer) {
    MutexGuard g(mutex_.get());

    uint8_t new_timer_no = 0;
    for (new_timer_no = 0; new_timer_no < timer_ctxs_.size(); new_timer_no++) {
        if (timer_ctxs_[new_timer_no].get() == nullptr) {
            timer.timer_no = new_timer_no;
            break;
        }
    }
    if (new_timer_no >= timer_ctxs_.size()) {
        return ESP_ERR_NO_MEM;
    }
    if (timer.duration_sec == 0 ||
        timer.duration_sec > kSecsPerDay) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(LOG_TAG_WATER_TIMER_MANAGER,
             "[CREATE WATER TIMER] timer no: %d, wdays: %d, first_start: %lld, volume: %d, duration: %d\n",
             new_timer_no, timer.wdays, timer.first_start_timestamp_s, timer.volume_ml, timer.duration_sec);

    char nvs_timer_key[4];
    sprintf(nvs_timer_key, "%d", new_timer_no);
    RETURN_IF_ERROR(nvs_set_blob(nvs_handle_, nvs_timer_key, &timer, sizeof(WaterTimer)));

    return SetupTimer(timer);
}

esp_err_t WaterTimerManager::UpdateTimer(const WaterTimer& timer) {
    if (timer.timer_no >= kMaxNumTimers) { return ESP_ERR_INVALID_ARG; }

    MutexGuard g(mutex_.get());

    auto ctx = timer_ctxs_[timer.timer_no].get();
    if (ctx == nullptr) { return ESP_ERR_NOT_FOUND; }

    ESP_LOGI(LOG_TAG_WATER_TIMER_MANAGER,
             "[UPDATE WATER TIMER] timer no: %d, wdays: %d, first_start: %lld, volume: %d, duration: %d\n",
             timer.timer_no, timer.wdays, timer.first_start_timestamp_s, timer.volume_ml, timer.duration_sec);

    ctx->timer = timer;
    RETURN_IF_ERROR(DoSetupTimer(ctx));

    char nvs_timer_key[4];
    sprintf(nvs_timer_key, "%d", timer.timer_no);
    RETURN_IF_ERROR(nvs_set_blob(nvs_handle_, nvs_timer_key, &timer, sizeof(WaterTimer)));

    return ESP_OK;
}

esp_err_t WaterTimerManager::GetTimer(uint8_t timer_no, WaterTimer* timer) {
    if (timer_no >= kMaxNumTimers || timer == nullptr) return ESP_ERR_INVALID_ARG;

    MutexGuard g(mutex_.get());

    auto ctx = timer_ctxs_[timer_no].get();
    if (ctx == nullptr) { return ESP_ERR_NOT_FOUND; }

    *timer = ctx->timer;

    return ESP_OK;
}


esp_err_t WaterTimerManager::DelTimer(uint8_t timer_no) {
    if (timer_no >= kMaxNumTimers) return ESP_ERR_INVALID_ARG;

    MutexGuard g(mutex_.get());

    auto ctx = timer_ctxs_[timer_no].get();
    if (ctx == nullptr) { return ESP_OK; }

    ESP_LOGI(LOG_TAG_WATER_TIMER_MANAGER, "[DELETE WATER TIMER] timer no: %d\n", timer_no);

    // stop timer and destroy context
    xTimerStop(ctx->timer_handle, 0);
    while (xTimerIsTimerActive(ctx->timer_handle)) vTaskDelay(1);
    xTimerDelete(ctx->timer_handle, 0);

    timer_ctxs_[timer_no].reset();

    // erase timer param in nvs
    char nvs_timer_key[4];
    sprintf(nvs_timer_key, "%d", timer_no);
    ESP_ERROR_CHECK(nvs_erase_key(nvs_handle_, nvs_timer_key));

    return ESP_OK;
}

esp_err_t WaterTimerManager::ListTimers(std::vector<WaterTimer>* timers) {
    MutexGuard g(mutex_.get());

    assert(timers != nullptr);

    for (auto& ctx : timer_ctxs_) {
        if (ctx) timers->push_back(ctx->timer);
    }

    ESP_LOGI(LOG_TAG_WATER_TIMER_MANAGER,
             "[LIST WATER TIMERS] num timers: %d\n",
             timers->size());

    return ESP_OK;
}

esp_err_t WaterTimerManager::ListTimersEncoded(BufferPtr* buf, size_t* buf_len) {
    /*|-num_timers(1)-|-timer_no(1)-|-wdays(1)-|-timestamp(8)-|-ml(4)-|-duration(4)-|-stopped_until(8)-|*/
    MutexGuard g(mutex_.get());

    uint8_t num_timers = 0;
    for (auto& ctx : timer_ctxs_) {
        if (ctx) num_timers++;
    }

    *buf_len = num_timers * kSizeOfEncodedTimer + 1;
    *buf = create_unique_buf(*buf_len);

    memcpy(buf->get(), &num_timers, 1);

    uint64_t offset = 1;
    for (auto& ctx : timer_ctxs_) {
        if (!ctx) continue;
        auto& timer = ctx->timer;
        memcpy(buf->get() + offset, &timer.timer_no, 1);
        memcpy(buf->get() + offset + 1, &timer.wdays, 1);
        memcpy(buf->get() + offset + 2, &timer.first_start_timestamp_s, 8);
        memcpy(buf->get() + offset + 10, &timer.volume_ml, 4);
        memcpy(buf->get() + offset + 14, &timer.duration_sec, 4);
        memcpy(buf->get() + offset + 18, &ctx->stopped_until, 8);
        offset += kSizeOfEncodedTimer;
    }

    ESP_LOGI(LOG_TAG_WATER_TIMER_MANAGER,
             "[LIST WATER TIMERS] num timers: %d\n",
             num_timers);

    return ESP_OK;
}

uint64_t WaterTimerManager::CalcSecsToStart(const WaterTimer& timer) {
    time_t now_timestamp_s;
    time(&now_timestamp_s);

    if (timer.wdays == 0) {
        if (now_timestamp_s > timer.first_start_timestamp_s) { return UINT64_MAX; }
        return timer.first_start_timestamp_s - now_timestamp_s;
    }

    auto tm = localtime(&now_timestamp_s);
    int64_t start_wday = 0;
    int64_t start_secs_in_day = timer.first_start_timestamp_s % kSecsPerDay;
    int64_t now_secs_in_day = now_timestamp_s % kSecsPerDay;
    uint8_t is_not_today = now_secs_in_day >= start_secs_in_day? 1 : 0;
    start_wday = (int)next_wday(timer.wdays, (tm->tm_wday + is_not_today) % 7);
    int64_t secs_to_start = (start_wday * kSecsPerDay + start_secs_in_day) - (tm->tm_wday * kSecsPerDay + now_secs_in_day);
    if (secs_to_start <= 0) { secs_to_start += kSecsPerWeek; }

    return secs_to_start;
}

bool WaterTimerManager::IsTimerRunning(const WaterTimerCtx* ctx, uint64_t* duration_s_left) {
    if (is_cotrolled_forcelly_) {
        // motor is being controlled somewhere, timers can not be started now.
        ESP_LOGI(LOG_TAG_WATER_TIMER_MANAGER, "[MOTOR IS BEGING CONTROLLED]");
        return false;
    }

    if (get_curtime_s() < ctx->stopped_until) {
        ESP_LOGI(LOG_TAG_WATER_TIMER_MANAGER,
                 "[TIMER IS STOPPED] timer_no: %d, until: %lld\n",
                 ctx->timer.timer_no,
                 ctx->stopped_until);
        return false;
    }

    auto& timer = ctx->timer;
    assert(timer.duration_sec < kSecsPerDay);

    time_t now_timestamp_s;
    time(&now_timestamp_s);

    if (now_timestamp_s < timer.first_start_timestamp_s) { return false; }

    int64_t secs_gone = (now_timestamp_s % kSecsPerDay) - (timer.first_start_timestamp_s % kSecsPerDay);
    secs_gone += (secs_gone < 0? kSecsPerDay : 0);
    time_t start_timestamp = now_timestamp_s - secs_gone;
    uint8_t start_wday = localtime(&start_timestamp)->tm_wday;

    if (secs_gone < timer.duration_sec &&
        (timer.wdays == 0 || ((timer.wdays >> start_wday) & 1))) {
        if (duration_s_left) { *duration_s_left = timer.duration_sec - secs_gone; }
        ESP_LOGI(LOG_TAG_WATER_TIMER_MANAGER,
                 "[TIMER IS WATERING] timer_no: %d, duration_s_left: %lld",
                 timer.timer_no, timer.duration_sec - secs_gone);
        return true;
    }
    return false;
}

void WaterTimerManager::DecodeTimer(uint8_t* buf, WaterTimer* timer) {
    /*|-timer_no(1)-|-wdays(1)-|-timestamp(8)-|-ml(4)-|*/
    assert(timer != nullptr);
    timer->timer_no = buf[0];
    timer->wdays = buf[1];
    timer->first_start_timestamp_s = *(uint64_t*)&buf[2];
    timer->volume_ml = *(uint32_t*)&buf[10];
}

}  // namespace sd

