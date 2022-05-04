#pragma once

#include "common.h"
#include "config_manager.h"
#include "gatt_server.h"
#include "motor.h"

#include "esp_err.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "nvs_handle.hpp"

#include <unordered_map>
#include <vector>

namespace sd {

struct WaterTimer {
    uint8_t timer_no;
    uint8_t wdays;
    uint64_t first_start_timestamp_s;
    uint32_t volume_ml;
    uint32_t duration_sec;
};

class WaterTimerManager {
  public:
    WaterTimerManager(ConfigManager* cfg_mgt, GATTServer* gatt_server, Motor* motor);
    ~WaterTimerManager();

    esp_err_t Init();
    esp_err_t CreateTimer(WaterTimer& timer);
    esp_err_t GetTimer(uint8_t timer_no, WaterTimer* timer);
    esp_err_t UpdateTimer(const WaterTimer& timer);
    esp_err_t DelTimer(uint8_t timer_no);
    esp_err_t ListTimers(std::vector<WaterTimer>* timers);
    esp_err_t ListTimersEncoded(BufferPtr* buf, size_t* buf_len);

  private:
    enum WaterTimerOP {
        CREATE = 0, UPDATE, DEL, STOP_NOW
    };

    enum WaterOP {
        START = 0, STOP
    };

    struct WaterTimerCtx {
        WaterTimerManager* timer_mgt;
        WaterTimer timer;
        TimerHandle_t timer_handle = nullptr;
        uint64_t stopped_until = 0;
    };

    esp_err_t SetupGATTService();
    esp_err_t SetupTimer(WaterTimer& timer);
    esp_err_t DoSetupTimer(WaterTimerCtx* ctx);
    void StartWaterOnTime(WaterTimerCtx* ctx);
    void HandleTimerOperation(uint8_t* write_buf, size_t len);
    void UpdateAllTimersDuration();
    void ReloadAllTimers();
    void StopTimerNow(uint8_t timer_no);
    static uint64_t CalcSecsToStart(const WaterTimer& timer);
    static bool IsWatering(const WaterTimerCtx* ctx, uint64_t* duration_s_left);
    static void DecodeTimer(uint8_t* buf, WaterTimer* timer);

    ConfigManager* cfg_mgt_;
    GATTServer* gatt_server_;
    Motor* motor_;
    nvs_handle_t nvs_handle_;
    TimerHandle_t reload_timer_handle_ = nullptr;
    std::vector<std::unique_ptr<WaterTimerCtx>> timer_ctxs_;
    std::unique_ptr<Mutex> mutex_;

    float ml_per_sec_;
    float water_speed_;

    static const uint64_t kTimeZone = 8;
    static const uint64_t kSecsPerHour = 3600;
    static const uint64_t kSecsPerDay = 86400;
    static const uint64_t kSecsPerWeek = 604800;
    static const uint64_t kMaxNumTimers = 16;

    static const uint16_t kSIDWaterTimer = 0x00FE;
    static const uint16_t kCIDWaterTimer = 0xFE01;
    static const uint16_t kCIDWaterControl = 0xFE02;
    static const uint16_t kCIDWaterSpeed = 0xFE03;
    static const uint16_t kCIDWaterMLPerSecond = 0xFE04;

    static const size_t kSizeOfEncodedTimer = 26;

    static constexpr float kDefaultWaterSpeed = 1;
    static constexpr float kDefaultWaterMLPerSecond = 1;
};  // class WaterTimerManager

}  // namespace sd
