#pragma once

#include "config_manager.h"
#include "gatt_server.h"
#include "motor.h"

#include "esp_err.h"

namespace sd {

class WaterAdjuster {
  public:
    WaterAdjuster(ConfigManager* cfg_mgt, GATTServer* gatt_server, Motor* motor);
    ~WaterAdjuster();

    esp_err_t Init();

  private:
    enum WaterOP {
        START = 0, STOP
    };

    ConfigManager* cfg_mgt_;
    GATTServer* gatt_server_;
    Motor* motor_;

    static const uint16_t kSIDWaterAdjuster = 0x000D;
    static const uint16_t kCIDWaterControl = 0x0D01;
    static const uint16_t kCIDWaterSpeed = 0x0D02;
    static const uint16_t kCIDWaterMLPerSecond = 0x0D03;

    static constexpr float kDefaultWaterSpeed = 0.8;
};  // class WaterAdjuster

}  // namespace sd
