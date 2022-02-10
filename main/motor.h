#include "gatt_server.h"

#include "esp_err.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "nvs_handle.hpp"

#include <memory>
#include <vector>

namespace sd {

// TODO(liang), abstract it
class Motor {
  public:
    struct StartTime {
        uint8_t time_no;
        uint64_t start_timestamp;
        uint64_t duration_ms;
        uint64_t period_ms;
        float speed;
    };

    Motor(GATTServer* gatt_server);
    ~Motor();

    esp_err_t Init();
    esp_err_t Start(float speed);
    esp_err_t StartAtTime(StartTime* start_time);
    esp_err_t ListTimes(std::vector<StartTime>* times);
    esp_err_t ClearTime(uint16_t time_no);
    esp_err_t ClearAllTimes();
    esp_err_t Stop();
    
  private:
    void TimeStartTask();

    const static uint8_t kMaxNumTimes = 255;

    GATTServer* gatt_server_;
    std::unique_ptr<nvs::NVSHandle> nvs_handle;
    std::vector<StartTime> times_;
    StartTime running_time_; 

};  // class Motro

}  // namespace sd;
