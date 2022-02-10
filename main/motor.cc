#include "motor.h"

namespace sd {

Motor::Motor(GATTServer* gatt_server) {
}


esp_err_t Motor::Init() {
    return ESP_OK;
}

esp_err_t Motor::Start(float speed) {
    return ESP_OK;
}

esp_err_t Motor::StartAtTime(StartTime* start_time) {
    if (times_.size() >= kMaxNumTimes) {
        return ESP_ERR_NO_MEM;
    }

    start_time->time_no = times_.size();
    times_.emplace_back(*start_time);

    return ESP_OK;
}

esp_err_t Motor::ListTimes(std::vector<StartTime>* times) {
    return ESP_OK;
}

esp_err_t Motor::ClearTime(uint16_t time_no) {
    return ESP_OK;
}

esp_err_t Motor::ClearAllTimes() {
    return ESP_OK;
}

esp_err_t Motor::Stop() {
    return ESP_OK;
}


}  // namespace sd
