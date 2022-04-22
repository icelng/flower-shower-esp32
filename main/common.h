#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nlohmann/json.hpp"
#include "sys/time.h"

#define LOG_TAG_MOTOR "motor"
#define LOG_TAG_MAIN "main"
#define LOG_TAG_RTC_DS3231 "rtc_ds3231"
#define LOG_TAG_CONFIG "configuration"
#define LOG_TAG_WATER_ADJUSTER "water_adjuster"

#define NVS_NS_MOTOR_TIMER "motor-timer"
#define NVS_NS_CONFIG_MANAGER "config-mgt"

#define RETURN_IF_ERROR(esp_err)\
    if (esp_err) return esp_err

namespace sd {

using Json = nlohmann::json;

struct BufferDeleter {
    void operator() (uint8_t* buf) {
        free(buf);
    }
};

using BufferPtr = std::unique_ptr<uint8_t, BufferDeleter>;

static inline auto create_unique_buf(size_t len) {
	return BufferPtr((uint8_t*)malloc(len));
}

static time_t to_timestamp(int year, int mon, int day, int hour, int min, int second) {
	struct tm tm;
	tm.tm_year = year - 1900;
	tm.tm_mon = mon - 1;
	tm.tm_mday = day;
    tm.tm_hour = hour;
    tm.tm_min = min;
    tm.tm_sec = second;
    return mktime(&tm);;
}

static inline uint64_t get_curtime_ms() {
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    uint64_t time_ms = (uint64_t)tv_now.tv_sec * 1000UL + (uint64_t)tv_now.tv_usec / 1000UL;
    return time_ms;
}

static inline uint64_t get_curtime_s() {
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    return tv_now.tv_sec;
}

static inline void set_system_time(time_t timestamp_s) {
    struct timeval tv;
    struct timezone tz;
    tv.tv_sec = timestamp_s;
    tv.tv_usec = 0;
    tz.tz_minuteswest = -480;  // GMT+8
    settimeofday(&tv, &tz);
}

class Mutex {
  public:
    Mutex() {
        sem_ = xSemaphoreCreateMutex();
        assert(sem_ != nullptr);
    }

    ~Mutex() {
        vSemaphoreDelete(sem_);
    }

    void Lock() {
        xSemaphoreTake(sem_, portMAX_DELAY);
    }

    void Unlock() {
        xSemaphoreGive(sem_);
    }

  private:
    SemaphoreHandle_t sem_;
};

class MutexGuard {
  public:
    explicit MutexGuard(Mutex* mutex) : mutex_(mutex) {
        mutex_->Lock();
    }
    ~MutexGuard() {
        mutex_->Unlock();
    }
  private:
    Mutex* mutex_;
};

}  // namespace sd
