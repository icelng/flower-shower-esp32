#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nlohmann/json.hpp"
#include "sys/time.h"

#define LOG_TAG_MOTOR "motor"
#define LOG_TAG_MAIN "main"

#define NVS_NS_MOTOR_TIMER "motor-timer"

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

static inline uint64_t get_curtime_ms() {
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    int64_t time_ms = (int64_t)tv_now.tv_sec * 1000L + (int64_t)tv_now.tv_usec / 1000L;
    return time_ms;
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
