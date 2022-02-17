#include "sys/time.h"

#define LOG_TAG_MOTOR "motor"

#define NVS_NS_MOTOR_TIMER "motor-timer"

#define RETURN_IF_ERROR(esp_err)\
    if (esp_err) return esp_err

namespace sd {
static uint64_t get_curtime_ms() {
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    int64_t time_ms = (int64_t)tv_now.tv_sec * 1000L + (int64_t)tv_now.tv_usec / 1000L;
    return time_ms;
}

}  // namespace sd
