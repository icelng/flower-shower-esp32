#pragma once

#include "esp_err.h"

struct tmrTimerControl;
typedef struct tmrTimerControl* TimerHandle_t;

namespace sd {

class RTCDS3231 {
  public:
    struct Time {
        uint16_t year;
        uint8_t month;
        uint8_t date;
        uint8_t hours;
        uint8_t minutes;
        uint8_t seconds;
        time_t timestamp_s;
    };

    RTCDS3231(bool adjust_system_time = true);
    ~RTCDS3231();

    esp_err_t Init();
    esp_err_t GetCurrentTime(Time* time);
    esp_err_t SetTime(time_t timestamp_s);
    esp_err_t SetTime(Time* time);
    esp_err_t AdjustSystemTime();

  private:
    bool is_initiated_ = false;
    bool adjust_system_time_ = true;
    TimerHandle_t adjust_timer_handle_ = nullptr;

    esp_err_t ReadRegister(uint8_t reg_addr, uint8_t* data, size_t len);
    esp_err_t ReadRegisterByte(uint8_t reg_addr, uint8_t* data);
    esp_err_t WriteRegisterByte(uint8_t reg_addr, uint8_t data);

    static const uint8_t kDS3231DeviceAddr          = 0x68;
    static const uint8_t kDS3231RegAddrSeconds      = 0x0;
    static const uint8_t kDS3231RegAddrMinutes      = 0x1;
    static const uint8_t kDS3231RegAddrHours        = 0x2;
    static const uint8_t kDS3231RegAddrDate         = 0x4;
    static const uint8_t kDS3231RegAddrCenturyMonth = 0x5;
    static const uint8_t kDS3231RegAddrYear         = 0x6;

    static const uint8_t kDS3231RegBitStartSeconds    = 0;
    static const uint8_t kDS3231RegBitStartTenSeconds = 4;
    static const uint8_t kDS3231RegBitStartMinutes    = 0;
    static const uint8_t kDS3231RegBitStartTenMinutes = 4;
    static const uint8_t kDS3231RegBitStartIs12Hour   = 6;
    static const uint8_t kDS3231RegBitStartIsPM       = 5;
    static const uint8_t kDS3231RegBitStartHours      = 0;
    static const uint8_t kDS3231RegBitStartTenHours   = 4;
    static const uint8_t kDS3231RegBitStartDate       = 0;
    static const uint8_t kDS3231RegBitStartTenDate    = 4;
    static const uint8_t kDS3231RegBitStartMonth      = 0;
    static const uint8_t kDS3231RegBitStartTenMonth   = 4;
    static const uint8_t kDS3231RegBitStartCentury    = 7;
    static const uint8_t kDS3231RegBitStartYear       = 0;
    static const uint8_t kDS3231RegBitStartTenYear    = 4;

    static const uint8_t kDS3231RegMaskSeconds    = ((1 << 4) - 1) << kDS3231RegBitStartSeconds;
    static const uint8_t kDS3231RegMaskTenSeconds = ((1 << 3) - 1) << kDS3231RegBitStartTenSeconds;
    static const uint8_t kDS3231RegMaskMinutes    = ((1 << 4) - 1) << kDS3231RegBitStartMinutes;
    static const uint8_t kDS3231RegMaskTenMinutes = ((1 << 3) - 1) << kDS3231RegBitStartTenMinutes;
    static const uint8_t kDS3231RegMaskIs12Hour   = ((1 << 1) - 1) << kDS3231RegBitStartIs12Hour;
    static const uint8_t kDS3231RegMaskIsPM       = ((1 << 1) - 1) << kDS3231RegBitStartIsPM;
    static const uint8_t kDS3231RegMaskHours      = ((1 << 4) - 1) << kDS3231RegBitStartHours;
    static const uint8_t kDS3231RegMaskTenHours   = ((1 << 2) - 1) << kDS3231RegBitStartTenHours;
    static const uint8_t kDS3231RegMaskPMTenHours = ((1 << 1) - 1) << kDS3231RegBitStartTenHours;
    static const uint8_t kDS3231RegMaskDate       = ((1 << 4) - 1) << kDS3231RegBitStartDate;
    static const uint8_t kDS3231RegMaskTenDate    = ((1 << 2) - 1) << kDS3231RegBitStartTenDate;
    static const uint8_t kDS3231RegMaskMonth      = ((1 << 4) - 1) << kDS3231RegBitStartMonth;
    static const uint8_t kDS3231RegMaskTenMonth   = ((1 << 1) - 1) << kDS3231RegBitStartTenMonth;
    static const uint8_t kDS3231RegMaskCentury    = ((1 << 1) - 1) << kDS3231RegBitStartCentury;
    static const uint8_t kDS3231RegMaskYear       = ((1 << 4) - 1) << kDS3231RegBitStartYear;
    static const uint8_t kDS3231RegMaskTenYear    = ((1 << 4) - 1) << kDS3231RegBitStartTenYear;

    static const uint16_t kYearFrom = 2020;


    // TODO(yangliang), make the codes about i2c initialization independent, to support more various i2c peripherals
    static const uint64_t kI2CSDAIONum = 25;
    static const uint64_t kI2CSCLIONum = 26;
    static const uint64_t kI2CFreqHZ = 400000;
    static const uint64_t kI2CMasterPort = 0;
    static const uint64_t kI2CMasterTimeoutMs = 3000;

    static const int kTimeZone = 8;  // GMT+8

};  // class RTCDS3231

}  // namespace sd
