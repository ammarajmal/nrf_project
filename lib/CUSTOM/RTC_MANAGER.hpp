#ifndef RTC_MANAGER_HPP
#define RTC_MANAGER_HPP

#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
//RTC
bool Century = false;
byte year1, month1, date1, hour1, minute1, second1;

class RTCManager {
private:
    RTC_DS3231 rtc;
    char timestampBuffer[32];
public:
    RTCManager();
    void initialize();
    void setAlarmRTC();
    void setAlarmRTC1();
    bool checkAlarm();
    bool checkAlarm1();
    void clearAlarm1();
    void clearAlarm();
    const char* getCurrentTimestamp();
};
#endif