#ifndef RTC_MANAGER_CPP
#define RTC_MANAGER_CPP
#include "RTC_MANAGER.hpp"
RTCManager::RTCManager() {}
void RTCManager::initialize() {
    if (!rtc.begin()) {
        Serial.println("Couldn't find RTC");
        while (1) delay(10); // Hang if RTC not found
    }
    Serial.println("RTC Initialized Successfully");
    setAlarmRTC1();  // Set the initial alarm
}
void RTCManager::setAlarmRTC() {
    DateTime now = rtc.now();
    // Set alarm to trigger at the start of the next minute
    rtc.setAlarm2(DateTime(now.year(), now.month(), now.day(), now.hour(), now.minute() + 1), DS3231_A2_PerMinute);
    rtc.clearAlarm(2);  // Clear any pending alarm
}
void RTCManager::setAlarmRTC1() {
    DateTime now = rtc.now();
    // Set alarm to trigger at the start of the next minute
    rtc.setAlarm1(DateTime(now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second() + 10), DS3231_A1_Second);
    rtc.clearAlarm(1);  // Clear any pending alarm
}
bool RTCManager::checkAlarm() {
    return rtc.alarmFired(2);
}
bool RTCManager::checkAlarm1() {
    return rtc.alarmFired(1);
}
void RTCManager::clearAlarm1() {
    rtc.clearAlarm(1);
}

void RTCManager::clearAlarm() {
    rtc.clearAlarm(2);
}
const char* RTCManager::getCurrentTimestamp() {
    DateTime now = rtc.now();

    // Format the timestamp and store it in the timestampBuffer array
    snprintf(timestampBuffer, sizeof(timestampBuffer), "%02d%02d%02d%02d%02d%02d",
                now.year()%100, now.month(), now.day(), now.hour(), now.minute(), now.second());

    return timestampBuffer; // Return the pointer to the timestampBuffer array
}
#endif