
// filename = test.cpp
// author = "Ammar Ajmal"
// date = "2024-04-09"

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <Adafruit_SleepyDog.h>
#include <RTClib.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>


void watchDogTimerSetup();
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) delay(10);
  // watchDogTimerSetup();
}
void watchDogTimerSetup() {
    Serial.println("Watchdog Library Demo!");
    // Set the watchdog timer to 8 seconds
    // This is the maximum time the watchdog timer can be set to
    // The watchdog timer can be set to 16ms, 32ms, 64ms, 125ms, 250ms, 500ms, 1s, 2s, 4s, 8s
    int countdownMS = Watchdog.enable(4000);
    Serial.print("enabled watchdog with maximum countdown of :");
    Serial.print(countdownMS, DEC);
    Serial.println(" ms");
    Serial.println("Watchdog timer is now running!");
    Serial.println("The device will reset in 4 seconds if the watchdog timer is not reset!");
    Serial.println();
    for (int i = 0; i < 10; i++) {
      Serial.print("Loop # ");
      Serial.println(i, DEC);
      delay(1000);
      // reset watchdog with every loop to make sure the program keeps on running.
      Watchdog.reset();
    }
    Serial.println();

}
void toggle(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
}
void loop() {
    // Reset the watchdog timer
    // Watchdog.reset();
    Serial.println("Working in the loop() !");
    delay(1000);
    // The watchdog timer will reset the device if it is not reset within 8 seconds
    // The device will reset in 8 seconds if the watchdog timer is not reset

}