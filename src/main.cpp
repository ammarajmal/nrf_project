
// filename = test.cpp
// author = "Ammar Ajmal"
// date = "2024-04-09"

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <RTClib.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
// #include "ADXL355.h"

// #include "ADS1220_WE.h"
#include <epd4in2_V2.h>
#include <epdpaint.h>
#include <qrcode.h>

class RTCManager {
private:
    RTC_DS3231 rtc;

public:
    RTCManager() {}

    void initialize() {
        if (!rtc.begin()) {
            Serial.println("Couldn't find RTC");
            Serial.flush();
            while (1);
        }
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    DateTime getCurrentDateTime() {
        return rtc.now();
    }
};

class SDFileManager {
public:
    SDFileManager() {}

    void initialize() {
        if (!SD.begin(10)) {
            Serial.println("SD initialization failed!");
            while (1);
        }
    }

    void writeData(const char* filename, const char* data) {
        File file = SD.open(filename, FILE_WRITE);
        if (file) {
            file.println(data);
            file.close();
            Serial.println("Data written to SD card.");
        } else {
            Serial.println("Error opening file for writing.");
        }
    }
};

class SensorManager {
private:
    // ADXL355 adxl355;
    // ADS1220_WE ads1220;

public:
    SensorManager() {}

    void initialize() {
        // adxl355.begin();
        // ads1220.init();
    }

    float readTemperature() {
        // Your temperature reading logic here
        return 0.0;
    }

    void readAcceleration(float& x, float& y, float& z) {
        // Your accelerometer reading logic here
    }

    void readStrain(float& strain) {
        // Your strain gauge reading logic here
    }
};

// Define classes for LED control, EPD operations, and QR code generation similarly...

class LEDController {
public:
    LEDController() {}

    void toggleLED(int n) {
        // Toggle LED logic here
    }
};

class EPDManager {
public:
    EPDManager() {}

    void initialize() {
        // EPD initialization logic here
    }

    void displayQRCode(const char* qrText) {
        // QR code display logic here
    }
};

// Define other classes...

// Main application class
class SmartSensingSystem {
private:
    RTCManager rtcManager;
    SDFileManager sdFileManager;
    SensorManager sensorManager;
    LEDController ledController;
    EPDManager epdManager;

public:
    SmartSensingSystem() {}

    void initialize() {
        rtcManager.initialize();
        sdFileManager.initialize();
        sensorManager.initialize();
        // Initialize other components...
    }

    void run() {
        // Main application logic here
    }
};

// Create an instance of the main application class
SmartSensingSystem smartSensingSystem;

// RTCManager time_qr;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.print("Hello Ammar ");
  // Serial.println(time_qr.().month(), DEC);
  // Serial.println("Hello World!");

  // watchDogTimerSetup();
}


void loop() {
    // Reset the watchdog timer
    // Watchdog.reset();
    // Serial.println("Working in the loop() !");
    delay(1000);
    // The watchdog timer will reset the device if it is not reset within 8 seconds
    // The device will reset in 8 seconds if the watchdog timer is not reset

}