
// filename = test.cpp
// author = "Ammar Ajmal"
// date = "2024-04-09"

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <DS3231.h>
#include <ADXL362.h>
#include <ADXL355.h>
#include <ADS1220_WE.h>
#include <RTClib.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Adafruit_SleepyDog.h>

#define ADXL355_CS_PIN    18
#define ADXL355_DRDY_PIN  17
#define ADXL362_CS_PIN    16
#define ADXL355_ADDRESS   0x1D
#define ADS1220_CS_PIN    19
#define ADS1220_DRDY_PIN  5
#define THERMISTOR_PIN    A1
#define REFERENCE_RESISTANCE  10000
#define NOMINAL_RESISTANCE    10000
#define NOMINAL_TEMPERATURE   25
#define B_VALUE   3950
#define NUM_SAMPLES 100


// Define the pin number for the LED
#define LED_PIN 4

volatile bool drdyIntrFlag = false;

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
    bool ADXL355_DRDY;
    volatile int ADXL355_DATA_COUNTER;
    ADXL355 adxl355;
    ADS1220_WE ads1220;


public:
    SensorManager() : adxl355(ADXL355_CS_PIN), ads1220(ADS1220_CS_PIN, ADS1220_DRDY_PIN) {
        ADXL355_DRDY = false;
        ADXL355_DATA_COUNTER = 0;
    }

    void initialize() {
        adxl355.begin();
        ads1220.init();
        ads1220.setDataRate(ADS1220_DR_LVL_0);
        ads1220.setConversionMode(ADS1220_CONTINUOUS);
        enableInterruptPin();
    }

    float readTemperature() {
        analogReference(AR_INTERNAL_3_0);
        analogReadResolution(14);
        int sensorValue = analogRead(THERMISTOR_PIN);
        float voltage = (3.0 / pow(2, 14)) * sensorValue;
        const float R1 = 10000;
        const float B = 3950;
        const float T0 = 298.15;
        const float R0 = 10000;
        float R2 = R1 * (3.29 / voltage - 1.0);
        float T = 1.0 / (1.0 / T0 + log(R2 / R0) / B);
        return T - 273.15;
    }

    void readAcceleration(float& x, float& y, float& z) {
        float xSamples[NUM_SAMPLES], ySamples[NUM_SAMPLES], zSamples[NUM_SAMPLES];
        accSensing(xSamples, ySamples, zSamples, NUM_SAMPLES);

        float xSum = 0, ySum = 0, zSum = 0;
        for (int i = 0; i < NUM_SAMPLES; i++) {
            xSum += xSamples[i];
            ySum += ySamples[i];
            zSum += zSamples[i];
        }

        x = xSum / NUM_SAMPLES;
        y = ySum / NUM_SAMPLES;
        z = zSum / NUM_SAMPLES;
        }

    void readStrain(float& strain) {
        float strainSamples[NUM_SAMPLES];
        strSensing(strainSamples, NUM_SAMPLES);

        float strainSum = 0;
        for (int i = 0; i < NUM_SAMPLES; i++) {
            strainSum += strainSamples[i];
        }

        strain = strainSum / NUM_SAMPLES;
        }
  private:
    void DRDYISR(void) {
        if (1) {
            ADXL355_DRDY = true;
            ADXL355_DATA_COUNTER += 1;
        }
    }

    static void drdyInterruptHndlr() {
        drdyIntrFlag = true;
    }


    void enableInterruptPin() {attachInterrupt(digitalPinToInterrupt(ADS1220_DRDY_PIN), [](){
    // Your interrupt handling code here
    drdyIntrFlag = true;
    }, FALLING);
    }

    void accSensing(float *xSamples, float *ySamples, float *zSamples, int numSamples) {
        memset(xSamples, 0, numSamples * sizeof(float));
        memset(ySamples, 0, numSamples * sizeof(float));
        memset(zSamples, 0, numSamples * sizeof(float));

        for (int i = 0; i < numSamples; i++) {
            float x = 0, y = 0, z = 0;

            while (!ADXL355_DRDY) {
                delay(5);
            }

            digitalWrite(LED_PIN, HIGH);
            adxl355.readXYZ(x, y, z);

            xSamples[i] = x;
            ySamples[i] = y;
            zSamples[i] = z;

            ADXL355_DRDY = false;
            digitalWrite(LED_PIN, LOW);
        }
    }

    void strSensing(float* strain, int numSamples) {
        memset(strain, 0, numSamples * sizeof(float));
        ads1220.setCompareChannels(ADS1220_MUX_0_1);

        for (int i = 0; i < numSamples; i++) {
            while (!drdyIntrFlag) {
                delay(5);
            }

            digitalWrite(LED_PIN, HIGH);
            float result = ads1220.getVoltage_mV();
            strain[i] = result;

            drdyIntrFlag = false;
            digitalWrite(LED_PIN, LOW);
        }
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
  Serial.print("Hello Ammar, I am still Alive. ");
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

