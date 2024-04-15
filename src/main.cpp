
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
#include "NEOPIXEL.hpp"
#include "epd4in2_V2.h"
#include "epdpaint.h"
using namespace Adafruit_LittleFS_Namespace;
//SENSOR ID
#define NODEID "N02 "

//ADC

#define PGA 32                // Programmable Gain, confirm that the same as set_pga_gain
#define VREF 2.048            // Internal reference of 2.048V
#define VFSR VREF/PGA
#define FSR (((long int)1<<23)-1)
#define ADS1220_CS_PIN    19
#define ADS1220_DRDY_PIN  5

#define COLORED      0
#define UNCOLORED    1

//DEFAULT
#define ST_WAIT 1500        // ms
#define DATA_RATE 1         // ksps, internal
#define TIM_Waitms(...)   delay(__VA_ARGS__)
#define SUCCESS true
#define ERROR false
#define LED_PIN 4

//EPD
#define QRversion    17

//RTC
bool Century = false;
byte year1, month1, date1, hour1, minute1, second1;

//POWER
#define LATCH 2 //LATCH
//ADXL
#define ADXL355_CS_PIN    18
#define ADXL355_DRDY_PIN    17
#define ADXL362_CS_PIN    16
#define ADXL355_ADDRESS 0x1D
bool ADXL355_DRDY=0;
volatile int ADXL355_DATA_COUNTER=0;
ADXL362 adxl362;
ADXL355 adxl355(ADXL355_CS_PIN);
int NUM_SAMPLES=100; //1 min 60s * 125Hz
//SD
//#define DEV  SD
int linesCount = 0; //LFS saved Line Count
#define SDCARD_SS_PIN 10
//LFS FILE NAME
#define FLAG_PATH "init6.txt"
#define DATA_PATH "DATATEMP6.txt"
#define SD_PATH "NODE02.txt"

// resistance values and parameters for the thermistor
#define THERMISTOR_PIN A1 // Analog pin connected to the thermistor
#define REFERENCE_RESISTANCE 10000
#define NOMINAL_RESISTANCE 10000
#define NOMINAL_TEMPERATURE 25
#define B_VALUE 3950

//EDS setting
int adxl_thresh = 120;
uint8_t NumOfEvent = 1;
uint8_t DayOfEvent = 2;
int SensorID = 21;
//DS3231 Clock;
RTC_DS3231 rtc;

//ADC

ADS1220_WE ads1220 = ADS1220_WE(ADS1220_CS_PIN, ADS1220_DRDY_PIN);

volatile bool drdyIntrFlag = false;


//ADXL362
extern void adxl362_init(int adxl_act_thresh, int adxl_act_interval);
//LED
NeoPixel_qr nRFBoard_neopixel;

Adafruit_LittleFS_Namespace::File file(InternalFS);
Adafruit_LittleFS lfs;
//EPD
Epd epd;
//SAVING
char resultString[200]; // Adjust the size based on the expected length of the message



class RTCManager {
private:
    RTC_DS3231 rtc;
    char timestampBuffer[32];


public:
    RTCManager() {}

    void initialize() {
        if (!rtc.begin()) {
            Serial.println("Couldn't find RTC");
            while (1) delay(10); // Hang if RTC not found
        }
        Serial.println("RTC Initialized Successfully");
        setAlarmRTC1();  // Set the initial alarm
    }

    void setAlarmRTC() {
        DateTime now = rtc.now();
        // Set alarm to trigger at the start of the next minute
        rtc.setAlarm2(DateTime(now.year(), now.month(), now.day(), now.hour(), now.minute() + 1), DS3231_A2_PerMinute);
        rtc.clearAlarm(2);  // Clear any pending alarm
    }
    void setAlarmRTC1() {
        DateTime now = rtc.now();
        // Set alarm to trigger at the start of the next minute
        rtc.setAlarm1(DateTime(now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second() + 10), DS3231_A1_Second);
        rtc.clearAlarm(1);  // Clear any pending alarm
    }

    bool checkAlarm() {
        return rtc.alarmFired(2);
    }
    bool checkAlarm1() {
        return rtc.alarmFired(1);
    }
    void clearAlarm1() {
        rtc.clearAlarm(1);
    }

    void clearAlarm() {
        rtc.clearAlarm(2);
    }
    const char* getCurrentTimestamp() {
        DateTime now = rtc.now();

        // Format the timestamp and store it in the timestampBuffer array
        snprintf(timestampBuffer, sizeof(timestampBuffer), "%02d%02d%02d%02d%02d%02d",
                 now.year()%100, now.month(), now.day(), now.hour(), now.minute(), now.second());

        return timestampBuffer; // Return the pointer to the timestampBuffer array
    }


};

class SDFileManager {
private:
    SDLib::File myFile;

public:
    SDFileManager() {}

    void initialize() {
        Serial.print("Initializing SD Card... ");
        if (!SD.begin(10)) {
            Serial.println("SD initialization failed!");
            while (1);
        }
        else {
            Serial.println("SD Initialized Successfully..!");
        }
    }

    void writeData(const char* filename, const char* data) {
        if (!SD.exists(filename)) {
            Serial.println("File does not exist. Creating file...");
            myFile = SD.open(filename, FILE_WRITE);
        } else {
            Serial.println("File exists. Opening file for data writing...");
            myFile = SD.open(filename, FILE_WRITE);
        }
        if (myFile) {
            myFile.println(data);
            myFile.flush();
            myFile.close();
            delay(100);
            Serial.print("File size after writing: "); Serial.println(myFile.size());
            Serial.println("Data written to SD card successfully.");
        } else {
            Serial.println("Failed to open file for writing.");
        }
    }

    void readData(const char* filename) {
        if (SD.exists(filename)) {
            Serial.println("Reading data from SD card");
            myFile = SD.open(filename, FILE_READ);
            while (myFile.available()) {
                Serial.write(myFile.read());
            }
            myFile.close();
            delay(100);
            Serial.println("Data read from SD card successfully.");
        }
        else {
            Serial.println("File does not exist. Cannot read data.");
        }
    }
};

class LittleFSManager {
public:
    LittleFSManager() {}

    void initialize() {
        Serial.print("Initializing LITTLEFS... ");
        if (!InternalFS.begin()) {
            Serial.println("LittleFS initialization failed!");
            while (1);
        }
        Serial.println("LITTLEFS Initialized Successfully..!");
    }

    // void writeData(const char* filename, const char* data) {
    //     File file = InternalFS.open(filename, FILE_WRITE);
    //     if (file) {
    //         file.println(data);
    //         file.close();
    //         Serial.println("Data written to LittleFS.");
    //     } else {
    //         Serial.println("Error opening file for writing.");
    //     }
    // }
};

class SensorManager {
  private:
    static bool ADXL355_DRDY;
    static volatile int ADXL355_DATA_COUNTER;
    ADXL355 adxl355;
    ADS1220_WE ads1220;


public:
    SensorManager() : adxl355(ADXL355_CS_PIN), ads1220(ADS1220_CS_PIN, ADS1220_DRDY_PIN) {
        ADXL355_DRDY = false;
        ADXL355_DATA_COUNTER = 0;
    }

    static void handleDRDYInterrupt355() {
        // Serial.println("Setting up DRDY Interrupt 355");
        ADXL355_DRDY = true;
        ADXL355_DATA_COUNTER += 1;
        // Serial.print("ADXL355_DRDY: ");
        // Serial.println(ADXL355_DRDY);
        // Serial.print("ADXL355_DATA_COUNTER: ");
        // Serial.println(ADXL355_DATA_COUNTER);
        // Serial.println("DRDY Interrupt 355 set up successfully..!");
    }
    static void handleDRDYInterrupt1220() {
        // Serial.println("Setting up DRDY Interrupt 1220");
        drdyIntrFlag = true;
        // Serial.print("DRDY Interrupt 1220: ");
        // Serial.println(drdyIntrFlag);
        // Serial.println("DRDY Interrupt 1220 set up successfully..!");
    }

    void initialize() {
        Serial.print("Initializing SENSORS... ");
        adxl355.begin();

        
        ads1220.init();
        ads1220.setDataRate(ADS1220_DR_LVL_0);
        ads1220.setConversionMode(ADS1220_CONTINUOUS);
        // attachInterrupt(digitalPinToInterrupt(ADXL355_DRDY_PIN), handleDRDYInterrupt, RISING);
        // Serial.println("now attaching the interrupts..!");
        attachInterrupt(digitalPinToInterrupt(ADXL355_DRDY_PIN),handleDRDYInterrupt355, RISING);
        attachInterrupt(digitalPinToInterrupt(ADS1220_DRDY_PIN), handleDRDYInterrupt1220, FALLING);
        Serial.println("SENSORS Initialized Successfully..!");
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

    void readAcceleration(float& xMean, float& yMean, float& zMean, float &tiltX, float &tiltY) {
        float xSamples[NUM_SAMPLES], ySamples[NUM_SAMPLES], zSamples[NUM_SAMPLES];
        accSensing(xSamples, ySamples, zSamples, NUM_SAMPLES);

        float xSum = 0, ySum = 0, zSum = 0;
        for (int i = 0; i < NUM_SAMPLES; i++) {
            xSum += xSamples[i];
            ySum += ySamples[i];
            zSum += zSamples[i];
        }

        xMean = xSum / NUM_SAMPLES;
        yMean = ySum / NUM_SAMPLES;
        zMean = zSum / NUM_SAMPLES;

        tiltX = atan2(xMean, sqrt(yMean * yMean + zMean * zMean)) * (180.0 / PI);
        tiltY = atan2(yMean, sqrt(xMean * xMean + zMean * zMean)) * (180.0 / PI);

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


    void enableInterruptPin() {
        attachInterrupt(digitalPinToInterrupt(ADS1220_DRDY_PIN), [](){
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
            
            // Serial.print("X: ");
            // Serial.print(x, 6);
            // Serial.print(" G, Y: ");
            // Serial.print(y, 6);
            // Serial.print(" G, Z: ");
            // Serial.print(z, 6);
            // Serial.println(" G");

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
bool SensorManager::ADXL355_DRDY = false;
volatile int SensorManager::ADXL355_DATA_COUNTER = 0;
// Define classes for LED control, EPD operations, and QR code generation similarly...

class LEDController {
public:
    LEDController() {}

    void toggleLED(int n, int delayTime = 1000) {
        // Toggle LED logic
        for (int i = 0; i < n; i++) {
            digitalWrite(LED_PIN, HIGH);
            delay(delayTime/2);
            digitalWrite(LED_PIN, LOW);
            delay(delayTime/2);
        }
    }
};

class EPDManager {
private:
    unsigned char image[((EPD_WIDTH * EPD_HEIGHT) / 8)] = {0};
    Paint paint = Paint(image, EPD_WIDTH, EPD_HEIGHT);
    // Paint paint(image, EPD_WIDTH, EPD_HEIGHT);

public:
    EPDManager() {}

    void initialize() {
        paint.Clear(UNCOLORED);
        Serial.print("Initializing EPD... ");
        if (epd.Init() != 0) {
            Serial.println("Failed to initialize EPD.");
            return;
        }
        epd.Clear();
        Serial.println("EPD Initialized Successfully..!");

        
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
    EPDManager epdManager;
    SensorManager sensorManager;
    SDFileManager sdFileManager;
    LittleFSManager littleFSManager;
    LEDController ledController;

    char dataString[32];

public:
    SmartSensingSystem() {}

    void initialize() {
        // Initialize components
        pinMode(LED_PIN, OUTPUT);
        digitalWrite(LED_PIN, HIGH);
        rtcManager.initialize();
        // epdManager.initialize();
        sdFileManager.initialize();
        littleFSManager.initialize();
        sensorManager.initialize();
        digitalWrite(LED_PIN, LOW);
    }

    void run() {
        if (rtcManager.checkAlarm1()) {

            rtcManager.clearAlarm1();
            // ledController.toggleLED(3);
            /*
            1. Initialze the littleFS and get the file ready for data saving.
            2. Get the acceleration data from the ADXL355 sensor, in the form of tiltX, and tiltY.
            3. Get the temperature data from the thermistor.
            4. Get the time data from the RTC.
            5. Construct the data string in the format: "TIMESTAMP, TILT_X, TILT_Y, TEMPERATURE".
            6. Save the data to the SD card.
            

            */

            //    2. Getting the acceleration data
            float xMean = 0, yMean = 0, zMean = 0, tiltX = 0, tiltY = 0;
            sensorManager.readAcceleration(xMean, yMean, zMean, tiltX, tiltY);
            Serial.print("X Mean: "); Serial.print(xMean, 6);
            Serial.print(" G, Y Mean: "); Serial.print(yMean, 6);
            Serial.print(" G, Z Mean: "); Serial.print(zMean, 6); Serial.println(" G");
            Serial.print("Tilt X: "); Serial.print(tiltX, 6); Serial.print(" degrees, Tilt Y: "); Serial.print(tiltY, 6); Serial.println(" degrees");

            //    3. Getting the temperature data
            float temperature = sensorManager.readTemperature();
            Serial.print("Temperature: "); Serial.print(temperature, 6); Serial.println(" °C");

            //    4. Getting the time data
            const char* formatedTimeStamp = rtcManager.getCurrentTimestamp();
            Serial.print("Current Date & Time: "); Serial.println(formatedTimeStamp);
            Serial.println("--------------------------------------------------");

            //   5. Constructing the data string
            int j = snprintf(dataString, sizeof(dataString),"%s,%.2f,%.2f,%.1f/", formatedTimeStamp, tiltX, tiltY, temperature);
            Serial.println(dataString);
            // Serial.print("Data String Length: "); Serial.print(j);  Serial.println(" bytes");
            Serial.println("--------------------------------------------------");

            //    6. Saving the data to the SD card and LittleFS
            sdFileManager.writeData(SD_PATH, dataString);
            sdFileManager.readData(SD_PATH);

            rtcManager.setAlarmRTC1();
        }
    }
};

// Create an instance of the main application class
SmartSensingSystem qr_sensor_node2;

// RTCManager time_qr;
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial.println("-----------------------------");
    Serial.println("STARTING QR SENSOR NODE 2...!");
    Serial.println("-----------------------------");
    Serial.println("Setup()");
    qr_sensor_node2.initialize();

}


void loop() {
    qr_sensor_node2.run();
    delay(10);
}

