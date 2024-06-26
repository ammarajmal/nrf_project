
// filename = test.cpp
// author = "Ammar Ajmal"
// date = "2024-04-09"

#include <Arduino.h>
#include "RTC_MANAGER.hpp"
#include "SD_FILE_MANAGER.hpp"
#include "LITTLEFS_MANAGER.hpp"
#include "LED_CONTROLLER.hpp"

#include <DS3231.h>
#include <ADXL362.h>
#include <ADXL355.h>
#include <ADS1220_WE.h>
#include <Adafruit_LittleFS.h>
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
        ledController.toggleLED(2);
        pinMode(LED_PIN, OUTPUT);
        digitalWrite(LED_PIN, HIGH);

        pinMode(LATCH, OUTPUT);
        digitalWrite(LATCH, HIGH);

        rtcManager.initialize();
        // epdManager.initialize();
        littleFSManager.initialize();
        sensorManager.initialize();
        digitalWrite(LED_PIN, LOW);
        bool latch_state = digitalRead(LATCH);
        while(!latch_state) {
            Serial.println("Latch is not set");
        }
        Serial.println("Latch is set");
        delay(1000);
        }

    void run() {
        ledController.toggleLED(4);
        digitalWrite(LATCH, HIGH);
        delay(10);
        bool latch_state = digitalRead(LATCH);
        Serial.print("Latch is set to :");
        Serial.println(latch_state);
        // run the following code if "rtcManager.checkAlarm1()" and LATCH  is set to high
        if (rtcManager.checkAlarm1() && digitalRead(LATCH)){
            Serial.println("Alarm 1 is set and Latch is high");
        }
        else
        {
            Serial.println("Alarm 1 is not set and Latch is low");
        }


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
            sdFileManager.write(dataString, SD_PATH);
            //
            delay(4000);
            Serial.println("Data written to SD Card..!");
            delay(4000);
            Serial.println("Reading Data from SD Card..!");
            sdFileManager.read(SD_PATH);
            delay(4000);

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
    // qr_sensor_node2.initialize();

}


void loop() {
    // qr_sensor_node2.run();
    delay(10);
    // rtc.clearAlarm1();

    // counting down from 60 to 0
    // for (int i = 60; i > 0; i--) {
    //     Serial.print("Waiting for Alarm: "); Serial.print(i); Serial.println(" seconds");
    //     delay(1000);
    // }
}

