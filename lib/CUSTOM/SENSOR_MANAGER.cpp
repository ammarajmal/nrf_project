#ifndef SENSOR_MANAGER_CPP
#define SENSOR_MANAGER_cPP
#include "SENSOR_MANAGER.hpp"

SensorManager::SensorManager() : adxl355(ADXL355_CS_PIN), ads1220(ADS1220_CS_PIN, ADS1220_DRDY_PIN) {
    ADXL355_DRDY = false;
    ADXL355_DATA_COUNTER = 0;
}

static void SensorManager::handleDRDYInterrupt355() {
    // Serial.println("Setting up DRDY Interrupt 355");
    ADXL355_DRDY = true;
    ADXL355_DATA_COUNTER += 1;
    // Serial.print("ADXL355_DRDY: ");
    // Serial.println(ADXL355_DRDY);
    // Serial.print("ADXL355_DATA_COUNTER: ");
    // Serial.println(ADXL355_DATA_COUNTER);
    // Serial.println("DRDY Interrupt 355 set up successfully..!");
}
static void SensorManager::handleDRDYInterrupt1220() {
    // Serial.println("Setting up DRDY Interrupt 1220");
    drdyIntrFlag = true;
    // Serial.print("DRDY Interrupt 1220: ");
    // Serial.println(drdyIntrFlag);
    // Serial.println("DRDY Interrupt 1220 set up successfully..!");
}

void SensorManager::initialize() {
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

float SensorManager::readTemperature() {
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

void SensorManager::readAcceleration(float& xMean, float& yMean, float& zMean, float &tiltX, float &tiltY) {
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

void SensorManager::readStrain(float& strain) {
    float strainSamples[NUM_SAMPLES];
    strSensing(strainSamples, NUM_SAMPLES);

    float strainSum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        strainSum += strainSamples[i];
    }

    strain = strainSum / NUM_SAMPLES;
    }
void SensorManager::DRDYISR(void) {
    if (1) {
        ADXL355_DRDY = true;
        ADXL355_DATA_COUNTER += 1;
    }
}

static void SensorManager::drdyInterruptHndlr() {
    drdyIntrFlag = true;
}


void SensorManager::enableInterruptPin() {
    attachInterrupt(digitalPinToInterrupt(ADS1220_DRDY_PIN), [](){
// Your interrupt handling code here
drdyIntrFlag = true;
}, FALLING);
}

void SensorManager::accSensing(float *xSamples, float *ySamples, float *zSamples, int numSamples) {
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

void SensorManager::strSensing(float* strain, int numSamples) {
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

#endif