#ifndef ADXL355_H
#define ADXL355_H

#include <Arduino.h>
#include <SPI.h>

class ADXL355 {
public:
    ADXL355(uint8_t csPin);
    void begin();
    void readXYZ(float &x, float &y, float &z);

private:
    uint8_t _csPin;
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    void initializeADXL355();
};

#endif // ADXL355_H
