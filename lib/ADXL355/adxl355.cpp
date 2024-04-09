#include "adxl355.h"

ADXL355::ADXL355(uint8_t csPin) : _csPin(csPin) {}

void ADXL355::begin() {
    pinMode(_csPin, OUTPUT);
    SPI.begin();
    initializeADXL355();
}

void ADXL355::readXYZ(float &x, float &y, float &z) {
    long rawX = ((readRegister(0x08) << 12) | (readRegister(0x09) << 4) | (readRegister(0x0A) >> 4));
    long rawY = ((readRegister(0x0B) << 12) | (readRegister(0x0C) << 4) | (readRegister(0x0D) >> 4));
    long rawZ = ((readRegister(0x0E) << 12) | (readRegister(0x0F) << 4) | (readRegister(0x10) >> 4));

    // Apply two's complement if necessary
    if (rawX & 0x80000) rawX -= (1 << 20);
    if (rawY & 0x80000) rawY -= (1 << 20);
    if (rawZ & 0x80000) rawZ -= (1 << 20);

    // Convert raw data to acceleration (g's)
    x = rawX * 0.0000039;
    y = rawY * 0.0000039;
    z = rawZ * 0.0000039;
}

void ADXL355::initializeADXL355() {
    writeRegister(0x2D, 0x01); // Standby mode
    writeRegister(0x2C, 0x01); // +/-2g range
    writeRegister(0x28, 0x05); // Set output data rate to 1000 Hz
    writeRegister(0x2D, 0x00); // Measurement mode
    delay(100); // Allow settings to take effect
}

void ADXL355::writeRegister(uint8_t reg, uint8_t value) {
    digitalWrite(_csPin, LOW);
    SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));
    SPI.transfer(reg << 1 | 0x00); // Write operation
    SPI.transfer(value);
    SPI.endTransaction();
    digitalWrite(_csPin, HIGH);
}

uint8_t ADXL355::readRegister(uint8_t reg) {
    digitalWrite(_csPin, LOW);
    SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));
    SPI.transfer(reg << 1 | 0x01); // Read operation
    uint8_t value = SPI.transfer(0x00); // Dummy byte for reading
    SPI.endTransaction();
    digitalWrite(_csPin, HIGH);
    return value;
}
