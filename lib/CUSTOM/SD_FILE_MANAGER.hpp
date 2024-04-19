#ifndef SD_FILE_MANAGER_HPP
#define SD_FILE_MANAGER_HPP

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

class SDFileManager {
private:
    const int chipSelect;
    SDLib::File file;
    bool isInitialized;
public:
    SDFileManager();
    void write(const char* data, const char* fileName);
    void read(char *fileName);
};
#endif