#ifndef LITTLEFS_MANAGER_HPP
#define LITTLEFS_MANAGER_HPP

#include <Arduino.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;

class LittleFSManager {
public:
    LittleFSManager();
    void initialize();
    // void writeData(const char* filename, const char* data);
};
#endif