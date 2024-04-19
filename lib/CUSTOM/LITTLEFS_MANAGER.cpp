#ifndef LITTLEFS_MANAGER_CPP
#define LITTLEFS_MANAGER_CPP

#include "LITTLEFS_MANAGER.hpp"
void LittleFSManager::initialize() {
    Serial.print("Initializing LITTLEFS... ");
    if (!InternalFS.begin()) {
        Serial.println("LittleFS initialization failed!");
        while (1);
    }
    Serial.println("LITTLEFS Initialized Successfully..!");
}
// void LittleFSManager::writeData(const char* filename, const char* data) {
//     File file = InternalFS.open(filename, FILE_WRITE);
//     if (file) {
//         file.println(data);
//         file.close();
//         Serial.println("Data written to LittleFS.");
//     } else {
//         Serial.println("Error opening file for writing.");
//     }
// }
#endif