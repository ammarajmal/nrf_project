#ifndef SD_FILE_MANAGER_CPP
#define SD_FILE_MANAGER_CPP
#include "SD_FILE_MANAGER.hpp"
SDFileManager::SDFileManager() : chipSelect(10) {
    Serial.print("Initializing SD Card... ");
    pinMode(chipSelect, OUTPUT);
    if (!SD.begin(chipSelect)) {
        Serial.println("SD initialization failed!");
        Serial.println("Things to check:");
        Serial.println("* is a card inserted?");
        Serial.println("* is your wiring correct?");
        Serial.println("* did you change the chipSelect pin to match your shield or module?");
        while (1);
    } else {
        Serial.println("SD Initialized Successfully..!");
        isInitialized = true;
    }
}
void SDFileManager::write(const char* data, const char* fileName) {
    if (!isInitialized) {
        return;
    }
    file = SD.open(fileName, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return;
    }
    file.println(data);
    file.close();
}
void SDFileManager::read(char *fileName) {
    if (!isInitialized) {
        return;
    }
    file = SD.open(fileName, FILE_READ);
    if (!file) {
        Serial.println("Failed to open file for reading");
        return;
    }
    while (file.available()) {
        Serial.write(file.read());
    }
    file.close();
}

#endif