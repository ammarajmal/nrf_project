// #include <Arduino.h>
// #include <Adafruit_TinyUSB.h>
// #include <SPI.h>
// #include <SD.h>

// #define RED_LED 13
// #define BLUE_LED 4

// //SENSOR ID
// #define NODEID "N02 "
// class SDFileManager {
// private:
//     const int chipSelect;
//     SDLib::File file;
//     bool isInitialized;
// public:
//     SDFileManager() : chipSelect(10), file(), isInitialized(false) {
//         pinMode(chipSelect, OUTPUT);

//         if (!SD.begin(chipSelect)) {
//             Serial.println("SD initialization failed!");
//             return;
//         }
//         isInitialized = true;
//     }
//     bool write(const char* data, const char* fileName) {
//         if (!isInitialized) {
//             return false;
//         }
//         file = SD.open(fileName, FILE_WRITE);
//         if (!file) {
//             Serial.println("Failed to open file for writing");
//             return false;
//         }
//         file.println(data);
//         file.close();
//         return true;
//     }
//     bool read(char *fileName) {
//         if (!isInitialized) {
//             return false;
//         }
//         file = SD.open(fileName, FILE_READ);
//         if (!file) {
//             Serial.println("Failed to open file for reading");
//             return false;
//         }
//         while (file.available()) {
//             Serial.write(file.read());
//         }
//         file.close();
//         return true;
//     }


// };






// void setup() {
//     Serial.begin(115200);
//     while (!Serial) {
//         delay(10);
//         }
//     pinMode(BLUE_LED, OUTPUT);
//     pinMode(RED_LED, OUTPUT);
//     digitalWrite(BLUE_LED, HIGH);
//     delay(1000);
//     digitalWrite(BLUE_LED, LOW);
//     delay(500);
//     digitalWrite(RED_LED, HIGH);
//     SDFileManager fileManager;
//     char fileName[13] = "data.txt";
//     if (fileManager.write("AMMAR AJMAL HERE!", fileName)) {
//         Serial.println("Data written successfully!");
//     } else {
//         Serial.println("Failed to write data!");
//     }
//     digitalWrite(RED_LED, LOW);
//     delay(500);
//     digitalWrite(BLUE_LED, HIGH);
//     if (fileManager.read(fileName)) {
//         Serial.println("Data read successfully!");
//     } else {
//         Serial.println("Failed to read data!");
//     }
//     digitalWrite(BLUE_LED, LOW);
//     delay(500);


// }
// void loop() {
//     delay(1000);
    
// }
