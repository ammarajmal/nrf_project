#ifndef LED_CONTROLLER_CPP
#define LED_CONTROLLER_CPP
#include "LED_CONTROLLER.hpp"
void LEDController::toggleLED(int n, int delayTime = 1000) {
    // Toggle LED logic
    for (int i = 0; i < n; i++) {
        digitalWrite(BLUE_LED_PIN, HIGH);
        delay(delayTime/2);
        digitalWrite(BLUE_LED_PIN, LOW);
        delay(delayTime/2);
    }
}
#endif
