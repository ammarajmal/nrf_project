#ifndef LED_CONTROLLER_HPP
#define LED_CONTROLLER_HPP

#include <Arduino.h>
#define BLUE_LED_PIN 4
#define RED_LED_PIN 13
class LEDController {
public:
    LEDController() {}
    void toggleLED(int n, int delayTime = 1000);
};
#endif
