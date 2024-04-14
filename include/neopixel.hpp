#ifndef NEOPIXEL_HPP
#define NEOPIXEL_HPP

#include <Adafruit_NeoPixel.h>
class NeoPixel_qr {
    public:
        NeoPixel_qr();
        ~NeoPixel_qr();
        bool init();
        void setPixelColor(int n, int r, int g, int b);
    private:
        Adafruit_NeoPixel onePixel;
};
#endif
