#ifndef __NEOPIXEL_CPP__
#define __NEOPIXEL_CPP__

#include"neopixel.hpp"
#define NEOPIXELPIN 8
NeoPixel_qr::NeoPixel_qr() {
    // constructor
}
NeoPixel_qr::~NeoPixel_qr() {
    // destructor
}

// Adafruit_NeoPixel onePixel = Adafruit_NeoPixel(1, NEOPIXELPIN, NEO_GRB + NEO_KHZ800);
bool NeoPixel_qr::init() {
    onePixel = Adafruit_NeoPixel(1, NEOPIXELPIN, NEO_GRB + NEO_KHZ800);
    onePixel.begin();
    onePixel.clear();
    onePixel.setPixelColor(0, 200, 0, 0);
    onePixel.setBrightness(10);
    onePixel.show();
    return true;
}
void NeoPixel_qr::setPixelColor(int n, int r, int g, int b) {
    onePixel.clear();
    onePixel.setPixelColor(n, r, g, b);
    onePixel.show();
    delay(400);
}

#endif
