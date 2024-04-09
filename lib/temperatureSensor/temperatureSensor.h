// TemperatureSensor.h
#ifndef TemperatureSensor_h
#define TemperatureSensor_h

#include "Arduino.h"

class TemperatureSensor {
  public:
    TemperatureSensor(int pin);
    float readTemperature();
  private:
    int _pin;
    const float _T0 = 298.15; // Reference temperature in Kelvin (25 degrees Celsius)
    const float _B1 = 3435; // B-parameter of the thermistor
    const float _R0 = 10000; // Reference resistance at T0 (10k ohms)
    
};

#endif
