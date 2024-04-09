// TemperatureSensor.cpp
#include "TemperatureSensor.h"

TemperatureSensor::TemperatureSensor(int pin) {
  _pin = pin;
  analogReference(AR_INTERNAL_3_0); // Set the reference voltage to 3.0V
  analogReadResolution(12); // Set the ADC resolution to 12 bits
}

float TemperatureSensor::readTemperature() {  
  analogReference(AR_INTERNAL_3_0); // Set the reference voltage to 3.0V
  analogReadResolution(12); // Set the ADC resolution to 12 bits
  int sensorValue = analogRead(A0);
  const float VCC = 3.2; // Supply voltage (e.g., 3.3V)
  const float R_REF = 10000.0; // Resistance of the reference resistor (10kΩ)
  const float R25 = 10000.0; // Resistance of the thermistor at 25°C (10kΩ)
  const float BETA = 3950.0; // Beta value of the thermistor
  float R_thermistor = (R_REF * sensorValue) / (VCC - sensorValue);

  // Calculate the temperature using the Steinhart-Hart equation
  // Note: This is a simplified linear approximation. For better accuracy, use the Steinhart-Hart equation.
  float inv_temperature = 1.0 / (273.15 + 25) + (1.0 / BETA) * log(R_thermistor / R25);
  float temperature = 1.0 / inv_temperature - 273.15; // Convert Kelvin to Celsius



    return temperature;
}
