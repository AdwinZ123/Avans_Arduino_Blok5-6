#include "Temperatuursensor.h"
#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>

Temperatuursensor::Temperatuursensor(int pin)
{
  _pin = pin;
}

Temperatuursensor::~Temperatuursensor() {}

float Temperatuursensor::Meet()
{
  OneWire oneWire(_pin);
  DallasTemperature sensors(&oneWire);

  float temperature = 25.0;
  sensors.begin(); // Start de temperatuur sensor

  // Lees de temperatuur
  sensors.requestTemperatures();
  temperature = sensors.getTempCByIndex(0); // Verkrijg de temperatuur van de eerste sensor

  Serial.print("Temperatuur meting: ");
  Serial.print(temperature);
  Serial.println();

  return temperature;
}