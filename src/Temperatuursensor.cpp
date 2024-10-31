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

   unsigned char testByteArray[8] = {0x8C, 0x07, 0x00, 0x64, 0x0F, 0x00, 0x14, 0x00};

    // Output the byte array
    Serial.println("____________________________________");
    Serial.println(testByteArray[0]);
    Serial.println(testByteArray[1]);
    Serial.println(testByteArray[2]);
    Serial.println(testByteArray[3]);
    Serial.println(testByteArray[4]);
    Serial.println(testByteArray[5]);
    Serial.println(testByteArray[6]);
    Serial.println(testByteArray[7]);
    Serial.println("____________________________________");



  return temperature;
}