#include "Troebelheidsensor.h"
#include <Arduino.h>
#include <math.h>

Troebelheidsensor::Troebelheidsensor(int pin)
{
  _pin = pin;
}

Troebelheidsensor::~Troebelheidsensor() {}

float Troebelheidsensor::Meet()
{
  Serial.println("Troebelheidsensor");
  int sensorValue = analogRead(_pin);            // read the input on analog pin 0:
  double voltage = sensorValue * (3.3 / 4096.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  double ntu = 600 * voltage;                    // convert measured voltage to corresponding NTU value

  // Print de temperatuur naar de seriële monitor
  Serial.print("Troebelheid: ");
  Serial.print(ntu);
  Serial.println(" NTU");

  return voltage;
}