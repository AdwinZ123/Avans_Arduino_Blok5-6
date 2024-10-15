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
  int sensorValue = analogRead(_pin);                                        // read the input on analog pin 0:
  double voltage = sensorValue * (5.0 / 1024.0);                             // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  double ntu = round(-1120.4 * pow(voltage, 2) + 5742.3 * voltage - 4353.8); // convert measured voltage to corresponding NTU value

  // int sensorValue = analogRead(_pin);// read the input on analog pin 0:
  // float voltage = sensorValue * (5.0 / 1024.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  //  Serial.println(voltage); // print out the value you read:

  // Print de temperatuur naar de seriële monitor
  Serial.print("Troebelheid: ");
  Serial.print(ntu);
  Serial.println(" NTU");

  return voltage;
}