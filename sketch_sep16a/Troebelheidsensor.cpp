#include "Troebelheidsensor.h"
#include <Arduino.h>

Troebelheidsensor::Troebelheidsensor(int pin) {
  _pin = pin;
}

Troebelheidsensor::~Troebelheidsensor() {}

float Troebelheidsensor::Meet() {
  int sensorValue = analogRead(_pin);// read the input on analog pin 0:
  float voltage = sensorValue * (5.0 / 1024.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  Serial.println(voltage); // print out the value you read:


  // Print de temperatuur naar de seriële monitor
  Serial.print("Troebelheid: ");
  Serial.print(voltage);
  Serial.println(" NTU");

  return temperature;
}