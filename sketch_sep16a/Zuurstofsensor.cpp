#include "Zuurstofsensor.h"
#include <Arduino.h>

Zuurstofsensor::Zuurstofsensor(int pin) {
  _pin = pin;
}

Zuurstofsensor::~Zuurstofsensor() {}

float Zuurstofsensor::Meet() {
  // Lees de analoge waarde van de sensor
  int sensorValue = analogRead(_pin);

  // Converteer de analoge waarde naar millivolts (0-1023 naar 0-5000mV)
  float voltage = sensorValue * (5000.0 / 1023.0);

  // Converteer de spanning naar temperatuur in Celsius
  float  temperature = voltage / 10.0;

  // Print de temperatuur naar de seriële monitor
  Serial.print("Temperatuur: ");
  Serial.print(temperature);
  Serial.println(" °C");

  return temperature;
}