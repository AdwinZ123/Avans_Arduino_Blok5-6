#include "Phsensor.h"
#include <Arduino.h>
#include <DFRobot_PH.h>

Phsensor::Phsensor(int pin) {
  _pin = pin;
}

Phsensor::~Phsensor() {}

float Phsensor::Meet(float temperatuur) {
  DFRobot_PH ph;
  ph.begin();

  float voltagePH = analogRead(_pin) / 1024.0 * 5000;  // Lees de spanning van de pH-sensor
  float phValue = ph.readPH(voltagePH, temperatuur);              // Zet spanning om naar pH met temperatuur van 25°C

  // Print pH-waarde naar seriële monitor
  Serial.print("pH: ");
  Serial.println(phValue, 2);  // Druk de pH-waarde af met twee decimalen

  return phValue;
}