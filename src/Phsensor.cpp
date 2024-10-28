#include "Phsensor.h"
#include <Arduino.h>
#include <DFRobot_PH.h>
#include <EEPROM.h>

Phsensor::Phsensor(int pin)
{
  _pin = pin;
}

Phsensor::~Phsensor() {}

float Phsensor::Meet(float temperatuur)
{
  Serial.println("Phsensor");
  DFRobot_PH ph;
  ph.begin();

  float voltage = analogRead(_pin) / 1024.0 * 5000; // read the voltage
  float phValue = ph.readPH(voltage, temperatuur);  // convert voltage to pH with temperature compensation
  // Serial.print("temperature:");
  // Serial.print(temperatuur, 1);
  Serial.print("^C  pH:");
  Serial.println(phValue, 2);

  ph.calibration(voltage, temperatuur); // calibration process by Serail CMD

  return phValue;
}