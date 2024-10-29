#include "Phsensor.h"
#include <Arduino.h>
#include <DFRobot_PH.h>
#include <EEPROM.h>

float voltage, phValue = 25;
DFRobot_PH ph;

Phsensor::Phsensor(int pin)
{
  _pin = pin;
  ph.begin();
}

Phsensor::~Phsensor() {}

float Phsensor::Meet(float temperatuur)
{
  Serial.println("Phsensor");
  voltage = analogRead(_pin)/4096.0*3300; // read the voltage
  phValue = ph.readPH(voltage, temperatuur);  // convert voltage to pH with temperature compensation
  Serial.print("temperature:");
  Serial.print(temperatuur, 1);
  Serial.print("^C  pH:");
  Serial.println(phValue, 2);

  ph.calibration(voltage, temperatuur);

  return phValue;
}