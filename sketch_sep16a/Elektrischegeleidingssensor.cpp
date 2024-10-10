#include "Elektrischegeleidingssensor.h"
#include <Arduino.h>
#include <EEPROM.h>
#include "GravityTDS.h"

Elektrischegeleidingssensor::Elektrischegeleidingssensor(int pin) {
  _pin = pin;
}

Elektrischegeleidingssensor::~Elektrischegeleidingssensor() {}

float Elektrischegeleidingssensor::Meet(float temperatuur) {
  GravityTDS gravityTds;
  gravityTds.setPin(_pin);
  gravityTds.setAref(5.0);       //reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin();            //initialization

  tdsValue = 0;

  gravityTds.setTemperature(temperatuur);  // set the temperature and execute temperature compensation
  gravityTds.update();                     //sample and calculate
  tdsValue = gravityTds.getTdsValue();     // then get the value
  Serial.print(tdsValue, 0);
  Serial.println("ppm");

  return tdsValue;
}