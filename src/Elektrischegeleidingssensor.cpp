#include "Elektrischegeleidingssensor.h"
#include <Arduino.h>
#include <EEPROM.h>
#include "CQRobotTDS.h"

Elektrischegeleidingssensor::Elektrischegeleidingssensor(int pin)
{
  _pin = pin;
}

Elektrischegeleidingssensor::~Elektrischegeleidingssensor() {}

float Elektrischegeleidingssensor::Meet(float temperatuur)
{
  Serial.println("Elektrischegeleidingssensor");

  CQRobotTDS tds(_pin);

  float tdsValue = tds.update(temperatuur);

  Serial.print("TDS value: ");
  Serial.print(tdsValue, 0);
  Serial.println(" ppm");

  return tdsValue;
}