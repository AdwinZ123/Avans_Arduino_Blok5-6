#include "Elektrischegeleidingssensor.h"
#include <Arduino.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include "CQRobotTDS.h"

Elektrischegeleidingssensor::Elektrischegeleidingssensor(int pin) : _tds(pin, 3.3) // Initialiseer _tds in de initializer list
{
  _pin = pin;
}

Elektrischegeleidingssensor::~Elektrischegeleidingssensor() {}

float Elektrischegeleidingssensor::Meet(float temperatuur)
{
  Serial.println("Elektrischegeleidingssensor");

  _tds.setAdcRange(4096);
  float tdsValue = _tds.update(temperatuur);
  float egvValue = tdsValue * 2;

  Serial.print("TDS value: ");
  Serial.print(tdsValue, 0);
  Serial.println(" ppm");
  Serial.print("EGV value: ");
  Serial.print(egvValue, 0);
  Serial.println(" ppm");

  return egvValue;
}