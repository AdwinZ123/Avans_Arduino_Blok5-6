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

  float voltage, phValue = 25.0;

  // Lees de temperatuur
  // sensors.requestTemperatures();
  // temperature = sensors.getTempCByIndex(0); // Verkrijg de temperatuur van de eerste sensor

  // Lees de pH-sensor
  voltage = analogRead(_pin) / 1024.0 * 5000; // Lees de spanning
  phValue = ph.readPH(voltage, temperatuur);  // Converteer de spanning naar pH met temperatuurcompensatie

  // Print de temperatuur en pH-waarde naar de seriële monitor
  Serial.print("Temperatuur: ");
  Serial.print(temperatuur, 1);
  Serial.print(" °C  pH: ");
  Serial.println(phValue, 2);

  /// Kalibratieproces via seriële commando's
  if (Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');
    if (cmd == "enterph")
    {
      Serial.println("In kalibratiemodus...");
    }
    else if (cmd == "calph")
    {
      ph.calibration(voltage, temperatuur); // Start en voltooi de kalibratie
      Serial.println("Kalibreren...");
    }
    else if (cmd == "exitph")
    {
      Serial.println("Kalibratie opgeslagen en modus verlaten.");
    }
  }

  // float voltage = analogRead(_pin) / 1024.0 * 5000; // read the voltage
  // float phValue = ph.readPH(voltage, temperatuur);  // convert voltage to pH with temperature compensation
  // // Serial.print("temperature:");
  // // Serial.print(temperatuur, 1);
  // Serial.print("^C  pH:");
  // Serial.println(phValue, 2);

  // ph.calibration(voltage, temperatuur); // calibration process by Serail CMD

  return phValue;
}