#include "Zuurstofsensor.h"
#include <Arduino.h>

Zuurstofsensor::Zuurstofsensor(int pin)
{
  _pin = pin;
}

Zuurstofsensor::~Zuurstofsensor() {}

int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
{
  // Single-point calibration Mode=0
  // Two-point calibration Mode=1
  bool TWO_POINT_CALIBRATION = 0;

  // Single point calibration needs to be filled CAL1_V and CAL1_T
  int CAL1_V = 1600; // mv
  int CAL1_T = 25;   // ℃

  // Two-point calibration needs to be filled CAL2_V and CAL2_T
  // CAL1 High temperature point, CAL2 Low temperature point
  int CAL2_V = 1300; // mv
  int CAL2_T = 15;   // ℃

  uint16_t DO_Table[41] = {
      14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
      11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
      9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
      7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};

#if TWO_POINT_CALIBRATION == 0
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}

int Zuurstofsensor::Meet(float temperatuur)
{
  int VREF = 3300;    // 5000;  // VREF (mv)
  int ADC_RES = 4096; // 1024;  // ADC Resolution

  uint8_t Temperaturet = (uint8_t)temperatuur;
  uint16_t ADC_Raw = analogRead(_pin);
  uint16_t ADC_Voltage = (uint32_t)VREF * ADC_Raw / ADC_RES;
  uint16_t DO = readDO(ADC_Voltage, Temperaturet);

  // Optimaliseer seriële output en voeg vertraging toe
  Serial.println("Zuurstofsensor: ");
  Serial.print("Temperaturet:\t");
  Serial.print(Temperaturet);
  Serial.print("\tADC RAW:\t");
  Serial.print(ADC_Raw);
  Serial.print("\tADC Voltage:\t");
  Serial.print(ADC_Voltage);
  Serial.print("\tDO:\t");
  Serial.println(DO);

  return DO;
}