#ifndef ELEKTRISCHEGELEIDINGSSENSOR_H
#define ELEKTRISCHEGELEIDINGSSENSOR_H

#include <Arduino.h>
#include "CQRobotTDS.h"

class Elektrischegeleidingssensor
{
private:
  int _pin;
  CQRobotTDS _tds;

public:
  Elektrischegeleidingssensor(int);
  ~Elektrischegeleidingssensor();

  float Meet(float temperatuur);
};

#endif