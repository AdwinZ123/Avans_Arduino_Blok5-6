#ifndef PHSENSOR_H
#define PHSENSOR_H

class Phsensor {
private:
    int _pin;
    
  public:
		Phsensor(int);
		~Phsensor();
		
    float Meet(float temperatuur);
};

#endif