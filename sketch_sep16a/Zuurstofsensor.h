#ifndef ZUURSTOFSENSOR_H
#define ZUURSTOFSENSOR_H

class Zuurstofsensor {
private:
    int _pin;
    
  public:
		Zuurstofsensor(int);
		~Zuurstofsensor();
		
    int16_t Meet(float temperatuur);
};

#endif