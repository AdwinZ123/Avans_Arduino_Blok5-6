#ifndef TEMPERATUURSENSOR_H
#define TEMPERATUURSENSOR_H

class Temperatuursensor{
  private:
    int _pin;
    
  public:
		Temperatuursensor(int);
		~Temperatuursensor();
		
    float Meet();
};

#endif