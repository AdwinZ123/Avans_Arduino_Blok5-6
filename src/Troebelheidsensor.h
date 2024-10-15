#ifndef TROEBELHEIDSENSOR_H
#define TROEBELHEIDSENSOR_H

class Troebelheidsensor {
private:
    int _pin;
    
  public:
		Troebelheidsensor(int);
		~Troebelheidsensor();
		
    float Meet();
};

#endif