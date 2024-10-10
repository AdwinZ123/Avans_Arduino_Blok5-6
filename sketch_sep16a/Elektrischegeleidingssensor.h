#ifndef ELEKTRISCHEGELEIDINGSSENSOR_H
#define ELEKTRISCHEGELEIDINGSSENSOR_H

class Elektrischegeleidingssensor {
private:
    int _pin;
    
  public:
		Elektrischegeleidingssensor(int);
		~Elektrischegeleidingssensor();
		
    float Meet();
};

#endif