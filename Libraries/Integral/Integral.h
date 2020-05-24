#ifndef Integral_h
#define Integral_h
#include <Arduino.h>

class Integral {
	public:
		double function(double a, double A, double T, double F);		
};
extern Integral integral;
#endif
