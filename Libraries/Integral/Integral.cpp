#include <Integral.h>
#include <Arduino.h>
#include <math.h>

//Variables para Runge - Kutta
double k1, k2, k3, k4;

Integral integral;

//Integral de una función
double Integral::function(double a, double A, double T, double F) {
	k1=a;
	k2=a+(0.5*T*k1);
	k3=a+(0.5*T*k2);
	k4=a+(T*k3);
	A=A+((0.1666666666666*F)*(k1+(2*k2)+(2*k3)+k4)*T);
	return A;
}