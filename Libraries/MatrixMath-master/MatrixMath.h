/*
 *  MatrixMath.h Library for Matrix Math
 *
 *  Created by Charlie Matlack on 12/18/10.
 *  Modified from code by RobH45345 on Arduino Forums, algorithm from
 *  NUMERICAL RECIPES: The Art of Scientific Computing.
 *  Modified to work with Arduino 1.0/1.5 by randomvibe & robtillaart
 *  Made into a real library on GitHub by Vasilis Georgitzikis (tzikis)
 *  so that it's easy to use and install (March 2015)
 */

#ifndef MatrixMath_h
#define MatrixMath_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class MatrixMath
{
public:
	//MatrixMath();
	void Print(double* A, int m, int n, String label);
	void Copy(double* A, int n, int m, double* B);
	void Multiply(double* A, double* B, int m, int p, int n, double* C);
	void Add(double* A, double* B, int m, int n, double* C);
	void Subtract(double* A, double* B, int m, int n, double* C);
	void Transpose(double* A, int m, int n, double* C);
	void Scale(double* A, int m, int n, float k);
	int Invert(double* A, int n);
};

extern MatrixMath Matrix;
#endif
