#include <Arduino.h>
#include <Control.h>
#include <math.h>
#include <MatrixMath.h>

#define N 2

double Xm[N][1];
double ErrorO[N][1];
double L[N][N];
double Xo[N][1];
double Desired[N][1];
double ErrorR[N][1];
double K[N][N];
double V[N][1];
double Iv[N][1];
double U[N][1];
double T[1][N];
double z[N][1];
double Z[N][N];

control::control(){
}

void control::algorithm(double* R) {
	/*Measured entries: Xm(t)*/
	for (int i=0; i<N; i++) {
		Xm[i][0]=(analogRead(i)*1.5*PI/1023)-0.75*PI;;
	}
	
	/*Observer error: ErrorO(t)=Xm(t)-Xo(t)*/
	Matrix.Subtract((double*) Xm, (double*) Xo, N, 1, (double*) ErrorO);
	
	/*Adaptive observer gain: L'(t)=ErrorO*(ErrorO)^T*/
	Matrix.Transpose((double*) ErrorO, N, 1, (double*) T);
	Matrix.Multiply((double*) ErrorO, (double*) T, N, 1, N, (double*) Z);
	Matrix.Add((double*) L, (double*) Z, N, N, (double*) L);
	
	/*Observer: Xo'(t)=L(t)*ErrorO(t)*/
	Matrix.Multiply((double*) L, (double*) ErrorO, N, N, 1, (double*) z);
	Matrix.Add((double*) Xo, (double*) z, N, 1, (double*) Xo);
	
	/*Check if there's any unusual value in L(t) or Xo(t)*/
	for (int i=0; i<N; i++) {
		for (int j=0; j<N; j++) {
			if (abs(ErrorO[i][0])<0.001 || isnan(Xo[i][0])==1 || isinf(Xo[i][0])==1 || isnan(L[i][j])==1 || isinf(L[i][j])==1) {
				L[i][j]=0;
				Xo[i][0]=Xm[i][0];
			}
		}
	}
	
	/*Reference error: ErrorR=R(t)-Xo(t)*/
	Matrix.Subtract((double*) R, (double*) Xo, N, 1, (double*) ErrorR);
	
	/*Adaptive control input gain: K'(t)=ErrorR*(ErrorR)^T*/
	Matrix.Transpose((double*) ErrorR, N, 1, (double*) T);
	Matrix.Multiply((double*) ErrorR, (double*) T, N, 1, N, (double*) Z);
	Matrix.Add((double*) K, (double*) Z, N, N, (double*) K);
	
	/*Active control input: V=K(t)*ErrorR(t)*/
	Matrix.Multiply((double*) K, (double*) ErrorR, N, N, 1, (double*) V);
	
	/*Discrete integral for active control input V(t): Iv=Iv+V(t)*/
	Matrix.Add((double*) Iv, (double*) V, N, 1, (double*) Iv);
	
	/*Unmodeled dynamics control input: Ud(t)=Iv-Xo*/
	Matrix.Subtract((double*) Iv, (double*) Xo, N, 1, (double*) z);
	
	/*General control input: U(t)=Ud(t)+V(t)*/
	Matrix.Add((double*) z, (double*) V, N, 1, (double*) U);
	
	/*Check if there's any unusual value in K(t) or V(t)*/
	for (int i=0; i<N; i++) {
		for (int j=0; j<N; j++) {
			if (abs(ErrorR[i][0])<0.001 || isnan(V[i][0])==1 || isinf(V[i][0])==1 || isnan(Iv[i][0])==1 || isinf(Iv[i][0])==1 || isnan(K[i][j])==1 || isinf(K[i][j])==1) {
				K[i][j]=0;
				V[i][0]=0;
				Iv[i][0]=0;
			}
		}
	}
	
	/*U(t) converted into PWM signal*/
	for (int i=0; i<N; i++) {
		if (U[i][0]>0.75*PI){
			U[i][0]=0.75*PI;
		}
		else if (U[i][0]<-0.75*PI) {
			U[i][0]=-0.75*PI;
		}
		U[i][0]=map(U[i][0],0,0.75*PI,0,255);
	}
	for (int i=0; i<N; i++) {
		if (U[i][0]<0) {
			U[i][0]=-1*U[i][0];
			digitalWrite(4+(5*i),0);
			digitalWrite(5+(5*i),1);
		}
		else {
			digitalWrite(5+(5*i),0);
			digitalWrite(4+(5*i),1);
		}
		analogWrite(3+(5*i),U[i][0]);
	}
	
	for (int i=0; i<N; i++) {
		Iv[i][0]=0;
		Serial.print(R[i]);
		Serial.print(",");
		Serial.print(Xm[i][0]);
		Serial.print(",");
		Serial.print(Xo[i][0]);
		Serial.print(",");
	}
	Serial.println();
}