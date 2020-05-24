#include <Control.h>
#include <math.h>

/*Set library function*/
control c;

/*Reference vector*/
double R[2][1];

void setup() {
  pinMode(3,1);             //PWM
  pinMode(4,1);
  pinMode(5,1);             
  pinMode(8,1);             //PWM
  pinMode(9,1);
  pinMode(10,1);  
  Serial.begin(115200);   
}

void loop() {
  /*Proposed signals:
  R[1](t)=+(x^3-2x^2)*(cos(x)+sin(x))
  R[2](t)=-(x^3-2x^2)*(cos(x)+sin(x))*/
  for (float r=0; r<=0.75*PI; r=r+0.01) {
    for (int i=0; i<2; i++) {
      R[i][0]=pow(-1,i)*((pow(r,3)-(2*pow(r,2)))*(cos(r)+sin(r)));
    }
    c.algorithm((double*) R);
  }
}
