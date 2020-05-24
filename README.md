# **zController v1.0**
## Controller for Uncertain Dynamic Systems based only on its feedback data

Dynamic systems' behaviour can be unpredictable, and sometimes chaotic, if they control inputs are not correctly defined.

Despite the fact that there exists a lot of control functions, such as the well known PID, these cannot deal with uncertainty in most of the cases, or they have to be constantly tuned in order to guarantee stability.

However, with this controller is possible to achieve convergence for almost any dynamic system that can be measured and that allows to receive a control input.

**Because of the algorithm's restrictions, this do not intend to be an universal controller for all the dynamic systems, but it can be a good starting point for designing better controllers** :smile:

This algorithm is divided in two parts:
 - [Arduino's code](https://github.com/dzentenol/zController/blob/master/Joints/Joints.ino "Joints.ino"), where you'll be able to find the main program that contains the desired signal to be followed by the dynamic system
 - Necessary Libraries ([own developed](https://github.com/dzentenol/zController/tree/master/Libraries "Control & Integral libraries") and [third - party one](https://github.com/eecharlie/MatrixMath "Math Matrix library by Charlie Matlack & Vasilis Georgitzikis"))

## Example of Use

### Let's control a DC Motor whose position can be measured by a single input and will be controlled to follow a defined trajectory

0. Import the provided libraries to your Arduino environment (more information [here](https://www.arduino.cc/en/guide/libraries#toc2 "How to Install a Library? - Arduino.cc"))

1. Define the number of states to be controlled by [Joints.ino](https://github.com/dzentenol/zController/blob/master/Joints/Joints.ino "Main Arduino Code") as follows

```cpp
/*Reference vector, where "n" refers to the number of states of the system. In this case, n = 1*/
double R[n][1];
```

2. Establish the controller outputs in your Arduino microcontroller

```cpp
void setup() {
  /* These will be used in a L293D Integrated Circuit*/
  pinMode(3,1);             // PWM
  pinMode(4,1);             // Movement direction in L293D (motor 1)
  pinMode(5,1);             // Movement direction in L293D (motor 1)
  Serial.begin(115200);   
}
```
**NOTE:** It has to be considered that the number of states to be measured has to be consistent with the type of measure (analog or digital) and with the microcontroller's inputs that are available for that purpose

3. Go to [Control.cpp](https://github.com/dzentenol/zController/blob/master/Libraries/Control/Control.cpp "Control main library") and change the following value

```cpp
#define N n
```

with the one that you defined previously. In this case, n = 1

4. Define the reference signal to be followed by the dynamic system and send it to the control algorithm

```cpp
  for (float r=0; r<=0.75*PI; r=r+0.01) {
    R[1][0]=pow(-1,n)*((pow(r,3)-(2*pow(r,2)))*(cos(r)+sin(r)));
    c.algorithm((double*) R);
  }
```

5. Connect your circuit to your Arduino microcontroller. As an example, a DC motor was connected mechanically to a 10kΩ (see schematics below)

![Example of Electronic Circuit](/Diagrams/Circuit.png)
