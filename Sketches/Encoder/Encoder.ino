#include "PID_v1.h"
#include "DualVNH5019MotorShield.h"

#define LEFT 0
#define RIGHT 1

#define SPEEDMAX 240 // maximum motor speed

double input;
double output;
double target;
double kp = 2;
double ki = 2;
double kd = 0;

volatile double coderLeft = 0;
volatile double coderRight = 0;

DualVNH5019MotorShield motor;

// initialize PID - goal to stay at zero
PID myPid(&input, &output, &target, kp, ki, kd, DIRECT);

void setup() {
  Serial.begin(9600);
  //pinMode (2, INPUT);
  //pinMode (3, INPUT);
  
  // interrupts start counting ticks
  attachInterrupt(2, LwheelSpeed, CHANGE);    //init the interrupt mode for the digital pin 2
  attachInterrupt(3, RwheelSpeed, CHANGE);   //init the interrupt mode for the digital pin 3

  // initialize pid
  target = 0; // stay forward at zero degrees
  myPid.SetOutputLimits(-90, 90);
  myPid.SetMode(AUTOMATIC); // turn on pid
  
  // initialize motor
  motor.init();
  motor.setSpeeds(SPEEDMAX, SPEEDMAX);
}

void loop() {
  input = coderRight - coderLeft;
  Serial.print(coderRight);
  Serial.print(" ");
  Serial.println(coderLeft);
  Serial.print("Difference: ");
  Serial.println(input);
  
  myPid.Compute();  
  motor.setSpeeds(SPEEDMAX + output, SPEEDMAX - output);

  /*Serial.print("Speed left: ");
  Serial.print(SPEEDMAX + output);
  Serial.print("Speed right: ");
  Serial.println(SPEEDMAX - output);*/
  delay(250);
}

void LwheelSpeed()
{
  coderLeft++;
}


void RwheelSpeed()
{
  coderRight++;
}
