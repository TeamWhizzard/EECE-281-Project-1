#include "WhizzardMotor.h"
#include "PID_v1.h"
#include "PinChangeInt.h"

PID myPid(&input, &output, &target, kp, ki, kd, DIRECT);

double input;      // PID input
double output = 0; // PID output
double lastOutput = 1;
double target = 0; // PID setpoint
double kp = 20;    // PID term: dependent on present error
double ki = 5;     // PID term: accumulation of past error
double kd = 1;     // PID term: prediction of future error based on current rate of change

volatile unsigned long leftLastTime = 0;
volatile unsigned long rightLastTime = 0;

volatile long leftEncoder = 0;
volatile long rightEncoder = 0;

WhizzardMotor::WhizzardMotor() { /*constructor*/ }

// TODO cleanup later, pin 4 right direction pin
// TODO cleanup later, pin 7 left direction pin

// motor initialization
void WhizzardMotor::init() {
  for (int i = 4; i <= 7; i++) { // Motor Pin Assignments
    pinMode(i, OUTPUT);
  }

  myPid.SetOutputLimits(-127, 127);
  myPid.SetMode(AUTOMATIC); // pid is driving, to start
  myPid.SetSampleTime(20); // Recalculate PID every x ms
  
  attachInterrupt(RIGHT, rightISR, CHANGE);  //init interrupt 0 for digital pin 2
  //attachInterrupt(LEFT, leftISR, RISING);   //init interrupt 1 for digital pin 3  
  attachPinChangeInterrupt(11, leftISR, CHANGE); // Pin 3 produces interference on Pin 2 so we dug up this software excitement.

}

// --------PID CONTROL CASE EXAMPLE--------    
// When the encoder difference goes POSITIVE, the right wheel is moving faster.
// So with DIRECT control, the output will go NEGATIVE to try and control it.
// Since output is NEGATIVE, we should SUBTRACT it from the leftSpeed to make the left wheel catch up,
// and ADD output to rightSpeed to slow the right wheel down.

// stops both motors
void WhizzardMotor::brake() {
  drive(0,0);
}

void WhizzardMotor::forward(int velocity) {
  digitalWrite(4, LOW); // LOW = moves forward
  digitalWrite(7, LOW); // LOW = moves forward
  drive(velocity);  
}

void WhizzardMotor::backward(int velocity) {
  digitalWrite(4, HIGH); // LOW = moves forward
  digitalWrite(7, HIGH); // LOW = moves forward

}

void WhizzardMotor::turnRight(int velocity) {
  digitalWrite(4, HIGH); // LOW = moves forward
  digitalWrite(7, LOW);  // LOW = moves forward

}

void WhizzardMotor::turnLeft(int velocity) {
  digitalWrite(4, LOW);  // LOW = moves forward
  digitalWrite(7, HIGH); // LOW = moves forward

}

//void WhizzardMotor::turn90Left() {}

// moves both motors
// speed 0 to 255, forward and backward is set by the calling function
void WhizzardMotor::drive(int velocity) {
  input = rightEncoder - leftEncoder;    
  myPid.Compute(); // updates PID output 
  if (output != lastOutput) { // TODO: this should really check to see if velocity changed, too.
    lastOutput = output;
    analogWrite (5, velocity + output); //Right motor, PWM Speed Control (0-255)
    analogWrite (6, velocity - output); //Left motor, PWM Speed Control (0-255)
  }
}
