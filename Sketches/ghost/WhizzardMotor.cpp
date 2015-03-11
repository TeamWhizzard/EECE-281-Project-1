#include "WhizzardMotor.h"
#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield motor;

WhizzardMotor::WhizzardMotor() { /*constructor*/ }

// motor initialization
void WhizzardMotor::init() {
  for (int i = 4; i <= 7; i++) { // Motor Pin Assignments
    pinMode(i, OUTPUT);
  }
  
  motor.init();
}

// stops both motors
void WhizzardMotor::brake() {
  motor.setBrakes(400, 400);
}

// moves both motors forward
void WhizzardMotor::forward(int left, int right) {
  motor.setSpeeds(left, right);
}

// move both motors backward
void WhizzardMotor::backward(int left, int right) {
  motor.setSpeeds(left, right);
}

//turns right
void WhizzardMotor::turnRight(int left, int right) {
  motor.setSpeeds(left, right);
}

//turns left
void WhizzardMotor::turnLeft(int left, int right) {
  motor.setSpeeds(left, right);
}

// turns the robot 90 degrees to the left
void WhizzardMotor::turn90Left() {
  turnRight(150, 150);
  delay(1000);
  brake();
}
