// #
// # Editor     : Lauren from DFRobot
// # Date       : 17.01.2012

// # Product name: Wheel Encoders for DFRobot 3PA and 4WD Rovers
// # Product SKU : SEN0038

// # Description:
// # The sketch for using the encoder on the DFRobot Mobile platform

// # Connection:
// #        left wheel encoder  -> Digital pin 2
// #        right wheel encoder -> Digital pin 3
// #

#include "PID_v1.h"
//#include "DualVNH5019MotorShield.h"

#define RIGHT 0
#define LEFT 1

// Motors
#define RIGHT_MOTOR_CONTROL 4 // Right motor Direction Control
#define RIGHT_MOTOR_SPEED 5 // Right motor Speed Control
#define LEFT_MOTOR_CONTROL 7 // Left motor Direction Control
#define LEFT_MOTOR_SPEED 6 // Left motor Speed Contro

double input;
double output;
double target = 0;
double kp = 2; // dependent on present error
double ki = 5; // accumulation of past error
double kd = 1; // prediction of future error based on current rate of change

volatile long coder[2] = {0, 0};
//int lastSpeed[2] = {0, 0};

int mspeed = 175;

static unsigned long timer = 0;                //print manager timer

PID myPid(&input, &output, &target, kp, ki, kd, DIRECT);

void setup() {
  Serial.begin(9600);                            //init the Serial port to print the data
  
  for (int i = 4; i <= 7; i++) {                 // Motor Pin Assignments
    pinMode(i, OUTPUT);
  }
  attachInterrupt(0, LwheelSpeed, CHANGE);    //init the interrupt mode for the digital pin 2
  attachInterrupt(1, RwheelSpeed, CHANGE);   //init the interrupt mode for the digital pin 3
  
  myPid.SetOutputLimits(-50, 50);
  myPid.SetMode(AUTOMATIC); // pid is driving, to start
}

void loop() {                   
    Serial.print("Encoders: ");
    Serial.print(coder[LEFT]);
    Serial.print(" ");
    Serial.println(coder[RIGHT]);
    Serial.print("Speeds: ");
    Serial.print(leftSpeed);
    Serial.print(" ");
    Serial.println(rightSpreed);
    
    input = coder[RIGHT] - coder[LEFT];
    myPid.Compute(); // updates PID output
    leftSpeed = mspeed + output;
    rightSpeed = mspeed - output;
    forward(leftSpeed, rightSpeed);
    delay(100);
}

void LwheelSpeed() {
  coder[LEFT] ++;  //count the left wheel encoder interrupts
}


void RwheelSpeed() {
  coder[RIGHT] ++; //count the right wheel encoder interrupts
}

void forward(int leftMotor, int rightMotor) {
  analogWrite (5, rightMotor); //PWM Speed Control (0-255)
  digitalWrite(4, HIGH); // HIGH = moves forwards
  analogWrite (6, leftMotor); //PWM Speed Control (0-255)
  digitalWrite(7, HIGH);  // HIGH = moves forwards
}
