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
#include "PinChangeInt.h"

#define RIGHT 0
#define LEFT 1

// Motors
// these defines are all wrong, ignoring for now
#define RIGHT_MOTOR_CONTROL 4 // Right motor Direction Control
#define RIGHT_MOTOR_SPEED 5 // Right motor Speed Control
#define LEFT_MOTOR_CONTROL 7 // Left motor Direction Control
#define LEFT_MOTOR_SPEED 6 // Left motor Speed Control

double input;
double output = 0;
double lastOutput = 1;
double target = 0;
double kp = 20; // dependent on present error
double ki = 5; // accumulation of past error
double kd = 1; // prediction of future error based on current rate of change

volatile unsigned long leftLastTime = 0;
volatile unsigned long rightLastTime = 0;
volatile long leftEncoder = 0;
volatile long rightEncoder = 0;

int mspeed = 127;
volatile int leftSpeed;
volatile int rightSpeed;

unsigned long printTimer = 0;

PID myPid(&input, &output, &target, kp, ki, kd, DIRECT);

void setup() {
  Serial.begin(115200);                            //init the Serial port to print the data
  
  for (int i = 4; i <= 7; i++) {                 // Motor Pin Assignments
    pinMode(i, OUTPUT);
  }
  attachInterrupt(RIGHT, rightISR, CHANGE);  //init interrupt 0 for digital pin 2
  //attachInterrupt(LEFT, leftISR, RISING);   //init interrupt 1 for digital pin 3
  
  attachPinChangeInterrupt(11, leftISR, CHANGE); // Pin 3 produces interference on Pin 2 so we dug up this software excitement.
  
  myPid.SetOutputLimits(-127, 127);
  myPid.SetMode(AUTOMATIC); // pid is driving, to start
  myPid.SetSampleTime(20); // Recalculate PID every 50 ms
}

void loop() {
  if (printTimer <= (millis() + 100) ) {
    Serial.print("Encoders: ");
    Serial.print(leftEncoder);
    Serial.print(" ");
    Serial.print(rightEncoder);
    Serial.print("       Speeds: ");
    Serial.print(leftSpeed);
    Serial.print(" ");
    Serial.println(rightSpeed);
    printTimer = millis();
  }
// --------PID CONTROL CASE EXAMPLE--------    
// When the encoder difference goes POSITIVE, the right wheel is moving faster.
// So with DIRECT control, the output will go NEGATIVE to try and control it.
// Since output is NEGATIVE, we should SUBTRACT it from the leftSpeed to make the left wheel catch up,
// and ADD output to rightSpeed to slow the right wheel down.

    input = rightEncoder - leftEncoder;    
    myPid.Compute(); // updates PID output 
    if (output != lastOutput) {
      lastOutput = output;
      leftSpeed = mspeed - output;           
      rightSpeed = mspeed + output;          
      forward(leftSpeed, rightSpeed);
    }

}

void leftISR() {
  if ( (millis() - leftLastTime) >= 14 ) {
    leftLastTime = millis();
    leftEncoder++;  //count the left wheel encoder interrupts
  }
}


void rightISR() {
  if ( (millis() - rightLastTime) >= 14 ) {
    rightLastTime = millis();
    rightEncoder++; //count the right wheel encoder interrupts
  }
}

void forward(int leftMotor, int rightMotor) {
  analogWrite (5, rightMotor); //PWM Speed Control (0-255)
  digitalWrite(4, LOW); // HIGH = moves forwards
  analogWrite (6, leftMotor); //PWM Speed Control (0-255)
  digitalWrite(7, LOW);  // HIGH = moves forwards
}
