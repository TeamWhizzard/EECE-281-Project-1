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
#include "DualVNH5019MotorShield.h"

#define LEFT 0
#define RIGHT 1

double input;
double output;
double target;
double kp = 2; // dependent on present error
double ki = 5; // accumulation of past error
double kd = 1; // prediction of future error based on current rate of change

//volatile long coder[2] = {0, 0};
volatile long coder[2] = {0, 0}; // set left motor to -206 to offset shield triggering interrupt
int lastSpeed[2] = {0, 0};

int mspeed = 350;

bool delayFlag = 1;

static unsigned long timer = 0;                //print manager timer

DualVNH5019MotorShield motors;

PID myPid(&input, &output, &target, kp, ki, kd, DIRECT);

void setup() {
  Serial.begin(9600);                            //init the Serial port to print the data
  motors.init();
  
  for (int i = 4; i <= 7; i++) {                 // Motor Pin Assignments
    pinMode(i, OUTPUT);
  }
  attachInterrupt(0, LwheelSpeed, CHANGE);    //init the interrupt mode for the digital pin 2
  attachInterrupt(1, RwheelSpeed, CHANGE);   //init the interrupt mode for the digital pin 3
  
  myPid.SetOutputLimits(-90, 90);
  myPid.SetMode(AUTOMATIC); // turn on pid
  
  target = 0; // stay forward at zero degrees
}

void loop() {
  if (delayFlag) {
    if (millis() > 10000) { // start PID once noise has settled
      coder[LEFT] = 0;
      coder[RIGHT] = 0;
      motors.setSpeeds(mspeed, mspeed);
      delayFlag = 0;
    }
  }
  
  //static unsigned long timer = 0;                //print manager timer

  //if(millis() - timer > 100){                   
    Serial.print("Coder value: ");
    Serial.print(coder[LEFT]);
    Serial.print("[Left Wheel] ");
    Serial.print(coder[RIGHT]);
    Serial.print("[Right Wheel]");
    Serial.print(mspeed - output);
    Serial.print("[Left Wheel] ");
    Serial.print(mspeed + output);
    Serial.println("[Right Wheel]");
    
    input = coder[RIGHT] - coder[LEFT];
    
    //myPid.Compute();
    //motors.setSpeeds(mspeed - output, mspeed + output);

    //lastSpeed[LEFT] = coder[LEFT];   //record the latest speed value
    //lastSpeed[RIGHT] = coder[RIGHT];
    //coder[LEFT] = 0;                 //clear the data buffer
    //coder[RIGHT] = 0;
    //timer = millis();
  //}
}

void LwheelSpeed()
{
  coder[LEFT] ++;  //count the left wheel encoder interrupts
}


void RwheelSpeed()
{
  coder[RIGHT] ++; //count the right wheel encoder interrupts
}


