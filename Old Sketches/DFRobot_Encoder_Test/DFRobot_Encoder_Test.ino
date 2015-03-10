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

#include "DualVNH5019MotorShield.h"

#define LEFT 0
#define RIGHT 1

long coder[2] = {0, 0};
int lastSpeed[2] = {0, 0};

static unsigned long timer = 0;                //print manager timer

DualVNH5019MotorShield motors;

void setup() {
  Serial.begin(9600);                            //init the Serial port to print the data
  motors.init();
  
  for (int i = 4; i <= 7; i++) {                 // Motor Pin Assignments
    pinMode(i, OUTPUT);
  }
  attachInterrupt(LEFT, LwheelSpeed, RISING);    //init the interrupt mode for the digital pin 2
  attachInterrupt(RIGHT, RwheelSpeed, RISING);   //init the interrupt mode for the digital pin 3
  delay(100);
  motors.setSpeeds(350, 350);
}

void loop() {
  static unsigned long timer = 0;                //print manager timer

  if(millis() - timer > 100){                   
    Serial.print("Coder value: ");
    Serial.print(coder[LEFT]);
    Serial.print("[Left Wheel] ");
    Serial.print(coder[RIGHT]);
    Serial.println("[Right Wheel]");

    lastSpeed[LEFT] = coder[LEFT];   //record the latest speed value
    lastSpeed[RIGHT] = coder[RIGHT];
    coder[LEFT] = 0;                 //clear the data buffer
    coder[RIGHT] = 0;
    timer = millis();
  }
}


void LwheelSpeed()
{
  coder[LEFT] ++;  //count the left wheel encoder interrupts
}


void RwheelSpeed()
{
  coder[RIGHT] ++; //count the right wheel encoder interrupts
}


