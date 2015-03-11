// Robot Pacman Ghost

// Standard Libraries
#include <Wire.h>
#include "WhizzardMotor.h"
#include "WhizzardUltraSensor.h"
#include "WhizzardLCD.h"

#define MANUAL 1
#define AUTO 0
#define MANUAL_SPEED       200 // set speed for manual control

int autonomous = AUTO; // current robot mode - default manual

// Controller serial commands
const char LEFT = 'L';
const char RIGHT = 'R';
const char CENTRE = 'C';
const char FORWARD = 'F';
const char STOP = 'S';
const char BACKUP = 'B';

// new library declarations
WhizzardMotor wMotors;
WhizzardUltraSensor wSensor;
WhizzardLCD wLcd;

/*
 *------------------------------------------------------------------------------------------
 * Setup Function
 *------------------------------------------------------------------------------------------
 */
void setup()
{
  Serial.begin(9600);
  //wMotors.init();  
  wSensor.init();
  wLcd.init();
  bluetoothInit();
}

void bluetoothInit() {
  while (1) {
    if (Serial.available() > 0) {
      int mode = Serial.parseInt();
      if (mode == 0 || mode == 1) {
        autonomous = mode;
      }
     Serial.println("!");
     Serial.flush();
     break;
    }
  }
}

/*
 *------------------------------------------------------------------------------------------
 *Main Loop Function
 *------------------------------------------------------------------------------------------
*/
void loop() {
  
  /*if (autonomous == AUTO) {
    char newSpeed = wSensor.calculatedApproach(); // calculate distance with ultrasonic sensor
    wLcd.lcdRefresh(wSensor.getDistanceCm()); // display values on LCD display
    if (newSpeed == 0) { // at wall, need to turn left
      wMotors.brake();
      wMotors.turn90Left();
      newSpeed = wSensor.calculatedApproach(); // proceed after turn
    }
    wMotors.forward(newSpeed, newSpeed);
    stringCreate(a, b, encoderRight, encoderLeft, music); // outputs data to controller bluetooth
  } else {  //full manual mode using controller
    while (Serial.available() > 0) {
      char heading = Serial.read();
      manualDrivingMode(heading);
    }
  }*/
}

// Controller driving logic interpreter
void manualDrivingMode(char heading) {
  if (heading == LEFT)
    wMotors.turnLeft(MANUAL_SPEED, MANUAL_SPEED);
  else if (heading == RIGHT)
    wMotors.turnRight(MANUAL_SPEED, MANUAL_SPEED);
  else if (heading == CENTRE)
    wMotors.forward(MANUAL_SPEED, MANUAL_SPEED);
  else if (heading == FORWARD)
    wMotors.forward(MANUAL_SPEED, MANUAL_SPEED);
  else if (heading == STOP)
    wMotors.brake();
  else if (heading == BACKUP)
    wMotors.backward(MANUAL_SPEED, MANUAL_SPEED);
}

/*
 *------------------------------------------------------------------------------------------
 * Bluetooth Communication Functions
 *------------------------------------------------------------------------------------------
 */

// creates a string out of individual motor speeds, encoder values and music selection value
// to send to the controller via bluetooth
void createBluetoothMessage(int speedRight, int speedLeft, int encoderRight, int encoderLeft) {
  String num1 = String(speedRight);
  String num2 = String(speedLeft);
  String num3 = String(encoderRight);
  String num4 = String(encoderLeft);

  String message = num1 + "," + num2 + "," + num3 + "," + num4;
  bluetoothMessage(message);
}

// sends a string via bluetooth from the robot to the controller
void bluetoothMessage(String message) {
  Serial.print(message); // prints message containing motor speeds and encoder values to the serial monitor
}
