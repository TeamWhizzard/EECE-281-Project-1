// Robot Pacman Ghost

// Standard Libraries
#include <Wire.h>
#include "WhizzardMotor.h"
#include "WhizzardUltraSensor.h"
#include "WhizzardLCD.h"
#include "PinChangeInt.h"
#include "PID_v1.h"

#define MANUAL 1
#define AUTO 0
#define MANUAL_SPEED       200 // set speed for manual control

int autonomous = AUTO; // current robot mode - default manual

volatile unsigned long leftLastInterrupt = 0; // timekeepers for interrupt debounce
volatile unsigned long rightLastInterrupt = 0;
volatile unsigned long leftEncoder = 0; // encoder count values
volatile unsigned long rightEncoder = 0;

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
  attachInterrupt(0, rightEncoderISR, CHANGE);  //init interrupt 0 for digital pin 2
  attachPinChangeInterrupt(11, leftEncoderISR, CHANGE); // Pin 3 produces interference on Pin 2 so we dug up this software excitement.
  wMotors.init();
  motorInterruptInit(); 
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
  
  if (autonomous == AUTO) {
    int newSpeed = wSensor.calculatedApproach(); // calculate distance with ultrasonic sensor
    wLcd.lcdRefresh(wSensor.getDistanceCm()); // display values on LCD display
    if (newSpeed == 0) { // at wall, need to turn left
      wMotors.brake();
      wMotors.turn90Left();
      newSpeed = wSensor.calculatedApproach(); // proceed after turn
    }
    wMotors.forward(newSpeed);
    //createBluetoothMessage(int speedRight, int speedLeft, int encoderRight, int encoderLeft); // outputs data to controller bluetooth
  } else {  //full manual mode using controller
    while (Serial.available() > 0) {
      char heading = Serial.read();
      manualDrivingMode(heading);
    }
  }
}

// Controller driving logic interpreter
void manualDrivingMode(char heading) {
  if (heading == LEFT)
    wMotors.turnLeft(MANUAL_SPEED);
  else if (heading == RIGHT)
    wMotors.turnRight(MANUAL_SPEED);
  else if (heading == CENTRE)
    wMotors.forward(MANUAL_SPEED);
  else if (heading == FORWARD)
    wMotors.forward(MANUAL_SPEED);
  else if (heading == STOP)
    wMotors.brake();
  else if (heading == BACKUP)
    wMotors.backward(MANUAL_SPEED);
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

/*
 *------------------------------------------------------------------------------------------
 * Encoder ISRs
 *------------------------------------------------------------------------------------------
 */
 // can't attach methods as interrupts, so these ISRs call the methods... OBJECT ORIENTATION!
 // this makes the interrupt longer, but we don't have time to TODO refactor this right now.
 
void leftEncoderISR() {
  if ( (millis() - leftLastInterrupt) >= 10 ) { // debounce the interrupt
    leftLastInterrupt = millis();
    leftEncoder++; // increment the counter
  }
}

void rightEncoderISR() {
  if ( (millis() - rightLastInterrupt) >= 10 ) { // debounce the interrupt
    rightLastInterrupt = millis();
    rightEncoder++; // increment the counter
  }
}

/*
 *------------------------------------------------------------------------------------------
 * Motor Control Functions
 *------------------------------------------------------------------------------------------
 */ //OBJECT ORIENTED!


double input;      // PID input
double output = 0; // PID output
double lastOutput = 1;
double target = 0; // PID setpoint
double kp = 20;    // PID term: dependent on present error
double ki = 5;     // PID term: accumulation of past error
double kd = 1;     // PID term: prediction of future error based on current rate of change

PID myPid(&input, &output, &target, kp, ki, kd, DIRECT);

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
  
  // these interrupt bases live in ghost.ino now, because we're
  // OBJECT ORIENTED!
  // attachInterrupt(0, rightBaseISR, CHANGE);  //init interrupt 0 for digital pin 2
  // attachInterrupt(LEFT, leftISR, RISING);   //init interrupt 1 for digital pin 3  
  // attachPinChangeInterrupt(11, leftBaseISR, CHANGE); // Pin 3 produces interference on Pin 2 so we dug up this software excitement.
}

// --------PID CONTROL CASE EXAMPLE--------    
// When the encoder difference goes POSITIVE, the right wheel is moving faster.
// So with DIRECT control, the output will go NEGATIVE to try and control it.
// Since output is NEGATIVE, we should SUBTRACT it from the leftSpeed to make the left wheel catch up,
// and ADD output to rightSpeed to slow the right wheel down.

// stops both motors
void brake() {
  drive(0);
}

void forward(int velocity) {
  digitalWrite(4, LOW); // LOW = moves forward
  digitalWrite(7, LOW); // LOW = moves forward
  drive(velocity);  
}

void backward(int velocity) {
  digitalWrite(4, HIGH); // LOW = moves forward
  digitalWrite(7, HIGH); // LOW = moves forward
  drive(velocity);
}

void turnRight(int velocity) {
  digitalWrite(4, HIGH); // LOW = moves forward
  digitalWrite(7, LOW);  // LOW = moves forward
  drive(velocity);
}

void turnLeft(int velocity) {
  digitalWrite(4, LOW);  // LOW = moves forward
  digitalWrite(7, HIGH); // LOW = moves forward
  drive(velocity);
}

void turn90Left() {
// TODO
}

// sets both motor voltages: 0 to 255 gives 0 to 5 volts
// forward and backward is set by the calling function
void drive(int velocity) {
  input = rightEncoder - leftEncoder;
  myPid.Compute(); // updates PID output 
  if (output != lastOutput) { // TODO: this should really check to see if velocity changed, too.
    lastOutput = output;
    analogWrite (5, velocity + output); //Right motor, PWM Speed Control (0-255)
    analogWrite (6, velocity - output); //Left motor, PWM Speed Control (0-255)
  }
}
