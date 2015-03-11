#include <Wire.h>
#include "PID_v1.h"
#include "PinChangeInt.h"
#include "NewPing.h"
#include "WhizzardLCD.h"

WhizzardLCD wLcd;

// Sonar
#define RANGEFINDER_TRIGGER_PIN  8
#define RANGEFINDER_ECHO_PIN A3
#define TEMPERATURE_PIN A2
#define MAX_DISTANCE       380 // maximum reading distance of ultrasonic sensor in cm
double distanceCm;     // distance read by ultrasonic sensor in cm
int temperature;      // temperature value in degrees C
float soundVelocity;       // intermediate ultrasonic sensor calculation value
volatile float echoPulse;           // time returned from ultrasonic sensor
float soundTime;           // intermediate ultrasonic sensor calculation value
unsigned int pingSpeed = 50; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer = 0;     // Holds the next ping time.
#define wallKp 20 // PID term: dependent on present error
#define wallKi 1
// PID term: accumulation of past error
#define wallKd 1  // PID term: prediction of future error based on current rate of change
double wallSetpoint = 5;

// Motors & Encoders
// TODO these commented out are all wrong, ignoring for now
//#define RIGHT_MOTOR_CONTROL 4 // Right motor Direction Control
//#define RIGHT_MOTOR_SPEED 5 // Right motor Speed Control
//#define LEFT_MOTOR_CONTROL 7 // Left motor Direction Control
//#define LEFT_MOTOR_SPEED 6 // Left motor Speed Control
#define MAX_SPEED 127 // Left motor Speed Control
#define motorKp 20 // PID term: dependent on present error
#define motorKi 5  // PID term: accumulation of past error
#define motorKd 1  // PID term: prediction of future error based on current rate of change
#define MAX_SPEED 127 // maximum PWM motor speed, to allow full-band PID coverage and avoid saturating the loop
volatile unsigned long leftLastTime = 0;
volatile unsigned long rightLastTime = 0;
volatile long leftEncoder = 0;
volatile long rightEncoder = 0;
int leftSpeed;
int rightSpeed;
double motorSpeed = 127; // TODO fix this type mess I made changing this from int to double for wallPID
int lastMotorSpeed;

//Motor PID variables
double motorInput;
double motorOutput = 0;
double lastMotorOutput = 1;
double motorSetpoint = 0; // PID Setpoint

// TODO: comment out to save memory
unsigned long printTimer = 0;

NewPing sonar(RANGEFINDER_TRIGGER_PIN, RANGEFINDER_ECHO_PIN, MAX_DISTANCE); // initialize ultrasonic sensor library
PID wallPID(&distanceCm, &motorSpeed, &wallSetpoint, wallKp, wallKi, wallKd, REVERSE);
PID motorPID(&motorInput, &motorOutput, &motorSetpoint, motorKp, motorKi, motorKd, DIRECT);

void setup() {
  Serial.begin(115200);                            //init the Serial port to print the data

  wLcd.init();
  // Motor Setup
  for (int i = 4; i <= 7; i++) {                 // Motor Pin Assignments
    pinMode(i, OUTPUT);
  }
  attachInterrupt(0, rightISR, CHANGE);  //init interrupt 0 for digital pin 2
  //attachInterrupt(LEFT, leftISR, RISING);   //init interrupt 1 for digital pin 3
  attachPinChangeInterrupt(11, leftISR, CHANGE); // Pin 3 produces interference on Pin 2 so we dug up this software excitement.

  // motor PID controller setup
  motorPID.SetOutputLimits(-127, 127);
  motorPID.SetMode(AUTOMATIC); // pid is driving, to start
  motorPID.SetSampleTime(15); // Recalculate PID every x ms, no more

  // sonar PID controller setup
  wallPID.SetOutputLimits(-127, 127);
  wallPID.SetMode(AUTOMATIC); // pid is driving, to start
  wallPID.SetSampleTime(50); // Recalculate PID every x ms, no more

  // Ultrasound setup
  pinMode(RANGEFINDER_TRIGGER_PIN, OUTPUT);
  pinMode(RANGEFINDER_ECHO_PIN, INPUT);
  pinMode(TEMPERATURE_PIN, INPUT);
  temperature = (500 * analogRead(TEMPERATURE_PIN)) >> 10;
  soundVelocity = (331.3 + (0.6 * temperature)); // speed of sound
}

void loop() {
  //  if ( (printTimer + 100) < millis() ) {

  // Motor serial prints for testing
  //    Serial.print("Encoders: ");
  //    Serial.print(leftEncoder);
  //    Serial.print(" ");
  //    Serial.print(rightEncoder);
  //    Serial.print("       Speeds: ");
  //    Serial.print(leftSpeed);
  //    Serial.print(" ");
  //    Serial.print(rightSpeed);
  //    Serial.print("       Heading: ");
  //    Serial.println(motorSetpoint);
  //    printTimer = millis();

  // Sonar serial prints for testing
  //    Serial.print("Distance: ");
  //    Serial.print(distanceCm);
  //    Serial.print(" cm Motorspeed: ");
  //    Serial.println(motorSpeed);
  //  }

  // turning test function
  //
  //    if ( (turnLastTime + 5000) < millis() ) {
  //      motorSetpoint = motorSetpoint + 10;
  //      turnLastTime = turnLastTime + 5000;
  //    }

  // records the distance to an obstruction
  if (millis() >= pingTimer) {   // pingSpeed milliseconds since last ping, do another ping.
    pingTimer += pingSpeed;      // Set the next ping time.
    sonar.ping_timer(echoCheck); // Send out the ping, calls "echoCheck" function every 24uS where you can check the ping status.
  }
  //wLcd.clearLine(0);
  //wLcd.print(String(distanceCm));
  wLcd.lcdDisplay(distanceCm);
  wLcd.lcdDisplay2(String(motorSpeed));
//  if (distanceCm < 5) {
//    wLcd.lcdDisplay2("Turn, buddy!");
//    while (1) {
//      motorPID.SetMode(MANUAL);
//      motorOutput = 0;
//      wallPID.SetMode(MANUAL);
//      motorSpeed = 0;
//      drive();
//    }
//  }
  //motorSpeed = map(distanceCm, 3, 200, 50, 300); // last value from 127

  // --------PID CONTROL CASE EXAMPLE--------
  // When the encoder difference goes POSITIVE, the right wheel is moving faster.
  // So with DIRECT control, the output will go NEGATIVE to try and control it.
  // Since output is NEGATIVE, we should SUBTRACT it from the leftSpeed to make the left wheel catch up,
  // and ADD output to rightSpeed to slow the right wheel down.
  motorInput = rightEncoder - leftEncoder;
  motorPID.Compute(); // updates PID output
  drive();
}

void leftISR() {
  if ( (millis() - leftLastTime) >= 5 ) {
    leftLastTime = millis();
    if (leftSpeed < 0) { // we're going backwards
      leftEncoder--; // so decrement counter
    } else { // we're going forwards
      leftEncoder++; // so increment counter
    }
  }
}

void rightISR() {
  if ( (millis() - rightLastTime) >= 5 ) {
    rightLastTime = millis();
    if (rightSpeed < 0) { // we're going backwards
      rightEncoder--; // so decrement counter
    } else { // we're going forwards
      rightEncoder++; // so increment counter
    }
  }
}

void drive() {
  boolean motorOutputChanged = (motorOutput != lastMotorOutput);
  boolean motorSpeedChanged = (motorSpeed != lastMotorSpeed);
  if (motorOutputChanged || motorSpeedChanged) {
    lastMotorOutput = motorOutput;
    lastMotorSpeed = motorSpeed;
    leftSpeed = motorSpeed - motorOutput;
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = motorSpeed + motorOutput;
    rightSpeed = constrain(rightSpeed, -255, 255);
    if (rightSpeed >= 0) {
      analogWrite (5, rightSpeed); //PWM Speed Control (0-255)
      digitalWrite(4, LOW); // HIGH = moves forwards
    }
    if (leftSpeed >= 0) {
      analogWrite (6, leftSpeed); //PWM Speed Control (0-255)
      digitalWrite(7, LOW);  // HIGH = moves forwards
    }
    if (rightSpeed < 0) {
      analogWrite (5, abs(rightSpeed)); //PWM Speed Control (0-255)
      digitalWrite(4, HIGH); // HIGH = moves forwards
    }
    if (leftSpeed < 0) {
      analogWrite (6, abs(leftSpeed)); //PWM Speed Control (0-255)
      digitalWrite(7, HIGH);  // HIGH = moves forwards
    }
  }
}

// from https://code.google.com/p/arduino-new-ping/wiki/Ping_Event_Timer_Sketch
void echoCheck() { // Timer2 interrupt calls this function every 24uS where you can check the ping status.
  if (sonar.check_timer()) { // This is how you check to see if the ping was received.
    // TODO: Reduce this to integer calculation using ping_result and temperature only

    // distanceCm = sonar.ping_result / US_ROUNDTRIP_CM;

    echoPulse = float(sonar.ping_result); // returns time to and from object
    soundTime = echoPulse / 2; // divide by two because functions returns twice the time needed
    distanceCm = (soundVelocity * soundTime) / 10000;
    wallPID.Compute();
  }
}
