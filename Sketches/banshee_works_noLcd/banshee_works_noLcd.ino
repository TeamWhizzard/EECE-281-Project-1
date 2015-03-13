#include <Wire.h>
#include "PID_v1.h"
#include "PinChangeInt.h"
#include "NewPing.h"
#include "LCD.h"
#include "LiquidCrystal_I2C.h"

// Ultrasonic constants / declaration
#define RANGEFINDER_TRIGGER_PIN  8
#define RANGEFINDER_ECHO_PIN     A3
#define TEMPERATURE_PIN          A0
#define MAX_DISTANCE             380 // maximum reading distance of ultrasonic sensor in cm

const unsigned int pingSpeed = 50;   // frequency of ping readings in ms. 50ms would be 20 times a second.
unsigned long pingTimer = 0;         // holds the next ping time
volatile float echoPulse;            // time returned from ultrasonic sensor
int temperature;                     // ambient temperature value in degrees C
float soundTime;                     // intermediate ultrasonic sensor calculation value
float soundVelocity;                 // intermediate ultrasonic sensor calculation value

double distanceCm;     // distance read by ultrasonic sensor in cm



// Ultrasonic PID constants / declarations
#define wallKp 22 // PID term: dependent on present error
#define wallKi 50 // PID term: accumulation of past error
#define wallKd 1  // PID term: prediction of future error based on current rate of change
double wallSetpoint = 10;
int leftSpeed;
int rightSpeed;

// Motors & Encoders
#define motorKp 20 // PID term: dependent on present error
#define motorKi 5  // PID term: accumulation of past error
#define motorKd 1  // PID term: prediction of future error based on current rate of change
volatile unsigned long leftLastTime = 0;
volatile unsigned long rightLastTime = 0;
volatile long leftEncoder = 0;
volatile long rightEncoder = 0;

double motorSpeed; // TODO fix this type mess I made changing this from int to double for wallPID
int lastMotorSpeed;

//Motor PID variables
double motorInput;
double motorOutput = 0;
double lastMotorOutput = 1;
double motorSetpoint = 0; // PID Setpoint

// TODO: comment out to save memory
unsigned long printTimer = 0;

// LCD Setup for Testing
// NOTE: These are **NOT** Arduino pins, they are only needed to be delcared so we can use the LCD library.
#define I2C_ADDR  0x27 // I2C Address for LCD Display
#define BACKLIGHT_PIN  3
#define EN_PIN  2
#define RW_PIN  1
#define RS_PIN  0
#define D4_PIN  4
#define D5_PIN  5
#define D6_PIN  6
#define D7_PIN  7

// object declaration
LiquidCrystal_I2C lcd(I2C_ADDR, EN_PIN, RW_PIN, RS_PIN, D4_PIN, D5_PIN, D6_PIN, D7_PIN); // Set the LCD I2C address

NewPing sonar(RANGEFINDER_TRIGGER_PIN, RANGEFINDER_ECHO_PIN, MAX_DISTANCE); // initialize ultrasonic sensor library
PID wallPID(&distanceCm, &motorSpeed, &wallSetpoint, wallKp, wallKi, wallKd, REVERSE);
PID motorPID(&motorInput, &motorOutput, &motorSetpoint, motorKp, motorKi, motorKd, DIRECT);

void setup() {
  Serial.begin(9600);                            //init the Serial port to print the data
  
  // LCD initialization
  lcd.begin(16, 2);        // initialize the lcd for 16 chars 2 lines and turn on backlight
  lcd.home();
  lcd.clear();
  lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
  lcd.setBacklight(HIGH);
  
  // Motor Setup
  for (int i = 4; i <= 7; i++) {                 // Motor Pin Assignments
    pinMode(i, OUTPUT);
  }
  attachInterrupt(0, rightISR, CHANGE);  //init interrupt 0 for digital pin 2
  //attachInterrupt(LEFT, leftISR, RISING);   //init interrupt 1 for digital pin 3  
  attachPinChangeInterrupt(11, leftISR, CHANGE); // Pin 3 produces interference on Pin 2 so we dug up this software excitement.
  
  // PID controller setup
  motorPID.SetOutputLimits(-127, 127);
  motorPID.SetMode(AUTOMATIC); // pid is driving, to start
  motorPID.SetSampleTime(20); // Recalculate PID every x ms, no more
  
  // Ultrasound setup
  pinMode(RANGEFINDER_TRIGGER_PIN, OUTPUT);
  pinMode(RANGEFINDER_ECHO_PIN, INPUT);
  pinMode(TEMPERATURE_PIN, INPUT);
  temperature = (500 * analogRead(TEMPERATURE_PIN)) >> 10;
  soundVelocity = (331.3 + (0.6 * temperature)); // speed of sound
  
  //bluetoothInit();
}

void bluetoothInit() {
  while (1) {
    if (Serial.available() > 0) {
     Serial.println("!");
     Serial.flush();
     break;
    }
  }
}

void testLCD(int rightSpeed, int leftSpeed, int rightEncoder, int leftEncoder) {
    lcd.clear();

    lcd.leftToRight();
    lcd.setCursor(0, 0);
    lcd.print("L: ");

    lcd.rightToLeft();
    lcd.setCursor(6, 0);
    lcd.print(leftSpeed);
    lcd.setCursor(16, 0);
    lcd.print(leftEncoder);

    lcd.leftToRight();
    lcd.setCursor(0, 1);
    lcd.print("R: ");

    lcd.rightToLeft();
    lcd.setCursor(6, 1);
    lcd.print(rightSpeed);
    lcd.setCursor(16, 1);
    lcd.print(rightEncoder);
}

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

    motorSpeed = map(distanceCm, 3, 200, 50, 300); // last value from 127 
  
// --------PID CONTROL CASE EXAMPLE--------    
// When the encoder difference goes POSITIVE, the right wheel is moving faster.
// So with DIRECT control, the output will go NEGATIVE to try and control it.
// Since output is NEGATIVE, we should SUBTRACT it from the leftSpeed to make the left wheel catch up,
// and ADD output to rightSpeed to slow the right wheel down.
  motorInput = rightEncoder - leftEncoder;    
  motorPID.Compute(); // updates PID output 
  boolean motorOutputChanged = (motorOutput != lastMotorOutput);
  boolean motorSpeedChanged = (motorSpeed != lastMotorSpeed);
  if (motorOutputChanged || motorSpeedChanged) {
    leftSpeed = motorSpeed - motorOutput;           
    rightSpeed = motorSpeed + motorOutput;          
    forward(leftSpeed, rightSpeed);
    lastMotorOutput = motorOutput;
    lastMotorSpeed = motorSpeed;
  }
  
  if (distanceCm < (wallSetpoint + 1) && (distanceCm != 0 )) {
    turnLeft();
    int time = millis();
    pingTimer = millis();
    while (millis() < (time + 1000)) {
      if (millis() >= pingTimer) {   // pingSpeed milliseconds since last ping, do another ping.
        pingTimer += pingSpeed;      // Set the next ping time.
        sonar.ping_timer(echoCheck); // Send out the ping, calls "echoCheck" function every 24uS where you can check the ping status.
      }
    }
  }
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(distanceCm);
  //testLCD(rightSpeed, leftSpeed, rightEncoder, leftEncoder);
  createBluetoothMessage(rightSpeed, leftSpeed, rightEncoder, leftEncoder);
}

void leftISR() {
  if ( (millis() - leftLastTime) >= 10 ) {
    leftLastTime = millis();
    leftEncoder++;  //count the left wheel encoder interrupts
  }
}

void rightISR() {
  if ( (millis() - rightLastTime) >= 10 ) {
    rightLastTime = millis();
    rightEncoder++; //count the right wheel encoder interrupts
  }
}

void forward(int leftMotor, int rightMotor) {
  if (rightMotor >= 0) {
    analogWrite (5, rightMotor); //PWM Speed Control (0-255)
    digitalWrite(4, LOW); // HIGH = moves forwards
  }
  if (leftMotor >= 0) {
    analogWrite (6, leftMotor); //PWM Speed Control (0-255)
    digitalWrite(7, LOW);  // HIGH = moves forwards
  }
  if (rightMotor < 0) {
    analogWrite (5, abs(rightMotor)); //PWM Speed Control (0-255)
    digitalWrite(4, HIGH); // HIGH = moves backward
  }
  if (leftMotor < 0) {
    analogWrite (6, abs(leftMotor)); //PWM Speed Control (0-255)
    digitalWrite(7, HIGH);  // HIGH = moves backward
  }
}

void turnLeft() {
   analogWrite (5, 127); //PWM Speed Control (0-255)
   digitalWrite(4, LOW); // HIGH = moves forwards
 
   // left goes backward 
   analogWrite (6, 127); //PWM Speed Control (0-255)
   digitalWrite(7, HIGH);  // HIGH = moves backward
   
 /*rightEncoder = 0;
 leftEncoder = 0;
 
 while((rightEncoder < 12)  && (leftEncoder < 10)) {
     //continue;
 }*/
 //if (leftEncoder < rightEncoder) { // scewed left, less turn
   //delay(650);
 //}else { // scewed right, less turn
   delay(630);
 //}
 
   analogWrite (5, 0); //PWM Speed Control (0-255)
   analogWrite (6, 0); //PWM Speed Control (0-255)
   
   rightEncoder = 0;
   leftEncoder = 0;
}

// from https://code.google.com/p/arduino-new-ping/wiki/Ping_Event_Timer_Sketch
void echoCheck() { // Timer2 interrupt calls this function every 24uS where you can check the ping status.
  if (sonar.check_timer()) { // This is how you check to see if the ping was received.
    // TODO: Reduce this to integer calculation using ping_result and temperature only
    
    // distanceCm = sonar.ping_result / US_ROUNDTRIP_CM;    

    echoPulse = float(sonar.ping_result); // returns time to and from object
    soundTime = echoPulse / 2; // divide by two because functions returns twice the time needed
    distanceCm = (soundVelocity * soundTime) / 10000;
  }
}
