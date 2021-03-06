// Robot Pacman Ghost
// Standard Libraries
//#include <math.h>
#include <Wire.h>

int autonomous = 0;

// Controller serial commands
const char LEFT = 'L';
const char RIGHT = 'R';
const char CENTRE = 'C';
const char FORWARD = 'F';
const char FORWARD_SPEED_2 = '6';
const char FORWARD_SPEED_1 = '5';
const char STOP = 'S';
const char BACKUP = 'B';

// LCD
// NOTE: These are **NOT** Arduino pins, they are only needed to be delcared for so we can use the LCD library.
//#include "LCD.h"
#include "LiquidCrystal_I2C.h"
#define I2C_ADDR 0x27 // I2C Address for LCD Display
#define BACKLIGHT_PIN 3
#define EN_PIN 2
#define RW_PIN 1
#define RS_PIN 0
#define D4_PIN 4
#define D5_PIN 5
#define D6_PIN 6
#define D7_PIN 7
#define BLOCK 0xFF // block character

// Motors
#define DIRECTION_CONTROL_M1 4 // Right motor Direction Control
#define RIGHT_MOTOR_SPEED 5 // Right motor Speed Control
#define DIRECTION_CONTROL_M2 7 // Left motor Direction Control
#define LEFT_MOTOR_SPEED 6 // Left motor Speed Control

// Sensor Inputs
#include "NewPing.h"
#define RANGEFINDER_TRIGGER_PIN 8
#define RANGEFINDER_ECHO_PIN A3
#define TEMPERATURE_PIN A2

// General Constants
#define MAX_DISTANCE 380 // maximum reading distance of ultrasonic sensor in cm
#define MAX_SPEED 350 // maximum PWM speed
#define FIRST_GEAR 200 // set speed for manual control speed 1
#define SECOND_GEAR 250 // set speed for manual control speed 2
#define THIRD_GEAR 350 // set speed for manual control speed 3
#define TURN_SPEED 200 // set speed for turning while in manual control

// Global Variables
int temperature; // temperature value in C
float echoPulseFront; // time returned from ultrasonic sensor
float velocity; // intermediate ultrasonic sensor calculation
float time; // intermediate ultrasonic sensor calculation
float distanceCm; // distance read by ultrasonic sensor
int distanceLast; // keeps track of last moisture reading
int music = 0; // assigned the values 0, 1, or 2 to play a song depending on the value

// array of custom characters that each represent a different amount of lines filled
// in a single block on an lcd. Broken into 5 segments
byte slices[6][8] = {
  {B00000, B00000, B00000, B00000, B00000, B00000, B00000, B00000},
  {B10000, B10000, B10000, B10000, B10000, B10000, B10000, B10000},
  {B11000, B11000, B11000, B11000, B11000, B11000, B11000, B11000},
  {B11100, B11100, B11100, B11100, B11100, B11100, B11100, B11100},
  {B11110, B11110, B11110, B11110, B11110, B11110, B11110, B11110},
  {B11111, B11111, B11111, B11111, B11111, B11111, B11111, B11111}
};
// collision detection threshold approach speeds and distances
char approachWallSpeeds[6] = {300, 200, 150, 100, 50, 0};
int approachWallThreshold[6] = {50, 35, 25, 15, 10, 3};

// Create input/output objects
NewPing sonar(RANGEFINDER_TRIGGER_PIN, RANGEFINDER_ECHO_PIN, MAX_DISTANCE); // initialize ultrasonic sensor
LiquidCrystal_I2C lcd(I2C_ADDR, EN_PIN, RW_PIN, RS_PIN, D4_PIN, D5_PIN, D6_PIN, D7_PIN); // Set the LCD I2C address
/*
*------------------------------------------------------------------------------------------
* Setup Function
*------------------------------------------------------------------------------------------
*/
void setup()
{
  Serial.begin(9600);
  for (int i = 4; i <= 7; i++) { // Motor Pin Assignments
    pinMode(i, OUTPUT);
  }
  pinMode(RANGEFINDER_TRIGGER_PIN, OUTPUT);
  pinMode(RANGEFINDER_ECHO_PIN, INPUT);
  pinMode(TEMPERATURE_PIN, INPUT);
  lcd.begin(16, 2); // initialize the lcd for 16 chars 2 lines and turn on backlight
  lcd.home();
  lcd.clear();
  lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
  lcd.setBacklight(HIGH);
  delay(100);
  for (int i = 0; i < 6; i++) {
    lcd.createChar(i, slices[i]);
  }
  recordTemp();
}
/*
*------------------------------------------------------------------------------------------
* Measurements Functions
*------------------------------------------------------------------------------------------
*/
// records the temperature and calculates the speed of sound
void recordTemp() {
  int data = analogRead(TEMPERATURE_PIN);
  temperature = (500 * data) >> 10;
  velocity = (331.3 + (0.6 * temperature)); // speed of sound
}
// records the distance to an obstruction
void reportDistance() {
  echoPulseFront = float(sonar.ping_median()); // returns time to and from object
  time = (echoPulseFront) / 2; // divide by two because functions returns twice the time needed
  distanceCm = (velocity * time) / 10000;
}
/*
*------------------------------------------------------------------------------------------
* Bluetooth Communication Functions
*------------------------------------------------------------------------------------------
*/
// creates a string out of individual motor speeds, encoder values and music selection value
// to send to the controller via bluetooth
void stringCreate(int speedRight, int speedLeft, int encoderRight, int encoderLeft, int music) {
  String num1 = String(speedRight);
  String num2 = String(speedLeft);
  String num3 = String(encoderRight);
  String num4 = String(encoderLeft);
  String num5 = String(music);
  String message = num1 + "," + num2 + "," + num3 + "," + num4 + "," + num5;
  bluetoothData(message);
}
// sends a string via bluetooth from the robot to the controller
void bluetoothData(String message) {
  Serial.print(message); // prints message containing motor speeds and encoder values to the serial monitor
}
/*
*------------------------------------------------------------------------------------------
*LCD Control Functions
*------------------------------------------------------------------------------------------
*/
//clears a given line on the lcd
void clearLine(int line) {
  lcd.setCursor(0, line);
  lcd.print("                ");
  delay(20);
  lcd.setCursor(0, line);
}
//Displays the distance measurement on the top line of the LCD
void lcdDisplay() {
  lcd.setCursor(0, 0);
  lcd.print("Distance: ");
  distanceCm /= 100; // convert from centimeters to meters
  // Displays "Oodles" when the distance recorded is zero since the range sensor is limited to 4 meters
  if (distanceCm == 0) {
    lcd.print(" ");
    lcd.setCursor(10, 0);
    lcd.print("Oodles");
  }
  // Displays the number value of the distance recorded
  else {
    lcd.setCursor(14, 0);
    lcd.print(" ");
    lcd.setCursor(10, 0);
    lcd.print(distanceCm);
    lcd.setCursor(15, 0);
    lcd.print("m");
  }
}
// distanceLive() - displays bar representation of most recent distance reading on bottom line of LCD display
void distanceLive(int distanceNew) {
  // graphs values 13cm and less
  if (distanceCm <= 13) {
    // we assume that when distanceCm equals zero, that the range is beyond 4meters (outside the range of the sensor)
    if (distanceCm == 0) {
      // fill up graph on bottom of the LCD
      lcd.setCursor(0, 1);
      for (int i = 0; i < 16; i++) {
        lcd.print(BLOCK);
        delay(30);
      }
      distanceNew = 15;
    }
    // clear the bottom of the LCD
    else if (distanceCm <= 3) {
      clearLine(1);
      distanceNew = 0;
    }
    // depending on the value between 4 and 13 it fills in two blocks of the LCD using lines thus splitting two blocks into 10 segments
    else if (distanceCm >= 4 && distanceCm <= 13) {
      int charVal = distanceCm - 3;
      clearLine(1);
      //
      if (charVal > 4) {
        lcd.print(BLOCK);
        charVal = charVal - 5;
      }
      lcd.write(byte(charVal));
    }
  }
  // graphs values greater than 13cm
  else {
    // when distance increases, so then the graph increases
    if (distanceNew > distanceLast) {
      lcd.setCursor(0, 1);
      lcd.print(BLOCK);
      for (int i = 0; i <= distanceNew; i++) {
        lcd.print(BLOCK);
        delay(30);
      }
    }
    // when distance decreases, so then the graph decreases
    else if (distanceNew < distanceLast) {
      for (int i = distanceLast + 1; i > distanceNew; i--) {
        lcd.setCursor(i, 1);
        lcd.print(" ");
        delay(30);
      }
    }
  }
  distanceLast = distanceNew;
}
// Updates the LCD calling both the written and graphing functions lcdDisplay() and distanceLive() respectively
void lcdRefresh() {
  // controls blocks 2 to 7 on the lcd using a range from 14 to 100cm
  if (distanceCm <= 100 && distanceCm >= 14) {
    distanceLive(distanceCm * 5 / 86 + (51 / 43)); // division will pass an int within range 0 to 7 based on tested distance thresholds
  }
  // controls blocks 8 to 15 on the lcd using a range from 101 to 400cm
  else if (distanceCm > 100) {
    distanceLive(distanceCm * 7 / 299 + (1685 / 299)); // division will pass an int within range 8 to 16 based on tested distance thresholds
  }
  // deals with distance values less that 14cm, including zero
  else {
    distanceLive(0);
  }
  lcdDisplay();
  delay(100);
}
/*
*------------------------------------------------------------------------------------------
* Motor Control Functions
*
* Controls the motors given a speed until a certain distance this version of motorControl()
* has both motors running at different speeds to help make the robot go straight
*------------------------------------------------------------------------------------------
*/
// stops both motors
void brake() {
  digitalWrite(DIRECTION_CONTROL_M1, HIGH); //brake right wheel
  digitalWrite(DIRECTION_CONTROL_M2, HIGH); //brake left wheel
  analogWrite (RIGHT_MOTOR_SPEED, 0); //PWM Speed Control
  analogWrite (LEFT_MOTOR_SPEED, 0); //PWM Speed Control
}
// moves both motors forward
void forward(int left, int right) {
  digitalWrite(DIRECTION_CONTROL_M1, LOW); //right wheel moves forwards
  digitalWrite(DIRECTION_CONTROL_M2, LOW); //left wheel moves forwards, should be HIGH but reads value wrong
  analogWrite (RIGHT_MOTOR_SPEED, right * 51 / 80); //PWM Speed Control
  analogWrite (LEFT_MOTOR_SPEED, left * 51 / 80); //PWM Speed Control
  //stringCreate(a, b, encoderRight, encoderLeft, music);
}
void backwards(int left, int right) {
  digitalWrite(DIRECTION_CONTROL_M1, HIGH); //right wheel moves forwards
  digitalWrite(DIRECTION_CONTROL_M2, HIGH); //left wheel moves forwards, should be HIGH but reads value wrong
  analogWrite (RIGHT_MOTOR_SPEED, right * 51 / 80); //PWM Speed Control
  analogWrite (LEFT_MOTOR_SPEED, left * 51 / 80); //PWM Speed Control
  //stringCreate(a, b, encoderRight, encoderLeft, music);
}
//turns right
//had to switch right and left turning due to board production issues
void turnRight(int left, int right) {
  digitalWrite(DIRECTION_CONTROL_M1, HIGH); //right wheel moves backwards
  digitalWrite(DIRECTION_CONTROL_M2, LOW); //left wheel moves forwards, should be HIGH but reads value wrong
  analogWrite (RIGHT_MOTOR_SPEED, right * 51 / 80); //PWM Speed Control
  analogWrite (LEFT_MOTOR_SPEED, left * 51 / 80); //PWM Speed Control
  //stringCreate(a, b, encoderRight, encoderLeft, music);
}
//turns left
//had to switch right and left turning due to board production issues
void turnLeft(int left, int right) {
  digitalWrite(DIRECTION_CONTROL_M1, LOW); //right wheel moves forwards
  digitalWrite(DIRECTION_CONTROL_M2, HIGH); //left wheel moves backwards, should be LOW but reads value wrong
  analogWrite (RIGHT_MOTOR_SPEED, right * 51 / 80); //PWM Speed Control
  analogWrite (LEFT_MOTOR_SPEED, left * 51 / 80); //PWM Speed Control
  //stringCreate(a, b, encoderRight, encoderLeft, music);
}
// turns the robot 90 degrees to the left
void turn90Left() {
  turnRight(150, 150);
  delay(1000);
  brake();
}

char calculatedApproach() {
  reportDistance();
  char calculatedSpeed = MAX_SPEED;
  /* a distance measurement of 0 is returned when the distance
  to an object exceeds the limit of the range finder (~3.8m) */
  if (distanceCm != 0 || distanceCm <= approachWallThreshold[0]) {
    for (int i = 0; distanceCm <= approachWallThreshold[i]; i++)
      calculatedSpeed = approachWallSpeeds[i];
  }
  return calculatedSpeed;
}
void manualDrivingMode(char heading) {
  if (heading == LEFT)
  
    turnLeft(TURN_SPEED, TURN_SPEED);
  else if (heading == RIGHT)
    turnRight(TURN_SPEED, TURN_SPEED);
    
  else if (heading == CENTRE)
    forward(FIRST_GEAR, FIRST_GEAR);
  else if (heading == FORWARD)
    forward(FIRST_GEAR, FIRST_GEAR);
  else if (heading == FORWARD_SPEED_1)
    forward(SECOND_GEAR, SECOND_GEAR);
  else if (heading == FORWARD_SPEED_2)
    forward(THIRD_GEAR, THIRD_GEAR);
    
  else if (heading == BACKUP)
    backwards(FIRST_GEAR, FIRST_GEAR);  
    
  else if (heading == STOP)
    brake();
}
/*
*------------------------------------------------------------------------------------------
*Main Loop Function
*------------------------------------------------------------------------------------------
*/
void loop() {
  if (autonomous) {
    // forward();
    // setSpeeds(350, 350);
    // delay(2000);
    // setSpeeds(300, 300);
    // delay(2000);
    // setSpeeds(250, 250);
    // delay(2000);
    // setSpeeds(150, 150);
    // delay(2000);
    // setSpeeds(75, 75);
    // delay(2000);
    char newSpeed = calculatedApproach();
    lcdRefresh();
    if (newSpeed == 0) { // at wall, need to turn left
      brake();
      turn90Left();
      newSpeed = calculatedApproach(); // proceed after turn
    }
    forward(newSpeed, newSpeed);
  } else { //full manual mode using controller
    while (Serial.available() > 0) {
      char heading = Serial.read();
      manualDrivingMode(heading);
    }
  }
}
