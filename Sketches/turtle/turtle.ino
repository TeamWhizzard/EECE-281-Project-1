#include "NewPing.h"
#include "LCD.h"
#include "LiquidCrystal_I2C.h"
#include <math.h>
#include <Wire.h>

#define I2C_ADDR  0x27 // <<----- Add your address here.  Find it from I2C Scanner
#define BACKLIGHT_PIN  3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7

int MR = 4;    //Right motor Direction Control
int SR = 5;    //Right motor Speed Control
int SL = 6;    //Left motor Speed Control
int ML = 7;    //Left motor Direction Control

// ultrasonic and temperature sensors
const int pinTrigFront = 3;
const int pinEchoFront = A3;
const int pinTemp = A2;

const int MAX_DISTANCE = 380; // maximum reading distance of ultrasonic sensor in cm
const int DISTANCE_THRESHOLD = 50; // cm
const int SPEED_MAX = 255; // PWM speed where 0 is brake, 255 is maximum
const int MAGIC = 10; // magic number to control difference in left and right motor speed
const int NO_MUSIC = 0; // value that will be assigned to "music" in order to not play music
const int INTRO_MUSIC = 1; // value that will be assigned to "music" to play intro song
const int GAME_OVER_MUSIC = 2; // value that will be assigned to "music" to play game over song

float temperature; // temperature value in C
float echoPulseFront; // time returned from ultrasonic sensor
float velocity; // intermediate ultrasonic sensor calculation
float time; // intermediate ultrasonic sensor calculation
float distanceCm; // distance read by ultrasonic sensor
int value; // used to help vary speeds on motors
int distanceLast; // keeps track of last moisture reading
char block = 0xFF; // block character used on lcd
int music = 0; // assigned the values 0, 1, or 2 to play a song depending on the value
int encoderRight = 0; // value given from the right motor encoder
int encoderLeft = 0; // value given from the left motor encoder

// array of custom characters that each represent a different amount of lines filled
// in a single block on an lcd. Broken into 5 segments
byte slices[6][8] = {{B00000,B00000,B00000,B00000,B00000,B00000,B00000,B00000},
                     {B10000,B10000,B10000,B10000,B10000,B10000,B10000,B10000},
                     {B11000,B11000,B11000,B11000,B11000,B11000,B11000,B11000},
                     {B11100,B11100,B11100,B11100,B11100,B11100,B11100,B11100},
                     {B11110,B11110,B11110,B11110,B11110,B11110,B11110,B11110},
                     {B11111,B11111,B11111,B11111,B11111,B11111,B11111,B11111}};

NewPing sonar(pinTrigFront, pinEchoFront, MAX_DISTANCE); // initialize ultrasonic sensor

// set the LCD address to 0x27 for a 16 chars 2 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);  // Set the LCD I2C address


/*
 *------------------------------------------------------------------------------------------
 *Setup Function
 *------------------------------------------------------------------------------------------
*/

void setup()
{
  Serial.begin(9600);
  for(int i = 4; i <= 7; i++){
    pinMode(i, OUTPUT);
  }
  pinMode(pinTrigFront, OUTPUT);
  pinMode(pinEchoFront, INPUT);
  pinMode(pinTemp, INPUT);
  lcd.begin(16,2);         // initialize the lcd for 16 chars 2 lines and turn on backlight
  lcd.home();
  lcd.clear();
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);
  for(int i = 0; i < 6; i++){
    lcd.createChar(i, slices[i]);
  }
  //recordTemp();
  delay(1000);
}


/*
 *------------------------------------------------------------------------------------------
 *Measurements Functions
 *------------------------------------------------------------------------------------------
*/

// records the temperature and calculates the speed of sound
void recordTemp() {
  int data = analogRead(pinTemp);
  int temperature = (500*data) >> 10;
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
 *Bluetooth Communication Functions
 *------------------------------------------------------------------------------------------
*/

// creates a string out of individual motor speeds, encoder values and music selection value
// to send to the controller via bluetooth
void stringCreate(int speedRight, int speedLeft, int encoderRight, int encoderLeft, int music){
  String num1 = String(speedRight);
  String num2 = String(speedLeft);
  String num3 = String(encoderRight);
  String num4 = String(encoderLeft);
  String num5 = String(music);
  
  String message = num1 + "," + num2 + "," + num3 + "," + num4 + "," + num5;
  bluetoothData(message);
}

// sends a string via bluetooth from the robot to the controller
void bluetoothData(String message){
  Serial.print(message); // prints message containing motor speeds and encoder values to the serial monitor
}


/*
 *------------------------------------------------------------------------------------------
 *LCD Control Functions
 *------------------------------------------------------------------------------------------
*/

// clears a given line on the lcd
void clearLine(int line){
  lcd.setCursor(0,line);
  lcd.print("                ");
  delay(20);
  lcd.setCursor(0,line);
}

// Displays the distance measurement on the top line of the LCD
void lcdDisplay(){
  lcd.setCursor(0,0);
  lcd.print("Distance: ");
  distanceCm /= 100; // convert from centimeters to meters
  // Displays "Oodles" when the distance recorded is zero since the range sensor is limited to 4 meters
  if(distanceCm == 0){
    lcd.print("      ");
    lcd.setCursor(10,0);
    lcd.print("Oodles");
  }
  // Displays the number value of the distance recorded
  else{
    lcd.setCursor(14,0);
    lcd.print(" ");
    lcd.setCursor(10,0);
    lcd.print(distanceCm);
    lcd.setCursor(15,0);
    lcd.print("m");
  }
}

// distanceLive() - displays bar representation of most recent distance reading on bottom line of LCD display
void distanceLive(int distanceNew){
  // graphs values 13cm and less
  if(distanceCm <= 13){
    // we assume that when distanceCm equals zero, that the range is beyond 4meters (outside the range of the sensor)
    if(distanceCm == 0){
      // fill up graph on bottom of the LCD
      lcd.setCursor(0,1);
      for(int i = 0; i < 16; i++){
        lcd.print((String) block);
        delay(30);
      }
      distanceNew = 15;
    }
    // clear the bottom of the LCD
    else if(distanceCm <= 3){
      clearLine(1);
      distanceNew = 0;
    }
    // depending on the value between 4 and 13 it fills in two blocks of the LCD using lines thus splitting two blocks into 10 segments
    else if(distanceCm >= 4 && distanceCm <= 13){
      int charVal = distanceCm - 3;
      clearLine(1);
      // 
      if(charVal > 4){
        lcd.print((String) block);
        charVal = charVal - 5;
      }
      lcd.write(byte(charVal));
    }
  }
  // graphs values greater than 13cm
  else{
    // when distance increases, so then the graph increases
    if(distanceNew > distanceLast) {
      lcd.setCursor(0,1);
      lcd.print((String) block);
      for (int i = 0; i <= distanceNew; i++) {
        lcd.print((String) block);
        delay(30);
      }
    }
    // when distance decreases, so then the graph decreases
    else if(distanceNew < distanceLast) {
      for (int i = distanceLast+1; i > distanceNew; i--) {
        lcd.setCursor(i,1);
        lcd.print(" ");
        delay(30);
      }
    }
  }
  distanceLast = distanceNew;
}

// Updates the LCD calling both the written and graphing functions lcdDisplay() and distanceLive() respectively
void lcdRefresh(){
  // controls blocks 2 to 7 on the lcd using a range from 14 to 100cm
  if(distanceCm <= 100 && distanceCm >= 14){
    distanceLive(distanceCm * 5 / 86 + (51/43)); // division will pass an int within range 0 to 7 based on tested distance thresholds
  }
  // controls blocks 8 to 15 on the lcd using a range from 101 to 400cm
  else if(distanceCm > 100){
    distanceLive(distanceCm * 7 / 299 + (1685/299)); // division will pass an int within range 8 to 16 based on tested distance thresholds
  }
  // deals with distance values less that 14cm, including zero
  else{
    distanceLive(0); // division will pass 0
  }
  lcdDisplay();
  delay(100);
}


/*
 *------------------------------------------------------------------------------------------
 *Motor Control Functions
 *------------------------------------------------------------------------------------------
*/

/*
 *Controls the motors given a speed until a certain distance this version of motorControl()
 *has both motors running at different speeds to help make the robot go straight
*/
void motorControl(int velocity, int threshold) {
  if (distanceCm == 0 || distanceCm >= threshold) {
    forward(velocity, velocity);
    //forward(velocity+value, velocity); // different velocity values to help it go straight
  }
  while (distanceCm == 0 || distanceCm >= threshold) {
    reportDistance();
    lcdRefresh();
  }
  //value *= 1.05; // scale value so that it affects the seperate speeds differently while slowing down
}

/*
 *Controls the turning. Turns left and measures the distance to the left wall, then turns right and measures
 *the distance to the right wall. If the distance to the right wall is over one meter or greater than the path
 *to the left, the robot moves to the right wall. If not, then it turns back to the left and proceeds down the
 *left path
 */
void turnControl(){
 while(distanceCm == 0 || distanceCm <= 5){
    // turns right first and measures distance
    reportDistance();
    turn90Right();
    reportDistance();
    float pathRight = distanceCm;
    delay(500);
    
    // then turns left and measures distance
    turn180Left();
    reportDistance();
    float pathLeft = distanceCm;
    delay(500);
    
    // decides on which path to take using the logic in the function description
    if(pathLeft >= 50 || pathLeft == 0){
      break;
    }
    else if(pathLeft >= pathRight && pathRight != 0){
      break;
    }
    else{
      turn180Right();
      break;
    }
  } 
}

// stops both motors
void brake(){
  digitalWrite(SR,LOW);  //brake right wheel
  digitalWrite(SL,LOW);  //brake left wheel
}
// moves both motors forward
void forward(char a,char b){
  analogWrite (SR,a);  //PWM Speed Control
  digitalWrite(MR,HIGH);  //right wheel moves forwards
  analogWrite (SL,b);  //PWM Speed Control
  digitalWrite(ML,LOW);  //left wheel moves forwards, should be HIGH but reads value wrong
  stringCreate(a, b, encoderRight, encoderLeft, music);
}
// move both motors backward
void backward(char a,char b){
  analogWrite (SR,a);  //PWM Speed Control
  digitalWrite(MR,LOW);  //right wheel moves backwards
  analogWrite (SL,b);  //PWM Speed Control
  digitalWrite(ML,HIGH);  //left wheel moves backwards, should be LOW but reads value wrong
  stringCreate(a, b, encoderRight, encoderLeft, music);
}
//turns right
//had to switch right and left turning due to board production issues
void turnRight(char a,char b){
  analogWrite (SR,a);  //PWM Speed Control
  digitalWrite(MR,LOW);  //right wheel moves backwards
  analogWrite (SL,b);  //PWM Speed Control
  digitalWrite(ML,LOW);  //left wheel moves forwards, should be HIGH but reads value wrong
  stringCreate(a, b, encoderRight, encoderLeft, music);
}
//turns left
//had to switch right and left turning due to board production issues
void turnLeft(char a,char b){
  analogWrite (SR,a);  //PWM Speed Control
  digitalWrite(MR,HIGH);  //right wheel moves forwards
  analogWrite (SL,b);  //PWM Speed Control
  digitalWrite(ML,HIGH);  //left wheel moves backwards, should be LOW but reads value wrong
  stringCreate(a, b, encoderRight, encoderLeft, music);
}

// turns the robot 90 degrees to the right
void turn90Right(){
  turnLeft(150, 150);
  delay(300);
  brake();
}

// turns the robot 90 degrees to the left
void turn90Left(){
  turnRight(150, 150);
  delay(300);
  brake();
}

// turns the robot 180 degrees to the left
void turn180Left(){
  turnRight(150, 150);
  delay(750);
  brake();
}

// turns the robot 180 degrees to the right
void turn180Right(){
  turnLeft(150, 150);
  delay(750);
  brake();
}


/*
 *------------------------------------------------------------------------------------------
 *Main Loop Function
 *------------------------------------------------------------------------------------------
*/
void loop() {
  reportDistance();
  
  // used to reset value to the magic number "MAGIC"
  value = MAGIC;
  
  // goes through 5 different speeds
  //motorControl(255, DISTANCE_THRESHOLD);
  //motorControl(175, 30);
  //motorControl(125, 20);
  //motorControl(75, 7);
  //motorControl(25, 3);
  
  forward(200, 200);
  delay(2000);
  brake();
  delay(500);
  backward(150, 150);
  delay(2000);
  brake();
  delay(500);
  turn90Right();
  delay(500);
  turn180Left();
  delay(500);

  // applies the brakes when close enough to the wall
  //brake();
  
  //turnControl();
  delay(500);
}
