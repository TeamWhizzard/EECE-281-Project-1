#include "DualVNH5019MotorShield.h"
#include "NewPing.h"
#include "DHT.h"
#include <math.h>
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

#define DHTTYPE DHT11
#define I2C_ADDR  0x27 // <<----- Add your address here.  Find it from I2C Scanner
#define BACKLIGHT_PIN  3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7

DualVNH5019MotorShield motors;

// ultrasonic and temperature sensors
const int pinTrigFront = 3;
const int pinEchoFront = A3;
const int pinTemp = A2;

const int MAX_DISTANCE = 300; // maximum reading distance of ultrasonic sensor in cm
const int DISTANCE_THRESHOLD = 50; // cm
const int SPEED_MAX = 255; // PWM speed where 0 is brake, 255 is maximum
const int MAGIC = 10; // magic number to control difference in left and right motor speed

float temperature; // temperature value in C

float echoPulseFront; // time returned from ultrasonic sensor
float velocity; // intermediate ultrasonic sensor calculation
float time; // intermediate ultrasonic sensor calculation
float distanceCm; // distance read by ultrasonic sensor
int value; // used to help vary speeds on motors
int distanceLast; // keeps track of last moisture reading
char block = 0xFF; // block character used on lcd

// custom character that produces a blank block on the lcd
byte lineZero[8] = {
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000
};
// custom character that produces a single line on the lcd
byte lineOne[8] = {
    B10000,
    B10000,
    B10000,
    B10000,
    B10000,
    B10000,
    B10000,
    B10000
};
// custom character that produces a double line on the lcd
byte lineTwo[8] = {
    B11000,
    B11000,
    B11000,
    B11000,
    B11000,
    B11000,
    B11000,
    B11000
};
// custom character that produces a triple line on the lcd
byte lineThree[8] = {
    B11100,
    B11100,
    B11100,
    B11100,
    B11100,
    B11100,
    B11100,
    B11100
};
// custom character that produces a quadruple line on the lcd
byte lineFour[8] = {
    B11110,
    B11110,
    B11110,
    B11110,
    B11110,
    B11110,
    B11110,
    B11110
};
// custom character that produces a full block on the lcd
byte lineFive[8] = {
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111
};

DHT dht(pinTemp, DHTTYPE);

NewPing sonar(pinTrigFront, pinEchoFront, MAX_DISTANCE); // initialize ultrasonic sensor

// set the LCD address to 0x27 for a 16 chars 2 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol

LiquidCrystal_I2C lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);  // Set the LCD I2C address


void setup()
{
  Serial.begin(9600);
  motors.init();
  pinMode(pinTrigFront, OUTPUT);
  pinMode(pinEchoFront, INPUT);
  dht.begin();
  lcd.begin(16,2);         // initialize the lcd for 16 chars 2 lines and turn on backlight
  lcd.clear();
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.createChar(0, lineZero);
  lcd.createChar(1, lineOne);
  lcd.createChar(2, lineTwo);
  lcd.createChar(3, lineThree);
  lcd.createChar(4, lineFour);
  lcd.createChar(5, lineFive);
  intro();
  delay(1000);
}

void intro() {
  Serial.println("START OF PROGRAM");
  Serial.println("__________________________________________");
  Serial.println("");

  temperature = dht.readTemperature();
  Serial.print("Temperature: ");
  Serial.println(temperature);
  // distance = velocity * time where velocity is speed of sound
  velocity = (331.3 + (0.6 * temperature)); // speed of sound

  reportDistance();
  Serial.println("");

  Serial.println("__________________________________________");
}

void bluetoothData(int pace, String orientation){
  Serial.print(pace); // prints the speed to the serial monitor
  Serial.print("; ");
  Serial.println(orientation); // prints the orientation to the serial monitor
}

// clears a given line on the lcd
void clearLine(int line){
  lcd.setCursor(0,line);
  lcd.print("                ");
  delay(20);
  lcd.setCursor(0,line);
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

// Displays the distance measurement on the top line of the LCD
void lcdDisplay(){
  lcd.backlight();
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
  delay(500);
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

void reportDistance() {
  echoPulseFront = float(sonar.ping_median()); // returns time to and from object
  time = (echoPulseFront) / 2; // divide by two because functions returns twice the time needed
  distanceCm = (velocity * time) / 10000;
  //Serial.print(distanceCm);
  //Serial.print(" ");
}

void reportCurrent() {
  Serial.print(motors.getM1CurrentMilliamps()); // M1 = Right Motor
  Serial.print(" ");
  Serial.print(motors.getM2CurrentMilliamps()); // M2 = Left Motor
  Serial.print(" ");
}

// Controls the motors given a speed until a certain distance
// this version of motorControl() has both motors running at even speeds
/*
void motorControl(int velocity, int threshold) {
  if (distanceCm == 0 || distanceCm >= threshold) {
    motors.setSpeeds(velocity, velocity);
  }
  while (distanceCm == 0 || distanceCm >= threshold) {
    Serial.print(velocity);
    Serial.print(" ");
    reportDistance();
    reportCurrent();
    Serial.println(" ");
  }
}
*/

/* 
Controls the motors given a speed until a certain distance this version of motorControl()
has both motors running at different speeds to help make the robot go straight

----------------------------------VERY FINICKY----------------------------------
----------------------------------NOT CONSTANT----------------------------------


*/
void motorControl(int velocity, int threshold) {
  if (distanceCm == 0 || distanceCm >= threshold) {
    motors.setSpeeds(velocity+value, velocity); // different velocity values to help it go straight
  }
  while (distanceCm == 0 || distanceCm >= threshold) {
    //Serial.print(velocity);
    //Serial.print(" ");
    bluetoothData(velocity, "Centre");
    reportDistance();
    lcdRefresh();
    reportCurrent();
    //Serial.println(" ");
  }
  value *= 1.05; // scale value so that it affects the seperate speeds differently while slowing down
}

/*
Controls the turning. Turns left and measures the distance to the left wall, then turns right and measures
the distance to the right wall. If the distance to the right wall is over one meter or greater than the path
to the left, the robot moves to the right wall. If not, then it turns back to the left and proceeds down the
left path
*/
void turnControl(){
 while(distanceCm == 0 || distanceCm <= 5){
    // turns left first and measures distance
    reportDistance();
    turnRight();
    reportDistance();
    float pathRight = distanceCm;
    delay(500);
    
    // then turns right and measures distance
    turnAroundLeft();
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
      turnAroundRight();
      break;
    }
  } 
}

// turns the robot 90 degrees to the right
void turnRight(){
  motors.setSpeeds(-100, 100);
  bluetoothData(0, "Turning Right");
  delay(1000);
  motors.setBrakes(400, 400);
}

// turns the robot 90 degrees to the left
void turnLeft(){
  motors.setSpeeds(100, -100);
  bluetoothData(0, "Turning Left");
  delay(1000);
  motors.setBrakes(400, 400);
}

// turns the robot 180 degrees to the left
void turnAroundLeft(){
  motors.setSpeeds(100, -100);
  bluetoothData(0, "Turning Left");
  delay(2000);
  motors.setBrakes(400, 400);
}

// turns the robot 180 degrees to the right
void turnAroundRight(){
  motors.setSpeeds(-100, 100);
  bluetoothData(0, "Turning Right");
  delay(2000);
  motors.setBrakes(400, 400);
}

void loop() {
  reportDistance();
  
  // used to reset value to the magic number "MAGIC"
  value = MAGIC;
  
  // goes through 5 different speeds
  motorControl(350, DISTANCE_THRESHOLD);
  motorControl(275, 30);
  motorControl(175, 20);
  motorControl(100, 7);
  motorControl(50, 3);

  // applies the brakes when close enough to the wall
  motors.setBrakes(400, 400);
  
  turnControl();
  delay(500);
}
