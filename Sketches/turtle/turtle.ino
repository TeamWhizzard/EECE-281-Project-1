#include "DualVNH5019MotorShield.h"
#include "NewPing.h"
#include "DHT.h"
#include <math.h>
#define DHTTYPE DHT11

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

DHT dht(pinTemp, DHTTYPE);

NewPing sonar(pinTrigFront, pinEchoFront, MAX_DISTANCE); // initialize ultrasonic sensor

void setup()
{
  Serial.begin(9600);
  motors.init();
  pinMode(pinTrigFront, OUTPUT);
  pinMode(pinEchoFront, INPUT);
  dht.begin();
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
    reportDistance();
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
  delay(1000);
  motors.setBrakes(400, 400);
}

// turns the robot 90 degrees to the left
void turnLeft(){
  motors.setSpeeds(100, -100);
  delay(1000);
  motors.setBrakes(400, 400);
}

// turns the robot 180 degrees to the left
void turnAroundLeft(){
  motors.setSpeeds(100, -100);
  delay(2000);
  motors.setBrakes(400, 400);
}

// turns the robot 180 degrees to the right
void turnAroundRight(){
  motors.setSpeeds(-100, 100);
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
