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
const int DISTANCE_THRESHOLD = 70; // cm
const int SPEED_MAX = 255; // PWM speed where 0 is brake, 255 is maximum
const int MAGIC = 18; // magic number to control difference in left and right motor speed

float temperature; // temperature value in C
float humidity; // humidity in percentage decimal

float echoPulseFront; // time returned from ultrasonic sensor
float velocity; // intermediate ultrasonic sensor calculation
float time; // intermediate ultrasonic sensor calculation
float distanceCm; // distance read by ultrasonic sensor
int value; // used to help vary speeds on motors
int count = 1; // used in loop() so that delay at beginning only occurs once

DHT dht(pinTemp, DHTTYPE);

NewPing sonar(pinTrigFront, pinEchoFront, MAX_DISTANCE); // initialize ultrasonic sensor

void setup()
{
  Serial.begin(9600);
  motors.init();
  pinMode(pinTrigFront, OUTPUT);
  pinMode(pinEchoFront, INPUT);
  dht.begin();
}

void intro() {
  Serial.println("START OF PROGRAM");
  Serial.println("__________________________________________");
  Serial.println("");

  temperature = dht.readTemperature();
  Serial.print("Temperature: ");
  Serial.println(temperature);

  humidity = dht.readHumidity() / 100;
  Serial.print("Humidity: ");
  Serial.println(humidity);

  reportDistance();
  Serial.println("");

  Serial.println("__________________________________________");
}

void reportDistance() {
  echoPulseFront = float(sonar.ping_median()); // returns time to and from object
  // distance = velocity * time where velocity is speed of sound
  velocity = (331.3 + (0.6 * temperature)); // speed of sound

  const float gasConst = 0.2869; // gas constant in air in kj/kg*K
  float cp = 1.005 + 1.82 * humidity; // heat capacity for constant pressure in kj/kj*K where 1.005 is the heat capacity in dry air, and 1.82 is the heat capacity of water vapor
  float cv = cp - gasConst; // heat capacity for constant volume
  float speedOfSound = powf((cp / cv) * gasConst * (temperature + 273.15) * 0.00001, 0.5); // cm/s

  time = (echoPulseFront) / 2; // divide by two because functions returns twice the time needed
  distanceCm = (velocity * time) / 10000;
  Serial.print(distanceCm);
  Serial.print(" ");
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
    motors.setSpeeds(velocity, velocity-value); // different velocity values to help it go straight
  }
  while (distanceCm == 0 || distanceCm >= threshold) {
    Serial.print(velocity);
    Serial.print(" ");
    reportDistance();
    reportCurrent();
    Serial.println(" ");
  }
  value *= 0.75; // scale value so that it affects the seperate speeds differently while slowing down
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
    turnLeft();
    reportDistance();
    float pathLeft = distanceCm;
    delay(500);
    
    // then turns right and measures distance
    turnRight();
    reportDistance();
    float pathRight = distanceCm;
    delay(500);
    
    // decides on which path to take using the logic in the function description
    if(pathRight >= 1000 || pathRight == 0){
      break;
    }
    else if(pathRight >= pathLeft && pathLeft != 0){
      break;
    }
    else{
      turnBackLeft();
      break;
    }
  } 
}

// turns the robot 90 degrees to the left
void turnLeft(){
  motors.setSpeeds(100, -100);
  delay(900);
  motors.setBrakes(400, 400);
}

// turns the robot 180 degrees to the right
void turnRight(){
  motors.setSpeeds(-100, 100);
  delay(1730);
  motors.setBrakes(400, 400);
}

// turns the robot 180 degrees to the left
void turnBackLeft(){
  motors.setSpeeds(100, -100);
  delay(1730);
  motors.setBrakes(400, 400);
}

void loop() {
  reportDistance();
  
  // 1 second delay at the beginning
  if(count == 1){
    delay(1000);
    //intro();
    count--;
  }
  
  // used to reset value to the magic number "MAGIC"
  value = MAGIC;
  
  // goes through 5 different speeds
  motorControl(400, DISTANCE_THRESHOLD);
  motorControl(300, 50);
  motorControl(150, 35);
  motorControl(100, 7);
  motorControl(50, 3);

  // applies the brakes when close enough to the wall
  motors.setBrakes(400, 400);
  
  turnControl();
  delay(500);
}
