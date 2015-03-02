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

float temperature; // temperature value in C
float humidity; // humidity in percentage decimal

float echoPulseFront; // time returned from ultrasonic sensor
float velocity; // intermediate ultrasonic sensor calculation
float time; // intermediate ultrasonic sensor calculation
float distanceCm; // distance read by ultrasonic sensor

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

void intro(){
  Serial.println("START OF PROGRAM");
  Serial.println("__________________________________________");
  Serial.println("");
  
  temperature = dht.readTemperature();
  Serial.print("Temperature: ");
  Serial.println(temperature);
  
  humidity = dht.readHumidity() / 100;
  Serial.print("Humidity: ");
  Serial.println(humidity);
  
  distance();
  Serial.println("");
  
  Serial.println("__________________________________________");
}

void distance(){
  echoPulseFront = float(sonar.ping_median()); // returns time to and from object
  // distance = velocity * time where velocity is speed of sound
  velocity = (331.3 + (0.6 * temperature)); // speed of sound
 
  const float gasConst = 0.2869; // gas constant in air in kj/kg*K
  float cp = 1.005 + 1.82*humidity; // heat capacity for constant pressure in kj/kj*K where 1.005 is the heat capacity in dry air, and 1.82 is the heat capacity of water vapor
  float cv = cp - gasConst; // heat capacity for constant volume
  float speedOfSound = powf((cp/cv)*gasConst*(temperature+273.15)*0.00001, 0.5); // cm/s
 
  //Serial.print("Velocity right: ");
  //Serial.print(velocity);
  //Serial.print("; Velocity new: ");
  //Serial.print(speedOfSound);
  
  time = (echoPulseFront) / 2; // devide by two because functions returns twice the time needed
  distanceCm = (velocity * time) / 10000;
  Serial.print("; Distance: ");
  Serial.print(distanceCm);
}

void reportCurrent(){
  Serial.print("; Right current: ");
  Serial.print(motors.getM1CurrentMilliamps());
  Serial.print("; Left current: ");
  Serial.print(motors.getM2CurrentMilliamps());
}

void motorControl(int speed1, int threshold){
  Serial.println("Speed: ");
  Serial.println(speed1);
  Serial.println("");
  if(distanceCm == 0 || distanceCm >= threshold)
    motors.setSpeeds(speed1, speed1);
  
  while(distanceCm == 0 || distanceCm >= threshold){
    distance();
    reportCurrent();
    Serial.println("");
  }
}

void loop(){
  delay(5000);
  intro();
  
  motorControl(400, DISTANCE_THRESHOLD);
  motorControl(300, 40);
  motorControl(150, 20);
  motorControl(100, 10);
  motorControl(50, 5);
  
  motors.setBrakes(400, 400);
  
  delay(10000);
}
