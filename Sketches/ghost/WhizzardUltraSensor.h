#ifndef WhizzardUltraSensor_H
#define WhizzardUltraSensor_H

#include "Arduino.h"

// sensor pins
#define RANGEFINDER_TRIGGER_PIN  8
#define RANGEFINDER_ECHO_PIN A3
#define TEMPERATURE_PIN A2

#define MAX_SPEED          127 // maximum PWM motor speed
#define MAX_DISTANCE       380 // maximum reading distance of ultrasonic sensor in cm

class WhizzardUltraSensor { 
 public:
 // public functions
  WhizzardUltraSensor();
  
  void init();
  char calculatedApproach();
  float getDistanceCm();

 private:
   // private functions/ constants
   // collision detection threshold approach speeds and distances
   char approachWallSpeeds[6]   = {127, 100, 75, 50, 25, 0};
   int approachWallThreshold[6] = {50,  35,  25,  15,  10, 3};
   
   float echoPulse;      // time returned from ultrasonic sensor
   float distanceCm;     // distance read by ultrasonic sensor in cm
   int temperature;      // temperature value in degrees C
   float velocity;       // intermediate ultrasonic sensor calculation value
   float time;           // intermediate ultrasonic sensor calculation value
   
   void recordTemp();
   void reportDistance();
};

#endif
