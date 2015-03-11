/*
 *------------------------------------------------------------------------------------------
 * Measurements Functions
 *------------------------------------------------------------------------------------------
 */
#include "WhizzardUltraSensor.h"
#include "NewPing.h"

NewPing sonar(RANGEFINDER_TRIGGER_PIN, RANGEFINDER_ECHO_PIN, MAX_DISTANCE); // initialize ultrasonic sensor library

WhizzardUltraSensor::WhizzardUltraSensor() { /*constructor*/ }

// sensor initialization
void WhizzardUltraSensor::init() {
  pinMode(RANGEFINDER_TRIGGER_PIN, OUTPUT);
  pinMode(RANGEFINDER_ECHO_PIN, INPUT);
  pinMode(TEMPERATURE_PIN, INPUT);
  
  recordTemp();
}

// Calculate the speed to travel based on distance from object.
char WhizzardUltraSensor::calculatedApproach() {
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

// records the temperature and calculates the speed of sound
void WhizzardUltraSensor::recordTemp() {
  int data = analogRead(TEMPERATURE_PIN);
  temperature = (500 * data) >> 10;
  velocity = (331.3 + (0.6 * temperature)); // speed of sound
}

// records the distance to an obstruction
void WhizzardUltraSensor::reportDistance() {
  echoPulse = float(sonar.ping_median()); // returns time to and from object
  time = (echoPulse) / 2; // divide by two because functions returns twice the time needed
  distanceCm = (velocity * time) / 10000;
}

// return current distance in meters
float WhizzardUltraSensor::getDistanceCm() {
 return distanceCm; 
}
