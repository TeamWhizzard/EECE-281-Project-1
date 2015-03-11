/*
 *------------------------------------------------------------------------------------------
 * Motor Control Functions
 *
 * Controls the motors given a speed until a certain distance this version of motorControl()
 * has both motors running at different speeds to help make the robot go straight
 *------------------------------------------------------------------------------------------
*/

#ifndef WhizzardMotor_H
#define WhizzardMotor_H

#include "Arduino.h"

class WhizzardMotor {
 
  public:
   // public functions
   WhizzardMotor();

   void init();
   void brake();
   void forward(int velocity);
   void backward(int velocity);
   void turnRight(int velocity);
   void turnLeft(int velocity);
   void turn90Left();
     
  private:
   // private functions/ variables
};

#endif
