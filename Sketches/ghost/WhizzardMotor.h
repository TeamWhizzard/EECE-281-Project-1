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
   void forward(int left, int right);
   void backward(int left, int right);
   void turnRight(int left, int right);
   void turnLeft(int left, int right);
   void turn90Left();
     
  private:
   // private functions/ variables
};

#endif
