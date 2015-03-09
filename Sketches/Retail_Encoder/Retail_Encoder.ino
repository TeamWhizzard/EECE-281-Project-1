#include "Arduino.h"


// Quadrature encoders
// Left encoder
#define c_LeftEncoderInterruptA 0
#define c_LeftEncoderInterruptB 1
#define c_LeftEncoderPinA 2
#define c_LeftEncoderPinB 3
#define LeftEncoderIsReversed

#define c_RightEncoderInterruptA 0
#define c_RightEncoderInterruptB 1
#define c_RightEncoderPinA 2
#define c_RightEncoderPinB 3
#define RightEncoderIsReversed

volatile bool _LeftEncoderASet;
volatile bool _LeftEncoderBSet;
volatile bool _LeftEncoderAPrev;
volatile bool _LeftEncoderBPrev;
volatile long _LeftEncoderTicks = 0;

volatile bool _RightEncoderASet;
volatile bool _RightEncoderBSet;
volatile bool _RightEncoderAPrev;
volatile bool _RightEncoderBPrev;
volatile long _RightEncoderTicks = 0;
void setup()
{
  Serial.begin(9600);

  // Quadrature encoders
  // Left encoder
  pinMode(c_LeftEncoderPinA, INPUT);      // sets pin A as input
 // digitalWrite(c_LeftEncoderPinA, LOW);  // turn on pullup resistors
  pinMode(c_LeftEncoderPinB, INPUT);      // sets pin B as input
 // digitalWrite(c_LeftEncoderPinB, LOW);  // turn on pullup resistors
  attachInterrupt(c_LeftEncoderInterruptA, HandleLeftMotorInterruptA, CHANGE);
  attachInterrupt(c_LeftEncoderInterruptB, HandleLeftMotorInterruptB, CHANGE);
  
  pinMode(c_RightEncoderPinA, INPUT);      // sets pin A as input
 // digitalWrite(c_LeftEncoderPinA, LOW);  // turn on pullup resistors
  pinMode(c_RightEncoderPinB, INPUT);      // sets pin B as input
 // digitalWrite(c_LeftEncoderPinB, LOW);  // turn on pullup resistors
  attachInterrupt(c_RightEncoderInterruptA, HandleRightMotorInterruptA, CHANGE);
  attachInterrupt(c_RightEncoderInterruptB, HandleRightMotorInterruptB, CHANGE);
}

void loop()
{
  Serial.print("EncoderLeft Ticks: ");
  Serial.print(_LeftEncoderTicks);
  Serial.print("  RevolutionsLeft: ");
  Serial.print(_LeftEncoderTicks/4000.0);//4000 Counts Per Revolution
  Serial.print("\n");

  Serial.print("EncoderRight Ticks: ");
  Serial.print(_RightEncoderTicks);
  Serial.print("  RevolutionsRight: ");
  Serial.print(_RightEncoderTicks/4000.0);//4000 Counts Per Revolution
  Serial.print("\n");
}


// Interrupt service routines for the left motor's quadrature encoder
void HandleLeftMotorInterruptA(){
  _LeftEncoderBSet = digitalRead(c_LeftEncoderPinB);
  _LeftEncoderASet = digitalRead(c_LeftEncoderPinA);
  
  _LeftEncoderTicks+=ParseEncoderLeft();
  
  _LeftEncoderAPrev = _LeftEncoderASet;
  _LeftEncoderBPrev = _LeftEncoderBSet;
}

// Interrupt service routines for the left motor's quadrature encoder
void HandleRightMotorInterruptA(){
  _RightEncoderBSet = digitalRead(c_RightEncoderPinB);
  _RightEncoderASet = digitalRead(c_RightEncoderPinA);
  
  _RightEncoderTicks+=ParseEncoderRight();
  
  _RightEncoderAPrev = _RightEncoderASet;
  _RightEncoderBPrev = _RightEncoderBSet;
}


// Interrupt service routines for the right motor's quadrature encoder
void HandleLeftMotorInterruptB(){
  // Test transition;
  _LeftEncoderBSet = digitalRead(c_LeftEncoderPinB);
  _LeftEncoderASet = digitalRead(c_LeftEncoderPinA);
  
  _LeftEncoderTicks+=ParseEncoderLeft();
  
  _LeftEncoderAPrev = _LeftEncoderASet;
  _LeftEncoderBPrev = _LeftEncoderBSet;
}

// Interrupt service routines for the right motor's quadrature encoder
void HandleRightMotorInterruptB(){
  // Test transition;
  _RightEncoderBSet = digitalRead(c_RightEncoderPinB);
  _RightEncoderASet = digitalRead(c_RightEncoderPinA);
  
  _RightEncoderTicks+=ParseEncoderRight();
  
  _RightEncoderAPrev = _RightEncoderASet;
  _RightEncoderBPrev = _RightEncoderBSet;
}

int ParseEncoderLeft(){
  if(_LeftEncoderAPrev && _LeftEncoderBPrev){
    if(!_LeftEncoderASet && _LeftEncoderBSet) return 1;
    if(_LeftEncoderASet && !_LeftEncoderBSet) return -1;
  }else if(!_LeftEncoderAPrev && _LeftEncoderBPrev){
    if(!_LeftEncoderASet && !_LeftEncoderBSet) return 1;
    if(_LeftEncoderASet && _LeftEncoderBSet) return -1;
  }else if(!_LeftEncoderAPrev && !_LeftEncoderBPrev){
    if(_LeftEncoderASet && !_LeftEncoderBSet) return 1;
    if(!_LeftEncoderASet && _LeftEncoderBSet) return -1;
  }else if(_LeftEncoderAPrev && !_LeftEncoderBPrev){
    if(_LeftEncoderASet && _LeftEncoderBSet) return 1;
    if(!_LeftEncoderASet && !_LeftEncoderBSet) return -1;
  }}
  
 int ParseEncoderRight(){
  if(_RightEncoderAPrev && _RightEncoderBPrev){
    if(!_RightEncoderASet && _RightEncoderBSet) return 1;
    if(_RightEncoderASet && !_RightEncoderBSet) return -1;
  }else if(!_RightEncoderAPrev && _RightEncoderBPrev){
    if(!_RightEncoderASet && !_RightEncoderBSet) return 1;
    if(_RightEncoderASet && _RightEncoderBSet) return -1;
  }else if(!_RightEncoderAPrev && !_RightEncoderBPrev){
    if(_RightEncoderASet && !_RightEncoderBSet) return 1;
    if(!_RightEncoderASet && _RightEncoderBSet) return -1;
  }else if(_RightEncoderAPrev && !_RightEncoderBPrev){
    if(_RightEncoderASet && _RightEncoderBSet) return 1;
    if(!_RightEncoderASet && !_RightEncoderBSet) return -1;
}

 }
