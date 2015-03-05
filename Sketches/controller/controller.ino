// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include "DualVNH5019MotorShield.h"
#include<Wire.h>
const int MPU=0x68;  // I2C address of the MPU-6050
const int readDelay = 50;
const int RIGHT_THRESHOLD = -4000;
const int LEFT_THRESHOLD = 4000;
const int FORWARD_THRESHOLD = 4000;
const int SLOWDOWN_THRESHOLD = -4000;
const float FS_ZERO_GYRO_SCALE = 131;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

void setup(){
      Wire.begin();
      Wire.beginTransmission(MPU);
      Wire.write(0x6B);  // PWR_MGMT_1 register
      Wire.write(0);     // set to zero (wakes up the MPU-6050)
      Wire.endTransmission(true);
      Serial.begin(9600);
}
void readAll(){
      Wire.beginTransmission(MPU);
      Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
      AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
      AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
      GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
      GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

}
void printAll(){
      Serial.print("AcX = "); Serial.print(AcX);
      Serial.print(" | AcY = "); Serial.print(AcY);
      Serial.print(" | AcZ = "); Serial.print(AcZ);
      Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
      Serial.print(" | GyX = "); Serial.print(GyX);
      Serial.print(" | GyY = "); Serial.print(GyY);
      Serial.print(" | GyZ = "); Serial.println(GyZ);
}
void countRotate() {
  float totalDegrees = 0;
  float prevDegrees;
  float currDegrees;
  while (1) {
    readAll();
    float GyXdps = GyX / FS_ZERO_GYRO_SCALE;
    Serial.print("Degrees per Second: ");
    Serial.println(GyXdps);
    float currDegrees = GyXdps * readDelay / 1000;
    Serial.print("Total Degrees turned: ");
    if (abs(currDegrees) >= 0.7){
      if (prevDegrees == 0)//Start will have no prevDegrees. Slight accuracy increase. (Edge case for trap rule)
        totalDegrees += currDegrees;
      else //Trapezoidal rule
        totalDegrees += (prevDegrees+currDegrees)/2;
    }
    Serial.println(totalDegrees);
    prevDegrees = currDegrees;
    delay(readDelay);
  }
}
//Checks for a turn motion and sets motors to turn appropriately.
void checkTurnControl(){
//  if (AcZ <= RIGHT_THRESHOLD){
//    Serial.println("TURNING RIGHT");
//    delay(1500);
//    Serial.println("TURN COMPLETE");
//    
//  }
//  else if (AcZ >= LEFT_THRESHOLD){
//    Serial.println("TURNING LEFT");
//    delay(1500);
//    Serial.println("TURN COMPLETE");
//  }
  
  if (AcY >= FORWARD_THRESHOLD){
    Serial.println("SPEEDING UP");
    delay(1500);
  }
  
  else if (AcY <= SLOWDOWN_THRESHOLD){
    Serial.println("SLOWING DOWN");
    delay(1500);
  }

}


void loop(){
      readAll();
      //checkTurnControl();
      countRotate();
      printAll();
      delay(readDelay);
}

