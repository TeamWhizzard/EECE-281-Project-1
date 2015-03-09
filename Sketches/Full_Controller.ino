#include <LiquidCrystal.h>
#include<Wire.h>
#include<Math.h>

#define RS_PIN   4
#define E_PIN    7
#define D4_PIN   8
#define D5_PIN   9
#define D6_PIN  10
#define D7_PIN  11
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(RS_PIN, E_PIN, D4_PIN, D5_PIN, D6_PIN, D7_PIN);

const int MPU=0x68;  // I2C address of the MPU-6050
const int readDelay = 150;   //Delay to read new acceleration/angular velocity values
const int FORWARD_THRESHOLD = -2500;   //Acceleration (scaled) threshold value to speed up
const int SLOWDOWN_THRESHOLD = 2500;  //Accereration (scaled) threshold value to slow down
const int COUNT_ROTATE_THRESHOLD = 125;  //Threshold of gyro rotation before counting it(due to hovering values at rest)
const int RIGHT_END_BOUNDARY = 90;  //End of degree boundary to indicate turning right
const int RIGHT_START_BOUNDARY = 30;  //Start of degree boundary indicate turning right
const int LEFT_START_BOUNDARY = -30;  //Start of degree boundary to indicate turning left
const int LEFT_END_BOUNDARY = -90;  //End of degree boundary to indicate turning left 
const int FS_ZERO_GYRO_SCALE = 131;  //Scale factor of angular velocity readings
const int RADIAN_TO_DEGREES = 180/PI;
const int alpha = 0.92;  //Complimentary filter coefficient 
float totalDegrees, gyroDegrees, xTilt,yTilt, zTilt;  //Variables to keep track of filter, gyro, and accel angle measurements
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ, Tmp;   //Raw readings from the MP6050
int AcXOffset = 0; int AcYOffset = 0; int AcZOffset = 1860;     //Offset to normalize accell readings to 0 (except AcZ -> gravity=16384)
int GyXOffset = -110; int GyYOffset= -140; int GyZOffset= -54;  //Offset to normalize gyro readings to 0

enum controlState {  //enumeration to cleanly maintain state of controller
  left,
  center,
  right
};
controlState state = center;

void setup(){
      Wire.begin();
      Wire.beginTransmission(MPU);
      Wire.write(0x6B);  // PWR_MGMT_1 register
      Wire.write(0);     // set to zero (wakes up the MPU-6050)
      Wire.endTransmission(true);
      lcd.begin(16, 2);
      Serial.begin(9600);
      Serial.println("C");  
    }

//Updates all acceleration and angular velocity readings
void readAll(){
      Wire.beginTransmission(MPU);
      Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
      AcX=Wire.read()<<8|Wire.read() + AcXOffset;  //  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
      AcY=Wire.read()<<8|Wire.read() + AcYOffset;  // // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      AcZ=Wire.read()<<8|Wire.read() + AcZOffset;  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
      GyX=Wire.read()<<8|Wire.read() + GyXOffset;  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
      GyY=Wire.read()<<8|Wire.read() + GyYOffset;  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      GyZ=Wire.read()<<8|Wire.read() + GyZOffset;  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

}

//Prints all the acceleration and angular velocity readings
void printAll(){
      //Print out the acceleration values for the x, y, z axises (scaled by a factor of G in m/s^2)
      Serial.print("AcX = "); Serial.print(AcX);
      Serial.print(" | AcY = "); Serial.print(AcY);
      Serial.print(" | AcZ = "); Serial.print(AcZ);
      //equation for temperature in degrees C from datasheet
      Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  
      //Print out the angular velocity values about the x, y, z axises (scaled by a factor in degrees/second)
      Serial.print(" | GyX = "); Serial.print(GyX);
      Serial.print(" | GyY = "); Serial.print(GyY);
      Serial.print(" | GyZ = "); Serial.println(GyZ);
}

//Counts the degrees rotated 
void countRotate() {
    readAll();  //Update all accel/gyro values
    float time = millis();
    while (abs(GyY) >= COUNT_ROTATE_THRESHOLD){  //Only loop (count degrees) when the controller is moving 
      checkRotateControl();
      //checkSpeedControl();
      gyroDegrees = totalDegrees;   //Set gyro angle to last measured filter angle
      delay(readDelay);    //Delay for the given duration inbetween reads
      float GyYdps = GyY / FS_ZERO_GYRO_SCALE;   //Convert gyro measurement to degrees per second
      processTilt(); //Acquire the degree angle from the accellerometer
      float currDegrees = GyYdps * (millis()- time) / 1000;  //Perform approximate integration by multiplying degree per second with the delay
      time = millis();  //Time the new duration right after initial sum calculated 
      gyroDegrees += currDegrees;  //Iteratively sum the current degrees into total degrees
      totalDegrees = calcFilterAngle(gyroDegrees, xTilt);  //Calculate the filter angle
      //Serial.print("Degrees per Second: ");  //Print out the dps reading
      //Serial.println(GyYdps);
      //Serial.print("Total Degrees turned: ");  //Print out total degrees turned so far
      //Serial.println(totalDegrees);  
      readAll();
  }
}

//Checks for a turn motion and sets motors to turn appropriately.
void checkSpeedControl(){
  readAll();  //Update accell readings
  if (AcY <= FORWARD_THRESHOLD){  //If acceleration is greater than the threshold, speed up
    Serial.println("A");
    delay(1500);
  }
  else if (AcY >= SLOWDOWN_THRESHOLD){  //Else check if its lower than the threshold, slow down
    Serial.println("B");
    delay(1500);
  }
}

//Checks whether the controller is in a boundary zone, and make the robot turn accordingly. 
void checkRotateControl(){
      //In the left boundary zone? Turn robot left while controller in left zone
      if (totalDegrees >= RIGHT_START_BOUNDARY && totalDegrees <= RIGHT_END_BOUNDARY && state != right){
        Serial.println("R");
        state = right;
      }
      //In the center boundary zone? Center the robot while controller in center zone
      else if (totalDegrees > LEFT_START_BOUNDARY && totalDegrees < RIGHT_START_BOUNDARY && state != center ){
        Serial.println("C");
        state = center; 
      }
      //In the right boundary zone? Turn robot right while controller in right zone. 
      else if (totalDegrees >= LEFT_END_BOUNDARY && totalDegrees <= LEFT_START_BOUNDARY  && state != left){
        Serial.println("L");
        state = left;
      }
}

//Calibrates the offsets of the accell/gyro readings to normalize their values to 0 or G (for AcZ)
void calibrateError(){
  float sumAcXDiff,sumAcYDiff,sumAcZDiff,sumGyXDiff,sumGyYDiff,sumGyZDiff = 0;  //Sum of offsets to be divided by reading count to obtain average later
  float i;
  for (i = 0; i < 100; i++){  //Increase i to take more readings as needed
    readAll();
    printAll();
    sumAcXDiff += 0 - AcX; sumAcYDiff += 0 - AcY; sumAcZDiff += 16384 - AcZ;  //Sum up accell offsets
    sumGyXDiff += 0 - GyX; sumGyYDiff += 0 - GyY; sumGyZDiff += 0 - GyZ;  //Sum up gyro offsets
    delay(readDelay);
  }
  AcXOffset = sumAcXDiff/(i); AcYOffset = sumAcYDiff/(i); AcZOffset = sumAcZDiff/(i);  //Average all accell offset measurements
  GyXOffset = sumGyXDiff/(i); GyYOffset = sumGyYDiff/(i); GyZOffset = sumGyZDiff/(i);  //Average all gyro offset measurements
  Serial.println(AcXOffset); Serial.println(AcYOffset); Serial.println(AcZOffset);
  Serial.println(GyXOffset); Serial.println(GyYOffset); Serial.println(GyZOffset);

}
void processTilt(){
  xTilt = atan2(AcX,sqrt(pow(AcY, 2) + pow(AcZ, 2))) * RADIAN_TO_DEGREES;
  //yTilt = atan2(AcY,sqrt(pow(AcX, 2)+ pow(AcZ, 2))) * RADIAN_TO_DEGREES;
  //zTilt = atan2(sqrt(pow(AcX, 2)+ pow(AcY, 2)),AcZ) * RADIAN_TO_DEGREES;
//  Serial.print("X Tilt = "); Serial.print(xTilt);
//  Serial.print(" | Y Tilt = "); Serial.print(yTilt);
//  Serial.print(" | Z Tilt = "); Serial.println(zTilt);
//  Serial.println("-----");
  
}

//Perform the complimentary filter to the gyro and accell angle for the best angle result
int calcFilterAngle(float gyroAngle, float acelAngle){
  return alpha * gyroAngle + (1 - alpha) * acelAngle;
}

void parseSerialData(){
  while (Serial.available() > 0) {
    int rightMotor;
    int leftMotor;
    int rightEncoder;
    int leftEncoder;
    int music;

    rightMotor = Serial.parseInt();
    leftMotor = Serial.parseInt();
    rightEncoder = Serial.parseInt();
    leftEncoder = Serial.parseInt();
    music = Serial.parseInt();

    int encoderDelta = leftEncoder - rightEncoder;
    
    lcdPrintMotorDebug(rightMotor, leftMotor, encoderDelta);
  }
}

void lcdPrintMotorDebug(int right, int left, int encoder) {
    lcd.clear();

    lcd.leftToRight();
    lcd.setCursor(0, 0);
    lcd.print("Left ");
    lcd.setCursor(8, 0);
    lcd.print("Right ");

    lcd.rightToLeft();
    lcd.setCursor(3, 1);
    lcd.print(left);

    lcd.setCursor(11, 1);
    lcd.print(right);

    if (encoder != 0) {
      if (encoder > 0) {
        lcd.setCursor(6, 1);
      } else {
        lcd.setCursor(14, 1);
      }
      lcd.print(encoder);
    } else { // motors are equal 
        lcd.setCursor(6, 1);
        lcd.print("   ");
        lcd.setCursor(14, 1);
        lcd.print("   ");
    }
}
void loop(){
      countRotate();  //While the controller is moving, record total degrees that the controller turned
      parseSerialData(); //Parse the serial data passed to the controller and print onto LCD
      //readAll();
      //printAll();  //Print accellerometer/gyroscope reading values
      
}
