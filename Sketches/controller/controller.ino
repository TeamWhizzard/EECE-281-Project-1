// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain

#include<Wire.h>
const int MPU=0x68;  // I2C address of the MPU-6050
const int readDelay = 50;   //Delay to read new acceleration/angular velocity values
const int FORWARD_THRESHOLD = 4000;   //Acceleration (scaled) threshold value to speed up
const int SLOWDOWN_THRESHOLD = -4000;  //Accereration (scaled) threshold value to slow down
const int COUNT_ROTATE_THRESHOLD = 100;  //Threshold of gyro rotation before counting it(due to hovering values at rest)
const int RIGHT_END_BOUNDARY = -90;  //End of degree boundary to indicate turning right
const int RIGHT_START_BOUNDARY = -20;  //Start of degree boundary indicate turning right
const int LEFT_START_BOUNDARY = 20;  //Start of degree boundary to indicate turning left
const int LEFT_END_BOUNDARY = 90;  //End of degree boundary to indicate turning left 
const int FS_ZERO_GYRO_SCALE = 131;  //Scale factor of angular velocity readings
float totalDegrees = 0;  //Variable to keep track of total degrees turned, with the controller centered being 0 degrees
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;  //Variables to keep track of acceleration/gyroscope values in all axises
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
      Serial.begin(9600);
}

//Updates all acceleration and angular velocity readings
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
    while (GyX > COUNT_ROTATE_THRESHOLD){  //Only loop (count degrees) when the controller is moving 
      float GyXdps = GyX / FS_ZERO_GYRO_SCALE;
      float currDegrees = GyXdps * readDelay / 1000;  //Perform approximate integration by multiplying degree per second with the delay
      if (abs(currDegrees) >= 0.7){  //Gyro readings hover below 0.7 dps at rest, don't take these values
          totalDegrees += currDegrees;  //Iteratively sum the current degrees into total degrees
      }
      Serial.print("Degrees per Second: ");  //Print out the dps reading
      Serial.println(GyXdps);
      Serial.print("Total Degrees turned: ");  //Print out total degrees turned so far
      Serial.println(totalDegrees);  
      delay(readDelay);    //Delay for the given duration inbetween reads
  } 
}

//Checks for a turn motion and sets motors to turn appropriately.
void checkTurnControl(){
  readAll();  //Update accell readings
  if (AcY >= FORWARD_THRESHOLD){  //If acceleration is greater than the threshold, speed up
    Serial.println("SPEEDING UP");
    delay(1500);
  }
  else if (AcY <= SLOWDOWN_THRESHOLD){  //Else check if its lower than the threshold, slow down
    Serial.println("SLOWING DOWN");
    delay(1500);
  }
}

//Checks whether the controller is in a boundary zone, and make the robot turn accordingly. 
void checkRotateControl(){
      //In the left boundary zone? Turn robot left while controller in left zone
      if (totalDegrees >= LEFT_END_BOUNDARY || totalDegrees <= LEFT_START_BOUNDARY && state != left){
        Serial.print("TURNING LEFT");
        state = left;
      }
      //In the center boundary zone? Center the robot while controller in center zone
      else if (totalDegrees > LEFT_START_BOUNDARY || totalDegrees < RIGHT_START_BOUNDARY && state != center ){
        Serial.print("CENTERED");
        state = center; 
      }
      //In the right boundary zone? Turn robot right while controller in right zone. 
      else if (totalDegrees >= RIGHT_START_BOUNDARY || totalDegrees <= RIGHT_END_BOUNDARY  && state != right){
        Serial.print("TURNING RIGHT");
        state = right;
      }
}

void loop(){
      countRotate();  //While the controller is moving, record total degrees that the controller turned
      checkRotateControl();  //Perform the appropriate action on the robot according to degrees turned
      printAll();  //Print accellerometer/gyroscope reading values
}
