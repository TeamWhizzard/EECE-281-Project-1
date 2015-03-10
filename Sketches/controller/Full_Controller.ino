#include <LiquidCrystal.h>   //For LCD functionality
#include <Wire.h>            //For Gyro/Accell functionality
#include <Math.h>            //For calculating angles (arctan function)
#incldue "WhizzardTone.h"    //For playing Pacman music

/*-----------------LCD Variables-------------------------
*/
#define RS_PIN   4
#define E_PIN    7
#define D4_PIN   8
#define D5_PIN   9
#define D6_PIN  10
#define D7_PIN  11
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(RS_PIN, E_PIN, D4_PIN, D5_PIN, D6_PIN, D7_PIN);

/*-----------------Pacman Tone Variables---------------------------
*/
//Int variables to relate songs to a number
const int songTheme = 0;  
const int songMunch = 1;
const int songDeath = 2;
const int SIXTYFOURTH = 35; // length of sixty-fourth note in ms
const int NS = 0;  // note space - rest

//The appropriate tones to be played for the theme 
const int theme_melody[] = {
  // measure one
  NOTE_B4, NOTE_B4, NOTE_B4, NS,
  NOTE_B5, NOTE_B5, NOTE_B5, NS,
  NOTE_FS5, NOTE_FS5, NOTE_FS5, NS,
  NOTE_DS5, NOTE_DS5, NOTE_DS5, NS,
  
  NOTE_B5, NS,
  NOTE_FS5, NOTE_FS5, NOTE_FS5, NOTE_FS5, NOTE_FS5, NS,
  NOTE_DS5, NOTE_DS5, NOTE_DS5, NOTE_DS5,
  NOTE_DS5, NOTE_DS5, NOTE_DS5, NS,
  
  NOTE_C5, NOTE_C5, NOTE_C5, NS,
  NOTE_C6, NOTE_C6, NOTE_C6, NS,
  NOTE_G5, NOTE_G5, NOTE_G5, NS,
  NOTE_E5, NOTE_E5, NOTE_E5, NS,
  
  NOTE_C6, NS,
  NOTE_G5, NOTE_G5, NOTE_G5, NOTE_G5, NOTE_G5, NS,
  NOTE_E5, NOTE_E5, NOTE_E5, NOTE_E5,
  NOTE_E5, NOTE_E5, NOTE_E5, NS,
  
  // measure two
  NOTE_B4, NOTE_B4, NOTE_B4, NS,
  NOTE_B5, NOTE_B5, NOTE_B5, NS,
  NOTE_FS5, NOTE_FS5, NOTE_FS5, NS,
  NOTE_DS5, NOTE_DS5, NOTE_DS5, NS,
  
  NOTE_B5, NS,
  NOTE_FS5, NOTE_FS5, NOTE_FS5, NOTE_FS5, NOTE_FS5, NS,
  NOTE_DS5, NOTE_DS5, NOTE_DS5, NOTE_DS5,
  NOTE_DS5, NOTE_DS5, NOTE_DS5, NS,
  
  NOTE_DS5, NS,
  NOTE_E5, NS,
  NOTE_F5, NOTE_F5, NOTE_F5, NS,
  NOTE_F5, NS,
  NOTE_FS5, NS,
  NOTE_G5, NOTE_G5, NOTE_G5, NS,
  
  NOTE_G5, NS,
  NOTE_GS5, NS,
  NOTE_A5, NOTE_A5, NOTE_A5, NS,
  NOTE_B5, NOTE_B5, NOTE_B5, NOTE_B5,
  NOTE_B5, NOTE_B5, NOTE_B5, NS
};

//The appropriate bass to be played for the theme
const int theme_bass[] = {
  // measure one
  NOTE_B2, NOTE_B2, NOTE_B2, NOTE_B2,
  NOTE_B2, NOTE_B2, NOTE_B2, NOTE_B2,
  NOTE_B2, NOTE_B2, NOTE_B2, NS,
  NOTE_B3, NOTE_B3, NOTE_B3, NS,
  
  NOTE_B2, NOTE_B2, NOTE_B2, NOTE_B2,
  NOTE_B2, NOTE_B2, NOTE_B2, NOTE_B2,
  NOTE_B2, NOTE_B2, NOTE_B2, NS,
  NOTE_B3, NOTE_B3, NOTE_B3, NS,
  
  NOTE_C3, NOTE_C3, NOTE_C3, NOTE_C3,
  NOTE_C3, NOTE_C3, NOTE_C3, NOTE_C3,
  NOTE_C3, NOTE_C3, NOTE_C3, NS,
  NOTE_C4, NOTE_C4, NOTE_C4, NS,
  
  NOTE_C3, NOTE_C3, NOTE_C3, NOTE_C3,
  NOTE_C3, NOTE_C3, NOTE_C3, NOTE_C3,
  NOTE_C3, NOTE_C3, NOTE_C3, NS,
  NOTE_C4, NOTE_C4, NOTE_C4, NS,
  
  // measure two
  NOTE_B2, NOTE_B2, NOTE_B2, NOTE_B2,
  NOTE_B2, NOTE_B2, NOTE_B2, NOTE_B2,
  NOTE_B2, NOTE_B2, NOTE_B2, NS,
  NOTE_B3, NOTE_B3, NOTE_B3, NS,
  
  NOTE_B2, NOTE_B2, NOTE_B2, NOTE_B2,
  NOTE_B2, NOTE_B2, NOTE_B2, NOTE_B2,
  NOTE_B2, NOTE_B2, NOTE_B2, NS,
  NOTE_B3, NOTE_B3, NOTE_B3, NS,
  
  NOTE_FS3, NOTE_FS3, NOTE_FS3, NOTE_FS3,
  NOTE_FS3, NOTE_FS3, NOTE_FS3, NS,
  NOTE_GS3, NOTE_GS3, NOTE_GS3, NOTE_GS3,
  NOTE_GS3, NOTE_GS3, NOTE_GS3, NS,
  
  NOTE_AS3, NOTE_AS3, NOTE_AS3, NOTE_AS3,
  NOTE_AS3, NOTE_AS3, NOTE_AS3, NS,
  NOTE_B3, NOTE_B3, NOTE_B3, NOTE_B3,
  NOTE_B3, NOTE_B3, NOTE_B3, NS
};

//Initalize the Tone objects to be used for treble and bass
Tone treble;
Tone bass;

//Assign buzzer pins
const int pinPiezoTreble = 5;
const int pinPiezoBass = 6;
/*-----------------MP6050 Gyro/Accell Tracking Variables----------------------
*/
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
int AcXOffset = 0; int AcYOffset = 0; int AcZOffset = 1860;     //Offset to normalize accell readings to 0 (except AcZ -> gravity=16384)
int GyXOffset = -110; int GyYOffset= -140; int GyZOffset= -54;  //Offset to normalize gyro readings to 0
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ, Tmp;   //Raw readings from the MP6050

//enumeration to cleanly maintain state of controller
enum controlState {  
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
      treble.begin(pinPiezoTreble);
      bass.begin(pinPiezoBass);
    }

// Pacman music!
// Opening theme song on button push
// Play Death when robot runs into a wall
// Play Munch when starts going in particular direction
void playSong (int song) {
  if (song == songTheme) {
    int numNotes = sizeof(theme_melody) / sizeof(int);
    for (int note = 0; note < numNotes; note++) {
      treble.play(theme_melody[note]);
      bass.play(theme_bass[note]);
      delay(SIXTYFOURTH);
      treble.stop();
      bass.stop();
    }
  }
  //To be added if needed later (for munching and dying)
  else if (song == songMunch) {}
  else { // song == songDeath
  }
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

//Measures the degrees rotated, where center is defined as the point of system initalization.
//Tilting the controller about its x axis is what contributes to the left/right/center control
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
  Serial.println(AcXOffset); Serial.println(AcYOffset); Serial.println(AcZOffset);     //Print out offsets for debugging purposes
  Serial.println(GyXOffset); Serial.println(GyYOffset); Serial.println(GyZOffset);

}

//Calculate angle(about x axis) via accelerometer readings 
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
  //Only perform LCD actions as serial data is sent to controller
  while (Serial.available() > 0) {
    //Order of data comming from robot to be formatted to LCD
    int rightMotor;
    int leftMotor;
    int rightEncoder;
    int leftEncoder;
    int music;
    
    //Parse the data in their expected order
    rightMotor = Serial.parseInt();
    leftMotor = Serial.parseInt();
    rightEncoder = Serial.parseInt();
    leftEncoder = Serial.parseInt();
    music = Serial.parseInt();
    
    //Calculate delta of revolutions where positive means left wheel is ahead
    int encoderDelta = leftEncoder - rightEncoder;
    
    //Call this function to perform printing to LCD
    lcdPrintMotorDebug(rightMotor, leftMotor, encoderDelta);
  }
}

void lcdPrintMotorDebug(int right, int left, int encoder) {
    lcd.clear();          //Clear the LCD before printing new data

    lcd.leftToRight();    //Orient messages to be printed from left to right
    lcd.setCursor(0, 0);  //Set cursor to top left corner
    lcd.print("Left ");   //Print Left to occupy first row, 4 columns
    lcd.setCursor(8, 0);  //Set cursor to top right half corner
    lcd.print("Right ");  //Print Right to occupy first row, columns 8-12

    lcd.rightToLeft();    //Orient messages to be printed right to left
    lcd.setCursor(3, 1);  //Set cursor to bottom row, 3rd column (right under the t in left)
    lcd.print(left);      //Print the left motor data <-----

    lcd.setCursor(11, 1); //Set cursor to bottom row (right under h in right)
    lcd.print(right);     //Print the right motor data <-----
  
    if (encoder != 0) {   //If there is a delta, perform LCD actions appropriately 
      if (encoder > 0) {  //Greater than 0 (left greater)? Set cursor under left side 
        lcd.setCursor(6, 1);
      } else {            //Less than 0(right greater)? Set cursor under right side
        lcd.setCursor(14, 1);
      }
      lcd.print(encoder);  //Print the delta at the position 
    } else { // motors are equal
        //Do the same thing as if there was a delta but with blank data  
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
