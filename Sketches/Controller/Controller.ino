#include <LiquidCrystal.h>   //For LCD functionality
#include <Wire.h>            //For Gyro/Accell functionality
#include <Math.h>            //For calculating angles (arctan function)
#include "WhizzardTone.h"    //For playing Pacman music

//LCD Constants
#define RS_PIN     8
#define E_PIN      9
#define D4_PIN    10
#define D5_PIN    11
#define D6_PIN    12
#define D7_PIN    13

//Automatic, Manual Mode
#define BUTTON    7
#define MANUAL    1
#define AUTO      0
int lastButtonState;

// block character
#define BLOCK   0xFF

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(RS_PIN, E_PIN, D4_PIN, D5_PIN, D6_PIN, D7_PIN);

// Pacman Tone Variables
// Variables to relate songs to a number
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

// Initalize the Tone objects to be used for treble and bass
Tone treble;
Tone bass;

// Assign buzzer pins
const int pinPiezoTreble = 5;
const int pinPiezoBass = 6;

// MP6050 Gyro/Accell Tracking Variables
const int MPU = 0x68; // I2C address of the MPU-6050
const int readDelay = 150;   //Delay to read new acceleration/angular velocity values
const int FORWARD_THRESHOLD = -8600;   //Acceleration (scaled) threshold value to speed up
const int BACKWARD_THRESHOLD = 10000;  //Accereration (scaled) threshold value to slow down
const int STOP_THRESHOLD = 21000;
const int COUNT_ROTATE_THRESHOLD = 125;  //Threshold of gyro rotation before counting it(due to hovering values at rest)
const int RIGHT_END_BOUNDARY = 90;  //End of degree boundary to indicate turning right
const int RIGHT_START_BOUNDARY = 30;  //Start of degree boundary indicate turning right
const int LEFT_START_BOUNDARY = -30;  //Start of degree boundary to indicate turning left
const int LEFT_END_BOUNDARY = -90;  //End of degree boundary to indicate turning left
const int FORWARD_END_BOUNDARY = 90;  //End of degree boundary to indicate forward
const int FORWARD_START_BOUNDARY = 27;  //Start of degree boundary indicate forward
const int STOP_START_BOUNDARY = -27;  //Start of degree boundary to indicate stop
const int STOP_END_BOUNDARY = -90;  //End of degree boundary to indicate stop
const int FS_ZERO_GYRO_SCALE = 131;  //Scale factor of angular velocity readings
const int RADIAN_TO_DEGREES = 180 / PI;
const int alpha = 0.92;  //Complimentary filter coefficient

float totalDegrees, gyroDegrees, totalSpeedDegrees, gyroSpeedDegrees, xTilt, yTilt, zTilt; //Variables to keep track of filter, gyro, and accel angle measurements
int AcXOffset = 0; int AcYOffset = 0; int AcZOffset = 1860;     //Offset to normalize accell readings to 0 (except AcZ -> gravity=16384)
int GyXOffset = -110; int GyYOffset = -140; int GyZOffset = -54; //Offset to normalize gyro readings to 0
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ, Tmp;   //Raw readings from the MP6050

//enumeration to cleanly maintain state of controller
enum controlTurnState {
  left,
  center,
  right
};
enum controlSpeedState {
  forward,
  middle,
  stopped
};
controlTurnState turnState = center;
controlSpeedState speedState = middle;

// LCD Custom Characters
byte slices[6][8] = {
  {B00000, B00000, B00000, B00000, B00000, B00000, B00000, B00000},
  {B10000, B10000, B10000, B10000, B10000, B10000, B10000, B10000},
  {B11000, B11000, B11000, B11000, B11000, B11000, B11000, B11000},
  {B11100, B11100, B11100, B11100, B11100, B11100, B11100, B11100},
  {B11110, B11110, B11110, B11110, B11110, B11110, B11110, B11110},
  {B11111, B11111, B11111, B11111, B11111, B11111, B11111, B11111}
};

void setup() {
  Serial.begin(9600);
  pinMode(BUTTON, INPUT);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  lcd.begin(16, 2);
  lcd.home();
  lcd.clear();
  for (int i = 0; i < 6; i++) { // Create custom LCD characters
    lcd.createChar(i, slices[i]);
  }
  treble.begin(pinPiezoTreble);
  bass.begin(pinPiezoBass);
  //bluetoothInit();
  lastButtonState = digitalRead(BUTTON);
  playSong(songTheme);
}

// Ensures a Bluetooth Connection. Does not continue until established.
void bluetoothInit() {
  while (1) {
    lastButtonState = digitalRead(BUTTON);
    Serial.println(lastButtonState);
    delay(250);
    if (Serial.available() > 0) {
      Serial.read(); // Robot has confirmed the bluetooth connection, continue with program.
      break;
    }
  }
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
void readAll() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read() + AcXOffset; //  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read() + AcYOffset; // // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read() + AcZOffset; // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read() + GyXOffset; // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read() + GyYOffset; // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read() + GyZOffset; // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

}

//Prints all the acceleration and angular velocity readings
void printAll() {
  //Print out the acceleration values for the x, y, z axises (scaled by a factor of G in m/s^2)
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  //equation for temperature in degrees C from datasheet
  Serial.print(" | Tmp = "); Serial.print(Tmp / 340.00 + 36.53);
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
  while (abs(GyY) >= COUNT_ROTATE_THRESHOLD) { //Only loop (count degrees) when the controller is moving
    checkTurnControl();
    checkSpeedControl();
    gyroDegrees = totalDegrees;  //Set gyro angle to last measured filter angle
    gyroSpeedDegrees = totalSpeedDegrees;
    delay(readDelay);    //Delay for the given duration inbetween reads
    float GyYdps = GyY / FS_ZERO_GYRO_SCALE;   //Convert gyro measurement to degrees per second
    float GyXdps = GyX / FS_ZERO_GYRO_SCALE;
    processTilt(); //Acquire the degree angle from the accellerometer
    float currDegrees = GyYdps * (millis() - time) / 1000; //Perform approximate integration by multiplying degree per second with the delay
    float currSpeedDegrees = GyXdps * (millis() - time) / 1000;
    time = millis();  //Time the new duration right after initial sum calculated
    gyroDegrees += currDegrees;  //Iteratively sum the current degrees into total degrees
    gyroSpeedDegrees += currSpeedDegrees;
    totalDegrees = calcFilterAngle(gyroDegrees, xTilt);  //Calculate the filter angle
    totalSpeedDegrees = calcFilterAngle(gyroSpeedDegrees, yTilt);
    //Serial.print("Degrees per Second: ");  //Print out the dps reading
    //Serial.println(GyYdps);
    //Serial.print("Total Degrees turned: ");  //Print out total degrees turned so far
    //Serial.println(totalDegrees);
    readAll();
  }
}

void checkSpeedControl() {
  //Tilting controller back stops robot
  if (totalSpeedDegrees >= STOP_END_BOUNDARY && totalSpeedDegrees <= STOP_START_BOUNDARY && speedState != stopped && turnState == center) {
    Serial.println("S");
    speedState = stopped;
  }
  if (totalSpeedDegrees > STOP_START_BOUNDARY && totalSpeedDegrees < FORWARD_END_BOUNDARY && speedState != stopped) {
    //Serial.print("F");
    //speedState = forward;
  }
  if (totalSpeedDegrees >= FORWARD_START_BOUNDARY && totalSpeedDegrees <= FORWARD_END_BOUNDARY && speedState != forward) {
    Serial.println("F");
    speedState = forward;
  }
}
//Checks the motion control(accell or degree position) of the controller, and make the robot turn or move accordingly.
void checkTurnControl() {
  //In the left boundary zone? Turn robot left while controller in left zone
  if (totalDegrees >= RIGHT_START_BOUNDARY && totalDegrees <= RIGHT_END_BOUNDARY && turnState != right) {
    Serial.println("R");
    turnState = right;
    speedState = forward;
  }
  //In the center boundary zone? Center the robot while controller in center zone
  else if (totalDegrees > LEFT_START_BOUNDARY && totalDegrees < RIGHT_START_BOUNDARY && turnState != center) {
    Serial.println("C");
    turnState = center;
    speedState = forward;
  }
  //In the right boundary zone? Turn robot right while controller in right zone.
  else if (totalDegrees >= LEFT_END_BOUNDARY && totalDegrees <= LEFT_START_BOUNDARY  && turnState != left) {
    Serial.println("L");
    turnState = left;
    speedState = forward;
  }
}

//Calibrates the offsets of the accell/gyro readings to normalize their values to 0 or G (for AcZ)
void calibrateError() {
  float sumAcXDiff, sumAcYDiff, sumAcZDiff, sumGyXDiff, sumGyYDiff, sumGyZDiff = 0; //Sum of offsets to be divided by reading count to obtain average later
  float i;
  for (i = 0; i < 100; i++) { //Increase i to take more readings as needed
    readAll();
    printAll();
    sumAcXDiff += 0 - AcX; sumAcYDiff += 0 - AcY; sumAcZDiff += 16384 - AcZ;  //Sum up accell offsets
    sumGyXDiff += 0 - GyX; sumGyYDiff += 0 - GyY; sumGyZDiff += 0 - GyZ;  //Sum up gyro offsets
    delay(readDelay);
  }
  AcXOffset = sumAcXDiff / (i); AcYOffset = sumAcYDiff / (i); AcZOffset = sumAcZDiff / (i); //Average all accell offset measurements
  GyXOffset = sumGyXDiff / (i); GyYOffset = sumGyYDiff / (i); GyZOffset = sumGyZDiff / (i); //Average all gyro offset measurements
  Serial.println(AcXOffset); Serial.println(AcYOffset); Serial.println(AcZOffset);     //Print out offsets for debugging purposes
  Serial.println(GyXOffset); Serial.println(GyYOffset); Serial.println(GyZOffset);

}

//Calculate angle(about x axis) via accelerometer readings
void processTilt() {
  xTilt = atan2(AcX, sqrt(pow(AcY, 2) + pow(AcZ, 2))) * RADIAN_TO_DEGREES;
  yTilt = atan2(AcY, sqrt(pow(AcX, 2) + pow(AcZ, 2))) * RADIAN_TO_DEGREES;
  //zTilt = atan2(sqrt(pow(AcX, 2)+ pow(AcY, 2)),AcZ) * RADIAN_TO_DEGREES;
  //Serial.print("X Tilt = "); Serial.print(xTilt);
  //Serial.print(" | Y Tilt = "); Serial.print(yTilt);
  //  Serial.print(" | Z Tilt = "); Serial.println(zTilt);
  //Serial.println("-----");

}

//Perform the complimentary filter to the gyro and accell angle for the best angle result
int calcFilterAngle(float gyroAngle, float acelAngle) {
  return alpha * gyroAngle + (1 - alpha) * acelAngle;
}


// LCD FUNCTIONS
void clearLine(int line) {
  lcd.setCursor(0, line);
  lcd.print("                ");
  delay(20);
  lcd.setCursor(0, line);
}

void lcdPrintSpeed(int val) {
  if (val >= 100)
    lcd.print(val);
  else if (val < 100 && val >= 10) {
    lcd.print(" ");
    lcd.print(val);
  }
  else {
    lcd.print("  ");
    lcd.print(val);
  }
}

void lcdEncoderVal(int num) {
  lcd.print("+");
  lcd.print(num);
}

void autoModeLCD() {
  while (Serial.available() > 0) {
    int rightSpeed;
    int leftSpeed;
    int rightEncoder;
    int leftEncoder;

    rightSpeed = Serial.parseInt();
    leftSpeed = Serial.parseInt();
    rightEncoder = Serial.parseInt();
    leftEncoder = Serial.parseInt();

    lcd.clear();

    lcd.leftToRight();
    lcd.setCursor(0, 0);
    lcd.print("L: ");

    lcd.rightToLeft();
    lcd.setCursor(6, 0);
    lcd.print(leftSpeed);
    lcd.setCursor(16, 0);
    lcd.print(leftEncoder);

    lcd.leftToRight();
    lcd.setCursor(0, 1);
    lcd.print("R: ");

    lcd.rightToLeft();
    lcd.setCursor(6, 1);
    lcd.print(rightSpeed);
    lcd.setCursor(16, 1);
    lcd.print(rightEncoder);
  }
}

void manualModeLCD() {
  lcd.setCursor(0, 0);
  lcd.print("Manual Mode!");
}

void debug() {
  readAll();
  processTilt();
  printAll();  //Print accellerometer/gyroscope reading values
  delay(readDelay);
}

void loop() {
  int newButtonState = digitalRead(BUTTON);
  if (newButtonState != lastButtonState) {
    Serial.println(newButtonState);
    lastButtonState = newButtonState;
  }
  if (lastButtonState == MANUAL) {
    //countRotate();  //While the controller is moving, record total degrees that the controller turned
    manualModeLCD();
    //checkTurnControl();
    //checkSpeedControl();
  } else {
    autoModeLCD(); //Checks if there is serial data passed to the controller and prints onto the LCD(fairly fast, won't interrupt angle measuring)
  }
}

