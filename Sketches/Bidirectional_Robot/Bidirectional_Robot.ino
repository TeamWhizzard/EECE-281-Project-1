#include <Wire.h>
#include "LCD.h"
#include "LiquidCrystal_I2C.h"

#define I2C_ADDR          0x27
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7


LiquidCrystal_I2C lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);  // Set the LCD I2C address

void setup() {
  Serial.begin(9600);
  lcd.begin(16,2);
  lcd.home();
  lcd.clear();
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);
  delay(100);
}

void stringCreate(int speedRight, int speedLeft, int encoderRight, int encoderLeft, int music){
  String num1 = String(speedRight);
  String num2 = String(speedLeft);
  String num3 = String(encoderRight);
  String num4 = String(encoderLeft);
  String num5 = String(music);
  
  String message = num1 + "," + num2 + "," + num3 + "," + num4 + "," + num5;
  bluetoothData(message);
}

void bluetoothData(String message){
  Serial.println(message);
}

void loop() {
  stringCreate(400, 400, 5, 5, 0);
  delay(2000);
  
  stringCreate(100, -100, 7, 6, 1);
  delay(2000);
  
  stringCreate(-200, 200, 4, 7, 2);
  delay(2000);
  
  stringCreate(-200, -150, 9, 8, 1);
  delay(2000);
}

