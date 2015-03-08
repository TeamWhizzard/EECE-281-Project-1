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
  //Serial.begin(57600);
  lcd.begin(16,2);
  lcd.home();
  lcd.clear();
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);
  delay(100);
}

void loop() {
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Hello there!");
  delay(10000);
}
