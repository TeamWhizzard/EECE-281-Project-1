#ifndef WhizzardLCD_H
#define WhizzardLCD_H

#include "Arduino.h"

// LCD
// NOTE: These are **NOT** Arduino pins, they are only needed to be delcared so we can use the LCD library.
#define I2C_ADDR  0x27 // I2C Address for LCD Display
#define BACKLIGHT_PIN  3
#define EN_PIN  2
#define RW_PIN  1
#define RS_PIN  0
#define D4_PIN  4
#define D5_PIN  5
#define D6_PIN  6
#define D7_PIN  7
#define BLOCK   0xFF // block character code

class WhizzardLCD {
 public:
  WhizzardLCD();
  void print(String line);
  void init();
  void lcdRefresh(float distance);
  void clearLine(int line);
  void lcdDisplay(float distance);
  void lcdDisplay2(String message);
 private:
  // array of custom characters that each represent a different amount of lines filled
  // in a single block on an lcd. Broken into 5 segments
  byte slices[6][8] = {
    {B00000, B00000, B00000, B00000, B00000, B00000, B00000, B00000},
    {B10000, B10000, B10000, B10000, B10000, B10000, B10000, B10000},
    {B11000, B11000, B11000, B11000, B11000, B11000, B11000, B11000},
    {B11100, B11100, B11100, B11100, B11100, B11100, B11100, B11100},
    {B11110, B11110, B11110, B11110, B11110, B11110, B11110, B11110},
    {B11111, B11111, B11111, B11111, B11111, B11111, B11111, B11111}
  };
  
  int distanceLast;     // keeps track of last distance measurement
  void distanceLive(int distanceNew, float distanceCm);
};

#endif
