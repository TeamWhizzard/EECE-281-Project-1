#include "WhizzardLCD.h"
#include "LCD.h"
#include "LiquidCrystal_I2C.h"

LiquidCrystal_I2C lcd(I2C_ADDR, EN_PIN, RW_PIN, RS_PIN, D4_PIN, D5_PIN, D6_PIN, D7_PIN); // Set the LCD I2C address

WhizzardLCD::WhizzardLCD() { /*constructor*/ }

// LCD initializer
void WhizzardLCD::init() {
  lcd.begin(16, 2);        // initialize the lcd for 16 chars 2 lines and turn on backlight
  lcd.home();
  lcd.clear();
  lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
  lcd.setBacklight(HIGH);
  delay(100);
  for (int i = 0; i < 6; i++) {
    lcd.createChar(i, slices[i]);
  }
}

/*
 *------------------------------------------------------------------------------------------
 *LCD Control Functions
 *------------------------------------------------------------------------------------------
*/

//clears a given line on the lcd
void WhizzardLCD::clearLine(int line) {
  lcd.setCursor(0, line);
  lcd.print("                ");
  delay(20);
  lcd.setCursor(0, line);
}

// parameter: distance in meters
//Displays the distance measurement on the top line of the LCD
void WhizzardLCD::lcdDisplay(float distance) {
  lcd.setCursor(0, 0);
  lcd.print("Distance: ");
  // Displays "Oodles" when the distance recorded is zero since the range sensor is limited to 4 meters
  if (distance == 0) {
    lcd.print("      ");
    lcd.setCursor(10, 0);
    lcd.print("Oodles");
  }
  // Displays the number value of the distance recorded
  else {
    lcd.setCursor(14, 0);
    lcd.print(" ");
    lcd.setCursor(10, 0);
    lcd.print(distance);
    lcd.setCursor(15, 0);
    lcd.print("m");
  }
}

// distanceLive() - displays bar representation of most recent distance reading on bottom line of LCD display
void WhizzardLCD::distanceLive(int distanceNew, float distanceCm) {
  // graphs values 13cm and less
  if (distanceCm <= 13) {
    // we assume that when distanceCm equals zero, that the range is beyond 4meters (outside the range of the sensor)
    if (distanceCm == 0) {
      // fill up graph on bottom of the LCD
      lcd.setCursor(0, 1);
      for (int i = 0; i < 16; i++) {
        lcd.print(BLOCK);
        delay(30);
      }
      distanceNew = 15;
    }
    // clear the bottom of the LCD
    else if (distanceCm <= 3) {
      clearLine(1);
      distanceNew = 0;
    }
    // depending on the value between 4 and 13 it fills in two blocks of the LCD using lines thus splitting two blocks into 10 segments
    else if (distanceCm >= 4 && distanceCm <= 13) {
      int charVal = distanceCm - 3;
      clearLine(1);
      //
      if (charVal > 4) {
        lcd.print(BLOCK);
        charVal = charVal - 5;
      }
      lcd.write(byte(charVal));
    }
  }
  // graphs values greater than 13cm
  else {
    // when distance increases, so then the graph increases
    if (distanceNew > distanceLast) {
      lcd.setCursor(0, 1);
      lcd.print(BLOCK);
      for (int i = 0; i <= distanceNew; i++) {
        lcd.print(BLOCK);
        delay(30);
      }
    }
    // when distance decreases, so then the graph decreases
    else if (distanceNew < distanceLast) {
      for (int i = distanceLast + 1; i > distanceNew; i--) {
        lcd.setCursor(i, 1);
        lcd.print(" ");
        delay(30);
      }
    }
  }
  distanceLast = distanceNew;
}

// parameter: distance in cm
// Updates the LCD calling both the written and graphing functions lcdDisplay() and distanceLive() respectively
void WhizzardLCD::lcdRefresh(float distance) {
  // controls blocks 2 to 7 on the lcd using a range from 14 to 100cm
  if (distance <= 100 && distance >= 14) {
    distanceLive((distance * 5 / 86 + (51 / 43)), distance); // division will pass an int within range 0 to 7 based on tested distance thresholds
  }
  // controls blocks 8 to 15 on the lcd using a range from 101 to 400cm
  else if (distance > 100) {
    distanceLive((distance * 7 / 299 + (1685 / 299)), distance); // division will pass an int within range 8 to 16 based on tested distance thresholds
  }
  // deals with distance values less that 14cm, including zero
  else {
    distanceLive(0, distance);
  }
  lcdDisplay(distance * 100); // passes distance in meters
  delay(100);
}
