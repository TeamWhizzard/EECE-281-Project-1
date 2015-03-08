#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

#define I2C_ADDR    0x27 // <<----- Add your address here.  Find it from I2C Scanner
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7

int distanceLast; // keeps track of last moisture reading
char block = 0xFF;
float distanceCm = 0;

// custom character that produces a blank block on the lcd
byte lineZero[8] = {
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000
};
// custom character that produces a single line on the lcd
byte lineOne[8] = {
    B10000,
    B10000,
    B10000,
    B10000,
    B10000,
    B10000,
    B10000,
    B10000
};
// custom character that produces a double line on the lcd
byte lineTwo[8] = {
    B11000,
    B11000,
    B11000,
    B11000,
    B11000,
    B11000,
    B11000,
    B11000
};
// custom character that produces a triple line on the lcd
byte lineThree[8] = {
    B11100,
    B11100,
    B11100,
    B11100,
    B11100,
    B11100,
    B11100,
    B11100
};
// custom character that produces a quadruple line on the lcd
byte lineFour[8] = {
    B11110,
    B11110,
    B11110,
    B11110,
    B11110,
    B11110,
    B11110,
    B11110
};
// custom character that produces a full block on the lcd
byte lineFive[8] = {
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111
};


// set the LCD address to 0x27 for a 16 chars 2 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol

LiquidCrystal_I2C lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);  // Set the LCD I2C address

void setup() {
  lcd.begin(16,2);         // initialize the lcd for 16 chars 2 lines and turn on backlight
  Serial.begin(9600);
  lcd.clear();
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.createChar(0, lineZero);
  lcd.createChar(1, lineOne);
  lcd.createChar(2, lineTwo);
  lcd.createChar(3, lineThree);
  lcd.createChar(4, lineFour);
  lcd.createChar(5, lineFive);
  delay(100);
}

// clears a given line on the lcd
void clearLine(int line){
  lcd.setCursor(0,line);
  lcd.print("                ");
  delay(20);
  lcd.setCursor(0,line);
}

// distanceLive() - displays bar representation of most recent distance reading on bottom line of LCD display
void distanceLive(int distanceNew){
  // graphs values 13cm and less
  if(distanceCm <= 13){
    // we assume that when distanceCm equals zero, that the range is beyond 4meters (outside the range of the sensor)
    if(distanceCm == 0){
      // fill up graph on bottom of the LCD
      lcd.setCursor(0,1);
      for(int i = 0; i < 16; i++){
        lcd.print((String) block);
        delay(30);
      }
      distanceNew = 15;
    }
    // clear the bottom of the LCD
    else if(distanceCm <= 3){
      clearLine(1);
      distanceNew = 0;
    }
    // depending on the value between 4 and 13 it fills in two blocks of the LCD using lines thus splitting two blocks into 10 segments
    else if(distanceCm >= 4 && distanceCm <= 13){
      int charVal = distanceCm - 3;
      clearLine(1);
      // 
      if(charVal > 4){
        lcd.print((String) block);
        charVal = charVal - 5;
      }
      lcd.write(byte(charVal));
    }
  }
  
  // graphs values greater than 13cm
  else{
    // when distance increases, so then the graph increases
    if(distanceNew > distanceLast) {
      lcd.setCursor(0,1);
      lcd.print((String) block);
      for (int i = 0; i <= distanceNew; i++) {
        lcd.print((String) block);
        delay(30);
      }
    }
    // when distance decreases, so then the graph decreases
    else if(distanceNew < distanceLast) {
      for (int i = distanceLast+1; i > distanceNew; i--) {
        lcd.setCursor(i,1);
        lcd.print(" ");
        delay(30);
      }
    }
  }
  distanceLast = distanceNew;
}

// Displays the distance measurement on the top line of the LCD
void lcdDisplay(){
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Distance: ");
  distanceCm /= 100; // convert from centimeters to meters
  
  // Displays "Oodles" when the distance recorded is zero since the range sensor is limited to 4 meters
  if(distanceCm == 0){
    lcd.print("      ");
    lcd.setCursor(10,0);
    lcd.print("Oodles");
  }
  
  // Displays the number value of the distance recorded
  else{
    lcd.setCursor(14,0);
    lcd.print(" ");
    lcd.setCursor(10,0);
    lcd.print(distanceCm);
    lcd.setCursor(15,0);
    lcd.print("m");
      
  }
  delay(500);
}

// Updates the LCD calling both the written and graphing functions lcdDisplay() and distanceLive() respectively
void lcdRefresh(){
  // controls blocks 2 to 7 on the lcd using a range from 14 to 100cm
  if(distanceCm <= 100 && distanceCm >= 14){
    distanceLive(distanceCm * 5 / 86 + (51/43)); // division will pass an int within range 0 to 7 based on tested distance thresholds
  }
  // controls blocks 8 to 15 on the lcd using a range from 101 to 400cm
  else if(distanceCm > 100){
    distanceLive(distanceCm * 7 / 299 + (1685/299)); // division will pass an int within range 8 to 16 based on tested distance thresholds
  }
  // deals with distance values less that 14cm, including zero
  else{
    distanceLive(0); // division will pass 0
  }
  lcdDisplay();
  delay(100);
}


/*void collision(){
  motors.setBrakes(400, 400);
  
  speed1 *= -1;
  motors.setSpeeds(speed1, speed1);
  delay(500);
  motors.setBrakes(400, 400);
  
  turnRight();
  reportDistance();
  float pathRight = distanceCm;
  turnRight();
  reportDistance();
  float pathBack = distanceCm;
  turnRight();
  reportDistance();
  float pathLeft = distanceCm;
  
  if(pathLeft >= 50 || pathLeft == 0){
    break;
  }
  else if((pathLeft >= pathRight && pathLeft >= pathBack) && pathRight != 0 && pathBack != 0){
    break;
  }
  else if(((pathBack >= pathRight) && pathRight != 0) || pathBack == 0){
    turnLeft();
    break;
  }
  else{
    turnBackLeft();
    break;
  }
}*/

// testing loop changed to suite different needs
void loop() {
  distanceCm = 0;
  lcdRefresh();
  distanceCm = 1;
  lcdRefresh();
  distanceCm = 2;
  lcdRefresh();
  distanceCm = 3;
  lcdRefresh();
  distanceCm = 4;
  lcdRefresh();
  distanceCm = 5;
  lcdRefresh();
  distanceCm = 6;
  lcdRefresh();
  distanceCm = 7;
  lcdRefresh();
  distanceCm = 8;
  lcdRefresh();
  distanceCm = 9;
  lcdRefresh();
  distanceCm = 10;
  lcdRefresh();
  distanceCm = 11;
  lcdRefresh();
  distanceCm = 12;
  lcdRefresh();
  distanceCm = 13;
  lcdRefresh();
}
