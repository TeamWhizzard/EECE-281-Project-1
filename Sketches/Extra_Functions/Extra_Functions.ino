#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// set the LCD address to 0x20 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol

// slave address, columns (40?), rows
LiquidCrystal_I2C lcd(0x20, 16, 2 LCD_5x8DOTS);//, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

void setup() {
  // TODO 40, 2?
  lcd.begin(16,2);         // initialize the lcd for 20 chars 4 lines and turn on backlight
  lcdDisplay();
}

void lcdDisplay(){
  lcd.backlight();
  
  lcd.setCursor(0,0);
  lcd.print("Hello, Theresa!");
  
  lcd.setCursor(1,0);
  lcd.print("Foreward");
  
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



void loop() {

}
