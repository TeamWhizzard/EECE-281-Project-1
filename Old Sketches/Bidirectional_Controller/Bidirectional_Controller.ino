#include <LiquidCrystal.h>

#define RS_PIN   4
#define E_PIN    7
#define D4_PIN   8
#define D5_PIN   9
#define D6_PIN  10
#define D7_PIN  11

#define BLOCK   0xFF // block character

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(RS_PIN, E_PIN, D4_PIN, D5_PIN, D6_PIN, D7_PIN);

byte slices[6][8] = {
  {B00000, B00000, B00000, B00000, B00000, B00000, B00000, B00000},
  {B10000, B10000, B10000, B10000, B10000, B10000, B10000, B10000},
  {B11000, B11000, B11000, B11000, B11000, B11000, B11000, B11000},
  {B11100, B11100, B11100, B11100, B11100, B11100, B11100, B11100},
  {B11110, B11110, B11110, B11110, B11110, B11110, B11110, B11110},
  {B11111, B11111, B11111, B11111, B11111, B11111, B11111, B11111}
};



void setup() {
  // set up the LCD's number of columns and rows:
  Serial.begin(9600);
  lcd.begin(16, 2);        // initialize the lcd for 16 chars 2 lines and turn on backlight
  lcd.home();
  lcd.clear();
  for (int i = 0; i < 6; i++) {
    lcd.createChar(i, slices[i]);
  }
}

void clearLine(int line) {
  lcd.setCursor(0, line);
  lcd.print("                ");
  delay(20);
  lcd.setCursor(0, line);
}

void lcdPrintSpeed(int val){
  if(val >= 100)
    lcd.print(val);
  else if(val < 100 && val >= 10){
    lcd.print(" ");
    lcd.print(val);
  }
  else{
    lcd.print("  ");
    lcd.print(val);
  }
}

void lcdEncoderVal(int num){
  lcd.print("+");
  lcd.print(num);
}

void lcdDisplay(int left, int right, int encoder) {
  lcd.setCursor(0,0);
  
  lcd.print("L:");
  lcdPrintSpeed(left);
  if(encoder < 0)
    lcdEncoderVal(abs(encoder));
  else
    lcd.print("   ");
    
  lcd.setCursor(8,0);
  lcd.print("R:");
  lcdPrintSpeed(right);
  if(encoder > 0)
    lcdEncoderVal(encoder);
  else
    lcd.print("   ");
}

void lcdMotorSpeedGraph(int left, int right){
  clearLine(1);
  if(left < 0){
    if(left <= 10){
      while(left >= 5){
        lcd.print(BLOCK);
        left -= 5;
      }
    }
    lcd.write(byte(left));
  }
  lcd.setCursor(8,1);
  if(right < 0){
    if(right <= 10){
      while(right >= 5){
        lcd.print(BLOCK);
        left -= 5;
      }
    }
    lcd.write(byte(right));
  }
}

void lcdRefresh(int left, int right, int encoder) {
  lcdDisplay(left, right, encoder);
  lcdMotorSpeedGraph(left/10, right/10);
  delay(100);
}

void loop() {
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

    int encoderDelta = rightEncoder - leftEncoder;
    
    lcdRefresh(leftMotor, rightMotor, encoderDelta);
  }
}

/*
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
*/
