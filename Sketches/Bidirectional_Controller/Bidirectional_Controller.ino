#include <LiquidCrystal.h>

#define RS_PIN   4
#define E_PIN    7
#define D4_PIN   8
#define D5_PIN   9
#define D6_PIN  10
#define D7_PIN  11

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(RS_PIN, E_PIN, D4_PIN, D5_PIN, D6_PIN, D7_PIN);

void setup() {
  // set up the LCD's number of columns and rows:
  Serial.begin(9600);
  lcd.begin(16, 2);
  // Print a message to the LCD.
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
\
     lcd.clear();

     lcd.setCursor(0, 0);     
     lcd.print("R: ");
     lcd.setCursor(3, 0);
     lcd.print(rightMotor);
     lcd.setCursor(8, 0);
     lcd.print(rightEncoder);

     lcd.setCursor(0, 1);
     lcd.print("L: ");
     lcd.setCursor(3, 1);
     lcd.print(leftMotor);
     lcd.setCursor(8, 1);
     lcd.print(leftEncoder);
    }
}

