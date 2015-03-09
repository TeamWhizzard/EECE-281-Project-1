// #
// # Editor     : Lauren from DFRobot
// # Date       : 17.01.2012

// # Product name: Wheel Encoders for DFRobot 3PA and 4WD Rovers
// # Product SKU : SEN0038

// # Description:
// # The sketch for using the encoder on the DFRobot Mobile platform

// # Connection:
// #        left wheel encoder  -> Digital pin 2
// #        right wheel encoder -> Digital pin 3
// #

#define LEFT 0
#define RIGHT 1
// Motors
#define RIGHT_MOTOR_CONTROL  4    // Right motor Direction Control
#define RIGHT_MOTOR_SPEED    5    // Right motor Speed Control
#define LEFT_MOTOR_CONTROL   7    // Left motor Direction Control
#define LEFT_MOTOR_SPEED     6    // Left motor Speed Control

long coder[2] = {0, 0};
int lastSpeed[2] = {0, 0};

void setup() {
  Serial.begin(9600);                            //init the Serial port to print the data
  for (int i = 4; i <= 7; i++) {                 // Motor Pin Assignments
    pinMode(i, OUTPUT);
  }
  attachInterrupt(LEFT, LwheelSpeed, CHANGE);    //init the interrupt mode for the digital pin 2
  attachInterrupt(RIGHT, RwheelSpeed, CHANGE);   //init the interrupt mode for the digital pin 3
}

void loop() {

  static unsigned long timer = 0;                //print manager timer
  forward(175, 175);
  
  if (millis() - timer > 100) {
    Serial.print("Coder value: ");
    Serial.print(coder[LEFT]);
    Serial.print("[Left Wheel] ");
    Serial.print(coder[RIGHT]);
    Serial.println("[Right Wheel]");

    lastSpeed[LEFT] = coder[LEFT];   //record the latest speed value
    lastSpeed[RIGHT] = coder[RIGHT];
    coder[LEFT] = 0;                 //clear the data buffer
    coder[RIGHT] = 0;
    timer = millis();
  }
}


void LwheelSpeed()
{
  coder[LEFT] ++;  //count the left wheel encoder interrupts
}


void RwheelSpeed()
{
  coder[RIGHT] ++; //count the right wheel encoder interrupts
}

// stops both motors
void brake() {
  digitalWrite(RIGHT_MOTOR_SPEED, LOW); //brake right wheel
  digitalWrite(LEFT_MOTOR_SPEED, LOW); //brake left wheel
}
// moves both motors forward
void forward(char a, char b) {
  analogWrite (RIGHT_MOTOR_SPEED, a); //PWM Speed Control
  digitalWrite(RIGHT_MOTOR_CONTROL, HIGH); //right wheel moves forwards
  analogWrite (LEFT_MOTOR_SPEED, b); //PWM Speed Control
  digitalWrite(LEFT_MOTOR_CONTROL, LOW); //left wheel moves forwards, should be HIGH but reads value wrong
}
// move both motors backward
void backward(char a, char b) {
  analogWrite (RIGHT_MOTOR_SPEED, a); //PWM Speed Control
  digitalWrite(RIGHT_MOTOR_CONTROL, LOW); //right wheel moves backwards
  analogWrite (LEFT_MOTOR_SPEED, b); //PWM Speed Control
  digitalWrite(LEFT_MOTOR_CONTROL, HIGH); //left wheel moves backwards, should be LOW but reads value wrong
}
//turns right
//had to switch right and left turning due to board production issues
void turnRight(char a, char b) {
  analogWrite (RIGHT_MOTOR_SPEED, a); //PWM Speed Control
  digitalWrite(RIGHT_MOTOR_CONTROL, LOW); //right wheel moves backwards
  analogWrite (LEFT_MOTOR_SPEED, b); //PWM Speed Control
  digitalWrite(LEFT_MOTOR_CONTROL, LOW); //left wheel moves forwards, should be HIGH but reads value wrong
}
//turns left
//had to switch right and left turning due to board production issues
void turnLeft(char a, char b) {
  analogWrite (RIGHT_MOTOR_SPEED, a); //PWM Speed Control
  digitalWrite(RIGHT_MOTOR_CONTROL, HIGH); //right wheel moves forwards
  analogWrite (LEFT_MOTOR_SPEED, b); //PWM Speed Control
  digitalWrite(LEFT_MOTOR_CONTROL, HIGH); //left wheel moves backwards, should be LOW but reads value wrong
}

// turns the robot 90 degrees to the right
void turn90Right() {
  turnLeft(150, 150);
  delay(500);
  brake();
}

// turns the robot 90 degrees to the left
void turn90Left() {
  turnRight(150, 150);
  delay(500);
  brake();
}

