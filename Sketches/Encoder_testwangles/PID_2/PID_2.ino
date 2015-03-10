// vars
#include "PID_v1.h"
#include "DualVNH5019MotorShield.h"

#define LEFT 0
#define RIGHT 1
// Motors
//#define RIGHT_MOTOR_CONTROL  4    // Right motor Direction Control
//#define RIGHT_MOTOR_SPEED    5    // Right motor Speed Control
//#define LEFT_MOTOR_CONTROL   7    // Left motor Direction Control
//#define LEFT_MOTOR_SPEED     6    // Left motor Speed Contro

#define SPEEDMAX 240

double input;
double output;
double target;
double kp = 2;
double ki = 2;
double kd = 0;

//volatile double coder[2] = {0, 0};
volatile double coderLeft = 0;
volatile double coderRight = 0;

DualVNH5019MotorShield motor;

// initialize PID - goal to stay at zero
PID myPid(&input, &output, &target, kp, ki, kd, DIRECT);

void setup() {
  Serial.begin(9600);
  pinMode (2, INPUT);
  pinMode (3, INPUT);
  motor.init();
  //for (int i = 4; i <= 7; i++) { pinMode(i, OUTPUT); } // Motor pin assignments
  
  // interrupts start counting ticks
  attachInterrupt(2, LwheelSpeed, CHANGE);    //init the interrupt mode for the digital pin 2
  attachInterrupt(3, RwheelSpeed, CHANGE);   //init the interrupt mode for the digital pin 3

  target = 0; // stay forward at zero degrees
  myPid.SetOutputLimits(-90, 90);
  myPid.SetMode(AUTOMATIC); // turn on pid
  
  motor.setSpeeds(SPEEDMAX, SPEEDMAX);
}

void loop() {
  //input = coder[RIGHT] - coder[LEFT];
  input = coderRight - coderLeft;
  Serial.print(coderRight);
  Serial.print(" ");
  Serial.println(coderLeft);
  Serial.print("Difference: ");
  Serial.println(input);
  
  myPid.Compute();  
  motor.setSpeeds(SPEEDMAX + output, SPEEDMAX - output);

  /*Serial.print("Speed left: ");
  Serial.print(SPEEDMAX + output);
  Serial.print("Speed right: ");
  Serial.println(SPEEDMAX - output);*/
  delay(250);
}

void LwheelSpeed()
{
  //coder[LEFT] ++;  //count the left wheel encoder interrupts
  coderLeft++;
}


void RwheelSpeed()
{
  //coder[RIGHT] ++; //count the right wheel encoder interrupts
  coderRight++;
}

// moves both motors forward
/*void forward(char a, char b) {
  analogWrite (RIGHT_MOTOR_SPEED, a); //PWM Speed Control
  digitalWrite(RIGHT_MOTOR_CONTROL, HIGH); //right wheel moves forwards
  analogWrite (LEFT_MOTOR_SPEED, b); //PWM Speed Control
  digitalWrite(LEFT_MOTOR_CONTROL, LOW); //left wheel moves forwards, should be HIGH but reads value wrong
}*/
