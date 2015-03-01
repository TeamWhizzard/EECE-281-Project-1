#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield motors;

void setup()
{
  Serial.begin(115200);
  motors.init();
}

void loop()
{
  motors.setSpeeds(100, 100);
  Serial.print("M1 current: ");
  Serial.println(motors.getM1CurrentMilliamps());
  Serial.print("M2 current: ");
  Serial.println(motors.getM2CurrentMilliamps());
  delay(1000);
}
