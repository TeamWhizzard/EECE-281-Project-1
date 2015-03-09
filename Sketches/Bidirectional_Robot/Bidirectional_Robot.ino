void setup() {
  Serial.begin(9600);
}

void stringCreate(int speedRight, int speedLeft, int encoderRight, int encoderLeft, int music){
  String num1 = String(speedRight);
  String num2 = String(speedLeft);
  String num3 = String(encoderRight);
  String num4 = String(encoderLeft);
  String num5 = String(music);
  
  String message = num1 + "," + num2 + "," + num3 + "," + num4 + "," + num5;
  bluetoothData(message);
}

void bluetoothData(String message){
  Serial.println(message);
}

void loop() {
  stringCreate(400, 400, 5, 5, 0);
  delay(2000);
  
  stringCreate(100, -100, 7, 6, 1);
  delay(2000);
  
  stringCreate(-200, 200, 4, 7, 2);
  delay(2000);
  
  stringCreate(-200, -150, 9, 8, 1);
  delay(2000);
}

