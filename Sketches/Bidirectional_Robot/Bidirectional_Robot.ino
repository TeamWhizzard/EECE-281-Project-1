#define LEFT  L
#define RIGHT  R
#define CENTRE  C
#define FORWARD  F
#define STOP  S
#define BACKUP  B

void setup() {
  Serial.begin(9600);
  
}

void manualDrivingMode(char heading){
  if(heading == LEFT)
      turnLeft();
    else if(heading == RIGHT)
      turnRight();
    else if(heading == CENTRE)
      straight();
    else if(heading == FORWARD)
      forward();
    else if(heading == STOP)
      brake();
    else if(heading == BACKUP)
      backward();
}

void loop(){
  while (Serial.available() > 0) {
    char heading = Serial.read();
    manualDrivingMode(heading);
  }
}
