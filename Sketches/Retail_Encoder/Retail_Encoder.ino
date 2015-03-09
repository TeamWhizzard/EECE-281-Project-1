#define leftEncoderInterrupt 0
#define leftEncoderPin 2
volatile int leftEncoderState;
volatile long leftEncoderTicks = 0;

void setup()
{
  Serial.begin(9600);
  // Left encoder
  pinMode(leftEncoderPin, INPUT); // sets pin A as input
  attachInterrupt(leftEncoderInterrupt, HandleLeftInterrupt, CHANGE);
}

void loop()
{
  Serial.print("EncoderLeft Ticks: ");
  Serial.print(leftEncoderTicks);
  Serial.print(" RevolutionsLeft: ");
  Serial.print(leftEncoderTicks / 10.0); //4000 Counts Per Revolution
  Serial.println("");
}

// Interrupt service routines for the left motor
void HandleLeftInterrupt() {
  leftEncoderState = digitalRead(leftEncoderPin);
  leftEncoderTicks += ParseEncoderLeft();
}

int ParseEncoderLeft() {
  if ((leftEncoderPrevState) && (!leftEncoderState)) {
    return 1;
  } else if ((!leftEncoderPrevState) && (!leftEncoderState)) {
    return 0;
  }
}

