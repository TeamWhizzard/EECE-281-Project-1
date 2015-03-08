#include "Tone.h"

int pinPiezoTreble = 5;
int pinPiezoBass = 9;

Tone treble;
//Tone bass;

void setup() {
  
  treble.begin(pinPiezoTreble);
  //bass.begin(pinPiezoBass);
  
  treble.play(440, 1000); // frequency, duration
  //bass.play(NOTE_A4);
}

void loop() {
  
}
