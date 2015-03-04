// Pacman music!
// Opening theme song on button push
// Play Death when robot runs into a wall
// Play Munch when starts going in particular direction

#include "lab2constants.h"

const int ON = 1;
const int OFF = 0;

const int songTheme = 0;
const int songMunch = 1;
const int songDeath = 2;

const int SIXTYFOURTH = 200; // one sixty-fourth note has a duration of ms

int pinPiezoTreble = 3;
int pinPiezoBass = 9;
int pinButton = 13;

int buttonState = OFF;

void setup() {
  Serial.begin(115200);
  
  pinMode(pinPiezoTreble, OUTPUT);
  pinMode(pinPiezoBass, OUTPUT);
  pinMode(pinButton, OUTPUT);

}

void loop() {
  //Serial.println(buttonState);
  //buttonState = digitalRead(pinButton);
  
  //if (buttonState == ON) { // play music
    playSong(songTheme);
    delay(2000);
  //}
}

void playSong (int song) {
  if (song == songTheme) {
    int numNotes = sizeof(theme_melody) / sizeof(int);
    
	for (int note = 0; note < numNotes; note++) {
		tone (pinPiezoTreble, theme_melody[note], SIXTYFOURTH); // port, note frequency, duration
		tone (pinPiezoBass, theme_bass[note], SIXTYFOURTH); // port, note frequency, duration
	}
  } else if (song == songMunch) {
	  // TODO
    
  } else { // song == songDeath
	// TODO
  }
}
