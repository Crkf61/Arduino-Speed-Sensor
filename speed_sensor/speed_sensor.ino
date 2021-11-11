

#include "fourier.hpp"

void setup() {
  //Connect 3v3 supply to VREF pin
  //so that analog reads are mapped nicely
  Serial.begin(9600);
  analogReference(EXTERNAL);

  analogRead(AUDIO); // Make sure pin is stable

  pinMode(RED, OUTPUT);
  //pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  //pinMode(YELLOW, OUTPUT);

  Serial.println("Beginning fourier transform testing");
}

void loop() {
  while(1)
    fourier_transform_loop();
}
