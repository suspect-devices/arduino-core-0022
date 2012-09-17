/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly. 
  This example code is in the public domain.
 */

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#ifndef BOARD_LED
#define BOARD_LED 13
#endif

void setup();
void loop();

void setup() {
  pinMode(BOARD_LED, OUTPUT);
}

void loop() {
  digitalWrite(BOARD_LED, HIGH);   // set the LED on
  delay(1000);                     // wait for a second
  digitalWrite(BOARD_LED, LOW);    // set the LED off
  delay(500);                      // wait for a half a second
}

