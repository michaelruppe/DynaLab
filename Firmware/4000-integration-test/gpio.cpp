#include "gpio.h"

void gpio_initialise() {
  pinMode(chA, INPUT);  // Use INPUT to ensure Pull-up/down resistors are disabled - else the signal from the reflectance sensor is corrupted.
  pinMode(chB, INPUT);
  pinMode(chC, INPUT);

  pinMode(btnRec, INPUT_PULLUP);
  pinMode(ledRec, OUTPUT);
}

void disable_linear_encoder() {
  detachInterrupt(digitalPinToInterrupt(chA));
  detachInterrupt(digitalPinToInterrupt(chB));
  detachInterrupt(digitalPinToInterrupt(chC));
}


// Read current encoder state (3-bit value)
uint8_t readEncoderState() {
  uint8_t state = 0;

  // Read pins and construct 3-bit state (A=bit2, B=bit1, C=bit0)
  if (!digitalRead(chA)) state |= 0b100;  // Assuming active LOW sensors
  if (!digitalRead(chB)) state |= 0b010;
  if (!digitalRead(chC)) state |= 0b001;

  return state;
}