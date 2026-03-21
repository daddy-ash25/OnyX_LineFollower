#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

#define NUM_BUTTONS 3

// Struct for button
struct Button {
  int pin;
  bool lastState;
  bool pressed;
  bool longPressFired;
  unsigned long pressStartTime;
};

// ===== FUNCTIONS =====
void initButtons();
int buttonCheck();
void printButton(int result);

#endif