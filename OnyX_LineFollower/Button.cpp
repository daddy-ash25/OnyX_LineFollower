#include "Button.h"

// ===== CONFIG =====
const unsigned long longPressThreshold = 600;
const unsigned long debounceDelay = 50;

// ===== BUTTON ARRAY =====
Button buttons[NUM_BUTTONS] = {
  { 9, HIGH, false, false, 0 },
  { 10, HIGH, false, false, 0 },
  { 11, HIGH, false, false, 0 }
};

// ===== INIT =====
void initButtons() {
  for (int i = 0; i < NUM_BUTTONS; i++) {
    pinMode(buttons[i].pin, INPUT_PULLUP);
  }
}

// ===== CHECK =====
int buttonCheck() {
  for (int i = 0; i < NUM_BUTTONS; i++) {
    bool currentState = digitalRead(buttons[i].pin);

    if (currentState == LOW && buttons[i].lastState == HIGH) {
      delay(debounceDelay);
      if (digitalRead(buttons[i].pin) == LOW) {
        buttons[i].pressStartTime = millis();
        buttons[i].pressed = true;
        buttons[i].longPressFired = false;
      }
    }

    if (buttons[i].pressed && !buttons[i].longPressFired) {
      unsigned long heldTime = millis() - buttons[i].pressStartTime;
      if (heldTime >= longPressThreshold) {
        buttons[i].longPressFired = true;
        return 3 + i;
      }
    }

    if (currentState == HIGH && buttons[i].lastState == LOW) {
      delay(debounceDelay);
      if (digitalRead(buttons[i].pin) == HIGH && buttons[i].pressed) {
        unsigned long pressDuration = millis() - buttons[i].pressStartTime;
        buttons[i].pressed = false;

        if (!buttons[i].longPressFired && pressDuration < longPressThreshold)
          return i;
      }
    }

    buttons[i].lastState = currentState;
  }

  return -1;
}

// ===== DEBUG =====
void printButton(int result) {
  if (result != -1) {
    int btn = result % NUM_BUTTONS;
    bool isLong = result >= 3;
    Serial.print("Button ");
    Serial.print(btn);
    Serial.print(" -> ");
    Serial.println(isLong ? "Long Press" : "Short Press");
  }
}