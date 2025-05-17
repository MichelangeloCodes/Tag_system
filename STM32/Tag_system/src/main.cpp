#include <Arduino.h>
#include "pindefines.h"

volatile bool buttonPressed = false;    // Flag to indicate button press or release
volatile bool lastButtonState = HIGH;   // The last button state
unsigned long lastDebounceTime = 0;     // The last time the button state was toggled
unsigned long debounceDelay = 50;       // Debounce time (ms)

void handleButton() {
  // Read the current button state
  bool currentButtonState = digitalRead(BUTTON_PIN);

  // Check if the button state changed (ignores button bounce)
  if (currentButtonState != lastButtonState) {
    lastDebounceTime = millis();  // Reset debounce timer
    lastButtonState = currentButtonState; // Update last button state

    buttonPressed = true;  // Set the flag to process the button event
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);  // Pin for the LED
  digitalWrite(LED_PIN, LOW); // Ensure LED is off initially

  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Button connected to ground (NC configuration)
  
  // Trigger interrupt on both falling and rising edge (button press and release)
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButton, CHANGE);  // Trigger on any change (both press and release)

  Serial.println("Interrupt with debounce setup complete!");
}

void loop() {
  // Only process button press if debounce time has passed
  if (buttonPressed) {
    unsigned long currentMillis = millis();
    
    // Check if enough time has passed since the last button press (debouncing)
    if (currentMillis - lastDebounceTime > debounceDelay) {
      buttonPressed = false;  // Reset the interrupt flag after debounce period

      // Toggle the LED state
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));

      Serial.println("Button interrupt triggered!");
    }
  }
}
