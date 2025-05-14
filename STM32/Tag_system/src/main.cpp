#include <Arduino.h>

// Use a known PWM-capable pin (PA8 for STM32F4)
const int ledPin = PA5;

void setup()
{
  pinMode(ledPin, OUTPUT);
}

void loop()
{
  // Fade in
  for (int duty = 0; duty <= 255; duty++)
  {
    analogWrite(ledPin, duty);
    delay(10);
  }

  // Fade out
  for (int duty = 255; duty >= 0; duty--)
  {
    analogWrite(ledPin, duty);
    delay(10);
  }
}
