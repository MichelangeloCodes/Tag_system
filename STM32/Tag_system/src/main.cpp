#include <Arduino.h>
#include "pindefines.h"

// === BUTTON / MICROSWITCH INTERRUPT STATE ===
volatile bool buttonPressed = false;
volatile bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// === ENCODER STATE ===
volatile long encoderPos = 0;

// === INTERRUPT: Microswitch ===
void handleButton() {
  bool currentButtonState = digitalRead(MS_YAW_RIGHT);

  if (currentButtonState != lastButtonState) {
    lastDebounceTime = millis();
    lastButtonState = currentButtonState;
    buttonPressed = true;
  }
}

// === INTERRUPT: Encoder A ===
void handleEncoderA() {
  bool A = digitalRead(ENCODER_A_YAW);
  bool B = digitalRead(ENCODER_B_YAW);

  if (A == B) {
    encoderPos++;
  } else {
    encoderPos--;
  }
}

// === INTERRUPT: Encoder B ===
void handleEncoderB() {
  bool A = digitalRead(ENCODER_A_YAW);
  bool B = digitalRead(ENCODER_B_YAW);

  if (A != B) {
    encoderPos++;
  } else {
    encoderPos--;
  }
}

void setup() {
  Serial.begin(9600);

  // SPI_SCK used as output to simulate LED toggle
  pinMode(SPI_SCK, OUTPUT);
  digitalWrite(SPI_SCK, LOW);

  // Microswitch setup
  pinMode(MS_YAW_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MS_YAW_RIGHT), handleButton, CHANGE);

  // Encoder pin setup
  pinMode(ENCODER_A_YAW, INPUT_PULLUP);
  pinMode(ENCODER_B_YAW, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_YAW), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_YAW), handleEncoderB, CHANGE);

  // Motor yaw setup
  pinMode(PWM_YAW, OUTPUT);
  pinMode(YAW_DIR_LEFT, OUTPUT);
  pinMode(YAW_DIR_RIGHT, OUTPUT);
  digitalWrite(YAW_DIR_RIGHT, LOW);
  digitalWrite(YAW_DIR_LEFT, HIGH);
  
  Serial.println("Microswitch & encoder setup complete.");
}

void loop() {
  unsigned long currentMillis = millis();

  // Handle debounced microswitch press
  if (buttonPressed && (currentMillis - lastDebounceTime > debounceDelay)) {
    buttonPressed = false;

    // Toggle SPI_SCK pin
    digitalWrite(SPI_SCK, !digitalRead(SPI_SCK));
    Serial.println("Microswitch interrupt triggered!");
  }

  // Print encoder position
  Serial.print("Encoder positie: ");
  Serial.println(encoderPos);

  analogWrite(PWM_YAW, 255);
}
