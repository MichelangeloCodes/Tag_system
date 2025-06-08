#include <Servo.h>

// Create servo objects
Servo pitchServo;
Servo yawServo;

// Define servo control pins
const int pitchServoPin = 9;
const int yawServoPin = 10;

// Serial input handling
String inputString = "";
bool newData = false;

void setup() {
  Serial.begin(9600);
  pitchServo.attach(pitchServoPin);
  yawServo.attach(yawServoPin);

  Serial.println("=== Servo Control Initialized ===");
  Serial.println("Enter angle using:");
  Serial.println("  pitch:<angle> to set pitch servo");
  Serial.println("  yaw:<angle> to set yaw servo");
  Serial.println("Example: pitch:45 or yaw:120");
}

void loop() {
  receiveSerialInput();
  if (newData) {
    processInput(inputString);
    inputString = "";
    newData = false;
  }
}

// Read serial input until newline character
void receiveSerialInput() {
  while (Serial.available() > 0) {
    char incomingChar = Serial.read();
    if (incomingChar == '\n') {
      newData = true;
    } else {
      inputString += incomingChar;
    }
  }
}

// Process the user's input and move the appropriate servo
void processInput(String input) {
  input.trim(); // remove whitespace
  input.toLowerCase(); // make it case-insensitive

  if (input.startsWith("pitch:")) {
    int angle = input.substring(6).toInt();
    angle = constrain(angle, 0, 180);
    pitchServo.write(angle);
    Serial.print("Pitch angle set to: ");
    Serial.print(angle);
    Serial.println("°");

  } else if (input.startsWith("yaw:")) {
    int angle = input.substring(4).toInt();
    angle = constrain(angle, 0, 180);
    yawServo.write(angle);
    Serial.print("Yaw angle set to: ");
    Serial.print(angle);
    Serial.println("°");

  } else {
    Serial.println("⚠ Invalid input. Use: pitch:<angle> or yaw:<angle>");
  }
}
