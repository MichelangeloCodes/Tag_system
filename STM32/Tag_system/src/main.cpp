#include <Arduino.h>
#include <PID_v1.h>
#include "pindefines.h"

// === Constants for Encoder → Degrees ===
const double COUNTS_PER_REV = 9600.0;
const double DEG_PER_COUNT = 360.0 / COUNTS_PER_REV;
const double SETPOINT_DEG = 90.0; // desired angle

// === Microswitch Debounce State ===
volatile bool buttonPressedYaw = false;
volatile bool lastButtonStateYaw = HIGH;
unsigned long lastDebounceTimeYaw = 0;
const unsigned long debounceDelayYaw = 50; // ms

volatile bool buttonPressedPitch = false;
volatile bool lastButtonStatePitch = HIGH;
unsigned long lastDebounceTimePitch = 0;
const unsigned long debounceDelayPitch = 50; // ms

// === Encoder State (counts) ===
volatile long encoderPosYaw = 0;
volatile long encoderPosPitch = 0;

// === Encoder Debounce Timing (µs) ===
volatile unsigned long lastEncAYawTime = 0;
volatile unsigned long lastEncBYawTime = 0;
volatile unsigned long lastEncAPitchTime = 0;
volatile unsigned long lastEncBPitchTime = 0;
const unsigned int ENCODER_DEBOUNCE_US = 200;

// === PID Variables ===
// Yaw
double yawInput, yawOutput, yawSetpoint;
// Pitch
double pitchInput, pitchOutput, pitchSetpoint;

// PID tuning parameters (tune these!)
double Kp_yaw = 2.0;
double Ki_yaw = 0.5;
double Kd_yaw = 0.1;

double Kp_pitch = 2.0;
double Ki_pitch = 0.5;
double Kd_pitch = 0.1;

// Create PID controllers
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, Kp_yaw, Ki_yaw, Kd_yaw, DIRECT);
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, Kp_pitch, Ki_pitch, Kd_pitch, DIRECT);

// === INTERRUPT: Microswitch YAW ===
void handleButtonYaw()
{
  bool current = digitalRead(MS_YAW);
  if (current != lastButtonStateYaw)
  {
    lastDebounceTimeYaw = millis();
    lastButtonStateYaw = current;
    buttonPressedYaw = true;
  }
}

// === INTERRUPT: Microswitch PITCH ===
void handleButtonPitch()
{
  bool current = digitalRead(MS_PITCH);
  if (current != lastButtonStatePitch)
  {
    lastDebounceTimePitch = millis();
    lastButtonStatePitch = current;
    buttonPressedPitch = true;
  }
}

// === INTERRUPTS: Encoder YAW ===
void handleEncoderAYaw()
{
  unsigned long now = micros();
  if (now - lastEncAYawTime < ENCODER_DEBOUNCE_US)
    return;
  lastEncAYawTime = now;
  bool A = digitalRead(ENCODER_A_YAW);
  bool B = digitalRead(ENCODER_B_YAW);
  encoderPosYaw += (A == B) ? 1 : -1;
}

void handleEncoderBYaw()
{
  unsigned long now = micros();
  if (now - lastEncBYawTime < ENCODER_DEBOUNCE_US)
    return;
  lastEncBYawTime = now;
  bool A = digitalRead(ENCODER_A_YAW);
  bool B = digitalRead(ENCODER_B_YAW);
  encoderPosYaw += (A != B) ? 1 : -1;
}

// === INTERRUPTS: Encoder PITCH ===
void handleEncoderAPitch()
{
  unsigned long now = micros();
  if (now - lastEncAPitchTime < ENCODER_DEBOUNCE_US)
    return;
  lastEncAPitchTime = now;
  bool A = digitalRead(ENCODER_A_PITCH);
  bool B = digitalRead(ENCODER_B_PITCH);
  encoderPosPitch += (A == B) ? 1 : -1;
}

void handleEncoderBPitch()
{
  unsigned long now = micros();
  if (now - lastEncBPitchTime < ENCODER_DEBOUNCE_US)
    return;
  lastEncBPitchTime = now;
  bool A = digitalRead(ENCODER_A_PITCH);
  bool B = digitalRead(ENCODER_B_PITCH);
  encoderPosPitch += (A != B) ? 1 : -1;
}

void setup()
{
  Serial.begin(9600);

  // Initialize debug LED pin
  pinMode(SPI_SCK, OUTPUT);
  digitalWrite(SPI_SCK, LOW);

  // Microswitches
  pinMode(MS_YAW, INPUT_PULLUP);
  pinMode(MS_PITCH, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MS_YAW), handleButtonYaw, FALLING);
  attachInterrupt(digitalPinToInterrupt(MS_PITCH), handleButtonPitch, FALLING);

  // Encoders YAW
  pinMode(ENCODER_A_YAW, INPUT_PULLUP);
  pinMode(ENCODER_B_YAW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_YAW), handleEncoderAYaw, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_YAW), handleEncoderBYaw, CHANGE);

  // Encoders PITCH
  pinMode(ENCODER_A_PITCH, INPUT_PULLUP);
  pinMode(ENCODER_B_PITCH, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PITCH), handleEncoderAPitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PITCH), handleEncoderBPitch, CHANGE);

  // Motor YAW outputs
  pinMode(PWM_YAW, OUTPUT);
  pinMode(YAW_DIR_LEFT, OUTPUT);
  pinMode(YAW_DIR_RIGHT, OUTPUT);
  digitalWrite(YAW_DIR_LEFT, HIGH);
  digitalWrite(YAW_DIR_RIGHT, LOW);

  // Motor PITCH outputs
  pinMode(PWM_PITCH, OUTPUT);
  pinMode(PITCH_DIR_UP, OUTPUT);
  pinMode(PITCH_DIR_DOWN, OUTPUT);
  digitalWrite(PITCH_DIR_UP, HIGH);
  digitalWrite(PITCH_DIR_DOWN, LOW);

  // PID setup
  yawSetpoint = SETPOINT_DEG;
  pitchSetpoint = SETPOINT_DEG;

  yawPID.SetMode(AUTOMATIC);
  yawPID.SetOutputLimits(-255, 255);

  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-255, 255);

  Serial.println("PID-controlled Yaw + Pitch system initialized.");
}

void loop()
{
  unsigned long now = millis();

  // Handle yaw microswitch
  if (buttonPressedYaw && (now - lastDebounceTimeYaw > debounceDelayYaw))
  {
    buttonPressedYaw = false;
    digitalWrite(SPI_SCK, !digitalRead(SPI_SCK));
    Serial.println("Yaw limit switch triggered.");
  }
  // Handle pitch microswitch
  if (buttonPressedPitch && (now - lastDebounceTimePitch > debounceDelayPitch))
  {
    buttonPressedPitch = false;
    Serial.println("Pitch limit switch triggered.");
  }

  // --- YAW PID computation & motor drive ---
  // Convert counts → degrees
  yawInput = encoderPosYaw * DEG_PER_COUNT;
  yawPID.Compute();
  // yawOutput in [-255,255]
  if (yawOutput >= 0)
  {
    digitalWrite(YAW_DIR_LEFT, HIGH);
    digitalWrite(YAW_DIR_RIGHT, LOW);
    analogWrite(PWM_YAW, (uint8_t)yawOutput);
  }
  else
  {
    digitalWrite(YAW_DIR_LEFT, LOW);
    digitalWrite(YAW_DIR_RIGHT, HIGH);
    analogWrite(PWM_YAW, (uint8_t)(-yawOutput));
  }

  // --- PITCH PID computation & motor drive ---
  pitchInput = encoderPosPitch * DEG_PER_COUNT;
  pitchPID.Compute();
  if (pitchOutput >= 0)
  {
    digitalWrite(PITCH_DIR_UP, HIGH);
    digitalWrite(PITCH_DIR_DOWN, LOW);
    analogWrite(PWM_PITCH, (uint8_t)pitchOutput);
  }
  else
  {
    digitalWrite(PITCH_DIR_UP, LOW);
    digitalWrite(PITCH_DIR_DOWN, HIGH);
    analogWrite(PWM_PITCH, (uint8_t)(-pitchOutput));
  }

  // Serial diagnostic (once every 200 ms)
  static unsigned long lastPrint = 0;
  if (now - lastPrint > 200)
  {
    lastPrint = now;
    Serial.print("Yaw: ");
    Serial.print(yawInput, 1);
    Serial.print("°  Out=");
    Serial.print(yawOutput);
    Serial.print("  |  Pitch: ");
    Serial.print(pitchInput, 1);
    Serial.print("°  Out=");
    Serial.println(pitchOutput);
  }
}
