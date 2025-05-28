// src/main_arduino.cpp  (SPI master on Mega 2560)
#include <Arduino.h>
#include <SPI.h>

// SPI pins on Mega: MOSI=51, MISO=50, SCK=52, SS=53
const uint8_t SS_PIN = 53;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Configure SPI as master, mode 0
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV16);  // ~1 MHz

  // Configure SS pin
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);  // idle high
}

void loop() {
  uint8_t dataOut[4] = {0xAA, 0xBB, 0xCC, 0xDD};
  uint8_t dataIn[4] = {0};

  // Select slave
  digitalWrite(SS_PIN, LOW);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  // Transfer 4 bytes
  for (uint8_t i = 0; i < 4; i++) {
    dataIn[i] = SPI.transfer(dataOut[i]);
  }

  SPI.endTransaction();
  digitalWrite(SS_PIN, HIGH);  // Deselect slave

  // Print received bytes
  Serial.print("Master got: ");
  for (uint8_t b : dataIn) {
    if (b < 0x10) Serial.print("0x0"); else Serial.print("0x");
    Serial.print(b, HEX);
    Serial.print(' ');
  }
  Serial.println();

  delay(1000);
}

