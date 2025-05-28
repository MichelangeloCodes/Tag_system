// src/main_nucleo.cpp  (SPI slave on Nucleo F410)
#include <Arduino.h>
#include <SPI.h>

#define BUFFER_SIZE 4
volatile uint8_t rxBuffer[BUFFER_SIZE];
volatile uint8_t txBuffer[BUFFER_SIZE] = {0x11, 0x22, 0x33, 0x44};
volatile uint8_t bufIndex = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Initialize SPI pins via Arduino API
  SPI.begin();              // configure PA5/SCK, PA6/MISO, PA7/MOSI, PA4/NSS as AF

  // Switch to slave mode: clear MSTR, enable SPI
  SPI1->CR1 &= ~SPI_CR1_MSTR;  // slave mode
  SPI1->CR1 |= SPI_CR1_SPE;    // enable SPI peripheral

  // Hardware NSS on PA4 will gate SPI; ensure pin is AF
  // (configured by SPI.begin())

  // Enable RXNE interrupt
  SPI1->CR2 |= SPI_CR2_RXNEIE;
  NVIC_EnableIRQ(SPI1_IRQn);
}

void loop() {
  static uint32_t last = millis();
  if (millis() - last >= 1000) {
    last = millis();
    Serial.print("Slave RX: ");
    for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
      uint8_t b = rxBuffer[i];
      if (b < 0x10) Serial.print("0x0"); else Serial.print("0x");
      Serial.print(b, HEX);
      Serial.print(' ');
    }
    Serial.println();
  }
}

extern "C" void SPI1_IRQHandler(void) {
  if (SPI1->SR & SPI_SR_RXNE) {
    // Read incoming byte (clears RXNE)
    uint8_t received = *((__IO uint8_t *)&SPI1->DR);
    // Queue next byte for MISO
    *((__IO uint8_t *)&SPI1->DR) = txBuffer[bufIndex];
    // Store received byte
    rxBuffer[bufIndex] = received;
    bufIndex = (bufIndex + 1) % BUFFER_SIZE;
  }
}
