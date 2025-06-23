// pindefines.h
// Name of every used pin on the STM32 Nucleo-F410RB
// Pinout reference: https://os.mbed.com/platforms/ST-Nucleo-F410RB/

#pragma once

// Motor Control - Yaw (First Motor)
const int PWM_YAW = D9;       // PWM for yaw motor
const int YAW_DIR_RIGHT = D8; // PA_8; // Direction control for yaw motor (right)
const int YAW_DIR_LEFT = D7;  // PB_10; // Direction control for yaw motor (left)
const int ENCODER_A_YAW = A0; // PA_0; // Encoder A for yaw motor
const int ENCODER_B_YAW = A1; // PA_1; // Encoder B for yaw motor

// Motor Control - Pitch (Second Motor)
const int PWM_PITCH = D6;       // PA_10;      // PWM for pitch motor
const int PITCH_DIR_UP = D5;    // PB_4;    // Direction control for pitch motor (up)
const int PITCH_DIR_DOWN = D4;  // PB_5;  // Direction control for pitch motor (down)
const int ENCODER_A_PITCH = A5; // PA_5; // Encoder A for pitch motor
const int ENCODER_B_PITCH = D3; // PB_3; // Encoder B for pitch motor

// Microswitches
const int MS_YAW = PA_11;   // Microswitch yaw - right limit
const int MS_PITCH = PA_12; // Microswitch yaw - left limit

// SPI Interface
const int SPI_SCK = PA_5;  // SPI clock
const int SPI_MISO = PA_6; // SPI MISO
const int SPI_MOSI = PA_7; // SPI MOSI
const int SPI_CS = PA_4;   // SPI chip select

// ACS712 current sensor
// const int ACS_PIN = A5;