// pindefines.h
// Name of every used pin on the STM32 Nucleo-F410RB
// Pinout reference: https://os.mbed.com/platforms/ST-Nucleo-F410RB/

#pragma once
#include <Arduino.h>

// Motor Control - Yaw (First Motor)
const int PWM_YAW           = PA_9;     // PWM for yaw motor
const int YAW_DIR_RIGHT     = PA_8;     // Direction control for yaw motor (right)
const int YAW_DIR_LEFT      = PB_10;    // Direction control for yaw motor (left)
const int ENCODER_A_YAW     = PA_0;     // Encoder A for yaw motor
const int ENCODER_B_YAW     = PA_1;     // Encoder B for yaw motor

// Motor Control - Pitch (Second Motor)
const int PWM_PITCH         = PA_7;     // PWM for pitch motor
const int PITCH_DIR_UP      = PB_6;     // Direction control for pitch motor (up)
const int PITCH_DIR_DOWN    = PB_7;     // Direction control for pitch motor (down)
const int ENCODER_A_PITCH   = PA_2;     // Encoder A for pitch motor
const int ENCODER_B_PITCH   = PA_3;     // Encoder B for pitch motor

// Microswitches
const int MS_YAW_RIGHT      = PC_13;    // Microswitch yaw - right limit
const int MS_YAW_LEFT       = PC_14;    // Microswitch yaw - left limit
const int MS_PITCH_UP       = PC_15;    // Microswitch pitch - up limit
const int MS_PITCH_DOWN     = PB_5;     // Microswitch pitch - down limit

// Emergency Stop
const int EMERGENCY_STOP    = PB_4;     // Emergency stop input
// const int shutdown_system   = NAN;      //

// SPI Interface
const int SPI_SCK           = PA_5;     // SPI clock
const int SPI_MISO          = PA_6;     // SPI MISO
const int SPI_MOSI          = PA_7;     // SPI MOSI
const int SPI_CS            = PA_4;     // SPI chip select
