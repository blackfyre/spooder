#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>

#ifdef ARDUINO_SAM_DUE
#define DEV_I2C Wire1
#elif defined(ARDUINO_ARCH_STM32)
#define DEV_I2C Wire
#elif defined(ARDUINO_ARCH_AVR)
#define DEV_I2C Wire
#else
#define DEV_I2C Wire
#endif

#define SerialPort Serial
#define INT_1 INT_IMU
#define LED_R 27
#define LED_G 25
#define LED_B 26
#define LED_OFF HIGH
#define LED_ON LOW

#endif