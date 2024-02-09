#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "../main.h"
#include "vectors.h"
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

/**
 * @file Init.h
 * @brief Contains initialization code for the servo drivers and leg structures.
 */

const uint8_t pwmDriver1Address = 0x40; // PCA9685 1
const uint8_t pwmDriver2Address = 0x41; // PCA9685 2

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)

struct ServoDriver
{
  Adafruit_PWMServoDriver driver;
  int channel;

  /**
   * @brief Writes the specified number of microseconds to the motor.
   * 
   * @param micros The number of microseconds to write.
   */
  void writeMs(uint16_t micros) {
    driver.writeMicroseconds(channel, micros);
  }
};

struct Leg {
  ServoDriver coxa;
  ServoDriver femur;
  ServoDriver tibia;
};

struct Legs
{
  Leg leg1;
  Leg leg2;
  Leg leg3;
  Leg leg4;
  Leg leg5;
  Leg leg6;
};



void setupServos();

int binomialCoefficient(int n, int k);



void setCycleStartPoints(int leg);

void setCycleStartPoints();

void moveToPos(int leg, Vector3 pos);

void set3HighestLeg();

Vector3 getGaitPoint(int leg, float pushFraction);


#endif // MOVEMENT_H