#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "../main.h"
#include "vectors.h"
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include "bezier.h"

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

const Vector3 offsets1 = {90, 75, -18};
const Vector3 offsets2 = {93, 75, -15};
const Vector3 offsets3 = {93, 75, -18};
const Vector3 offsets4 = {87, 80, -26};
const Vector3 offsets5 = {85, 89, -16};
const Vector3 offsets6 = {93, 85, -24};
const Vector3 offsets[6] = {offsets1, offsets2, offsets3, offsets4, offsets5, offsets6};

const float a1 = 41;  // Coxa Length
const float a2 = 116; // Femur Length
const float a3 = 183; // Tibia Length
float legLength = a1 + a2 + a3;

Vector3 currentPoints[6];
Vector3 cycleStartPoints[6];

Vector3 currentRot(180, 0, 180);
Vector3 targetRot(180, 0, 180);

float strideMultiplier[6] = {1, 1, 1, -1, -1, -1};
float rotationMultiplier[6] = {-1, 0, 1, -1, 0, 1};

Vector3 ControlPoints[10];
Vector3 RotateControlPoints[10];

Vector3 AttackControlPoints[10];

// Hexapod state management
enum State
{
    Initialize,
    Stand,
    Car,
    Calibrate,
    SlamAttack
};

enum LegState
{
    Propelling,
    Lifting,
    Standing,
    Reset
};

enum Gait
{
    Tri,
    Wave,
    Ripple,
    Bi,
    Quad,
    Hop
};

int totalGaits = 6;
Gait gaits[6] = {Tri, Wave, Ripple, Bi, Quad, Hop};

float points = 1000;
int cycleProgress[6];
LegState legStates[6];
int standProgress = 0;

State currentState = Initialize;
Gait currentGait = Tri;
Gait previousGait = Tri;
int currentGaitID = 0;

float standingDistanceAdjustment = 0;

float distanceFromGroundBase = -60;
float distanceFromGround = 0;
float previousDistanceFromGround = 0;

float liftHeight = 130;
float landHeight = 70;
float strideOvershoot = 10;
float distanceFromCenter = 190;

float crabTargetForwardAmount = 0;
float crabForwardAmount = 0;

Vector2 joy1TargetVector = Vector2(0, 0);
float joy1TargetMagnitude = 0;

Vector2 joy1CurrentVector = Vector2(0, 0);
float joy1CurrentMagnitude = 0;

Vector2 joy2TargetVector = Vector2(0, 0);
float joy2TargetMagnitude = 0;

Vector2 joy2CurrentVector = Vector2(0, 0);
float joy2CurrentMagnitude = 0;

unsigned long timeSinceLastInput = 0;

float landingBuffer = 15;

int attackCooldown = 0;
long elapsedTime = 0;
long loopStartTime = 0;

Vector3 targetCalibration = Vector3(224, 0, 116);
int inBetweenZ = -20;

float forwardAmount;
float turnAmount;
float tArray[6];
int ControlPointsAmount = 0;
int RotateControlPointsAmount = 0;
float pushFraction = 3.0 / 6.0;
float speedMultiplier = 0.5;
float strideLengthMultiplier = 1.5;
float liftHeightMultiplier = 1.0;
float maxStrideLength = 200;
float maxSpeed = 100;
float legPlacementAngle = 56;

int leftSlider = 50;
float globalSpeedMultiplier = 0.55;
float globalRotationMultiplier = 0.55;


void setupServos();


Vector2 GetPointOnBezierCurve(Vector2* points, int numPoints, float t) {
  Vector2 pos;

  for (int i = 0; i < numPoints; i++) {
    float b = binomialCoefficient(numPoints - 1, i) * pow(1 - t, numPoints - 1 - i) * pow(t, i);
    pos.x += b * points[i].x;
    pos.y += b * points[i].y;
  }

  return pos;
}


Vector3 GetPointOnBezierCurve(Vector3* points, int numPoints, float t) {
  Vector3 pos;

  for (int i = 0; i < numPoints; i++) {
    float b = binomialCoefficient(numPoints - 1, i) * pow(1 - t, numPoints - 1 - i) * pow(t, i);
    pos.x += b * points[i].x;
    pos.y += b * points[i].y;
    pos.z += b * points[i].z;
  }

  return pos;
}

int binomialCoefficient(int n, int k) {
  int result = 1;

  // Calculate the binomial coefficient using the formula:
  // (n!) / (k! * (n - k)!)
  for (int i = 1; i <= k; i++) {
    result *= (n - (k - i));
    result /= i;
  }

  return result;
}


void setCycleStartPoints(int leg){
  cycleStartPoints[leg] = currentPoints[leg];    
}

void setCycleStartPoints(){
  for(int i = 0; i < 6; i++){
    cycleStartPoints[i] = currentPoints[i]; 
  }     
}


#endif