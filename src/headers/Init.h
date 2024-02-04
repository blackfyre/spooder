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
