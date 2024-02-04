#include "main.h"
#include <Scheduler.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
#include "headers/vectors.h"
#include "headers/Init.h"
#include "LSM6DSOXSensor.h"
#include "headers/activity_recognition.h"
#include "tof.h"
#include "display.h"
#include "gamepad.h"


// Hexapod state management
enum State {
  Initialize,
  Stand,
  Car,
  Calibrate,
  SlamAttack
};

enum LegState {
  Propelling,
  Lifting,
  Standing,
  Reset
};

enum Gait {
  Tri,
  Wave,
  Ripple,
  Bi,
  Quad,
  Hop  
};

int totalGaits = 6;
Gait gaits[6] = {Tri,Wave,Ripple,Bi,Quad,Hop};


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

Vector2 joy1TargetVector = Vector2(0,0);
float joy1TargetMagnitude = 0;

Vector2 joy1CurrentVector = Vector2(0,0);
float joy1CurrentMagnitude = 0;

Vector2 joy2TargetVector = Vector2(0,0);
float joy2TargetMagnitude = 0;

Vector2 joy2CurrentVector = Vector2(0,0);
float joy2CurrentMagnitude = 0;

unsigned long timeSinceLastInput = 0;

float landingBuffer = 15;

int attackCooldown = 0;
long elapsedTime = 0;
long loopStartTime = 0;

//Interrupts.
volatile int mems_event = 0;

// MLC
ucf_line_t *ProgramPointer;
int32_t LineCounter;
int32_t TotalNumberOfLine;

void INT1Event_cb();
void printMLCStatus(uint8_t status);

// Setup the Servo drivers
Adafruit_PWMServoDriver pcaPanel1 = Adafruit_PWMServoDriver(pwmDriver1Address);
Adafruit_PWMServoDriver pcaPanel2 = Adafruit_PWMServoDriver(pwmDriver2Address);

// Gyro
LSM6DSOXSensor AccGyr(&DEV_I2C, LSM6DSOX_I2C_ADD_L);



// Arduino setup function. Runs in CPU 1
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  //define RGB LED as outputs
  BP32.pinMode(LED_R, OUTPUT);
  BP32.pinMode(LED_G, OUTPUT);
  BP32.pinMode(LED_B, OUTPUT);
  BP32.digitalWrite(LED_R, LED_OFF);
  BP32.digitalWrite(LED_G, LED_OFF);
  BP32.digitalWrite(LED_B, LED_OFF);

  
  setupDisplay();

  // Initialize serial
  Serial.begin(115200);
  
  display.print(".");
  display.display();

  //setupGamepad();
  display.print(".");
  display.display();

  //setupMLC();
  display.print(".");
  display.display();
  
  delay(300);

  setupTOF();
  display.print(".");
  display.display();

  display.print("done!");
  display.display();

  delay(500);

  display.clearDisplay();

  //Scheduler.startLoop(loop2);

  addToLogBuffer("Ready!");
  addToLogBuffer("Waiting on gamepad...");
  // amber led represents setup complete
  digitalWrite(LED_BUILTIN, HIGH);
}

// loop for processing sensory data
void loop() {
  //TODO: rename to loopGamepad
  //loopGamepad();
  //TODO: rename to loopMLC
  //loopMLC();
  loopTOF();
  loopDisplay();
  
  delay(50);
}

// loop for acting on recieved data
// do not overwhelm i2c
//void loop2() {
  

//  delay(50);
//}

