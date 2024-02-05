#include "main.h"
#include <Adafruit_PWMServoDriver.h>
#include "headers/vectors.h"
#include "headers/Init.h"
#include "tof/tof.h"
#include "display/display.h"
#include "control/gamepad.h"


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


// Setup the Servo drivers
Adafruit_PWMServoDriver pcaPanel1 = Adafruit_PWMServoDriver(pwmDriver1Address);
Adafruit_PWMServoDriver pcaPanel2 = Adafruit_PWMServoDriver(pwmDriver2Address);



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

