#include "movement.h"
#include "helpers.h"

// Setup the Servo drivers
Adafruit_PWMServoDriver pcaPanel1 = Adafruit_PWMServoDriver(pwmDriver1Address);
Adafruit_PWMServoDriver pcaPanel2 = Adafruit_PWMServoDriver(pwmDriver2Address);

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

Vector3 targetCalibration = Vector3(224, 0, 116);
int inBetweenZ = -20;

void stateCalibration()
{
    currentState = Calibrate;

    bool legsUp = true;

    for (int i = 0; i < 6; i++)
    {
        if (currentPoints[i].z < inBetweenZ)
            legsUp = false;
    }

    if (!legsUp)
    {
        for (int i = 0; i < 6; i++)
        {
            float nextZ = lerp(currentPoints[i].z, inBetweenZ + 2, 0.02);
            moveToPos(i, Vector3(currentPoints[i].x, currentPoints[i].y, nextZ));
        }
    }
    else
    {
        for (int i = 0; i < 6; i++)
        {
            float nextX = min(currentPoints[i].x + 0.5, targetCalibration.x);
            float nextY = min(currentPoints[i].y + 0.5, targetCalibration.y);
            float nextZ = min(currentPoints[i].z + 0.5, targetCalibration.z);
            moveToPos(i, Vector3(nextX, nextY, nextZ));
        }
    }
}

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


/**
 * @brief Attaches the servos to the appropriate pins.
 */
void setupServos()
{
  Legs legs;
  legs.leg1.coxa.driver = pcaPanel1;
  legs.leg1.coxa.channel = 0;

  legs.leg1.femur.driver = pcaPanel1;
  legs.leg1.femur.channel = 1;

  legs.leg1.tibia.driver = pcaPanel1;
  legs.leg1.tibia.channel = 2;

  legs.leg2.coxa.driver = pcaPanel1;
  legs.leg2.coxa.channel = 3;

  legs.leg2.femur.driver = pcaPanel1;
  legs.leg2.femur.channel = 4;

  legs.leg2.tibia.driver = pcaPanel1;
  legs.leg2.tibia.channel = 5;

  legs.leg3.coxa.driver = pcaPanel1;
  legs.leg3.coxa.channel = 6;

  legs.leg3.femur.driver = pcaPanel1;
  legs.leg3.femur.channel = 7;

  legs.leg3.tibia.driver = pcaPanel1;
  legs.leg3.tibia.channel = 8;

  legs.leg4.coxa.driver = pcaPanel2;
  legs.leg4.coxa.channel = 0;

  legs.leg4.femur.driver = pcaPanel2;
  legs.leg4.femur.channel = 1;

  legs.leg4.tibia.driver = pcaPanel2;
  legs.leg4.tibia.channel = 2;

  legs.leg5.coxa.driver = pcaPanel2;
  legs.leg5.coxa.channel = 3;

  legs.leg5.femur.driver = pcaPanel2;
  legs.leg5.femur.channel = 4;

  legs.leg5.tibia.driver = pcaPanel2;
  legs.leg5.tibia.channel = 5;

  legs.leg6.coxa.driver = pcaPanel2;
  legs.leg6.coxa.channel = 6;

  legs.leg6.femur.driver = pcaPanel2;
  legs.leg6.femur.channel = 7;

  legs.leg6.tibia.driver = pcaPanel2;
  legs.leg6.tibia.channel = 8;

}

/**
 * @brief Initializes the state of the hexapod.
 */
void stateInit()
{
    moveToPos(0, Vector3(160, 0, 0));
    moveToPos(1, Vector3(160, 0, 0));
    moveToPos(2, Vector3(160, 0, 0));
    moveToPos(3, Vector3(160, 0, 0));
    moveToPos(4, Vector3(160, 0, 0));
    moveToPos(5, Vector3(160, 0, 0));

    delay(25);

    moveToPos(0, Vector3(225, 0, 115));
    moveToPos(1, Vector3(225, 0, 115));
    moveToPos(2, Vector3(225, 0, 115));
    moveToPos(3, Vector3(225, 0, 115));
    moveToPos(4, Vector3(225, 0, 115));
    moveToPos(5, Vector3(225, 0, 115));
    // return;

    delay(500);
}

// Standing Control Points Array
Vector3 SCPA[6][10];

Vector3 standingStartPoints[6];     // the points the legs are at in the beginning of the standing state
Vector3 standingInBetweenPoints[6]; // the middle points of the bezier curves that the legs will follow to smoothly transition to the end points
Vector3 standingEndPoint;

int currentLegs[3] = {-1, -1, -1};
int standLoops = 0;

/**
 * @brief Function representing the standing state of the hexapod.
 */
void stateStanding()
{
    bool moveAllAtOnce = false;
    bool highLift = false;
    setCycleStartPoints();
    standingEndPoint = Vector3(distanceFromCenter, 0, distanceFromGround + standingDistanceAdjustment);
    standLoops = 2;
    // We only set the starting, inbetween, and ending points one time, which is when we enter the standing state.
    if (currentState == Calibrate || currentState == Initialize || currentState == SlamAttack)
        moveAllAtOnce = true;
    if (currentState == SlamAttack)
        highLift = true;
    if (currentState != Stand)
    {

        set3HighestLeg();
        standLoops = 0;
        standProgress = 0;
        memcpy(standingStartPoints, currentPoints, sizeof(currentPoints[0]) * 6);
        currentState = Stand;

        // Calculate the inbetween and ending points
        for (int i = 0; i < 6; i++)
        {
            Vector3 inBetweenPoint = standingStartPoints[i];
            inBetweenPoint.x = (inBetweenPoint.x + standingEndPoint.x) / 2;
            inBetweenPoint.y = (inBetweenPoint.y + standingEndPoint.y) / 2;

            inBetweenPoint.z = ((inBetweenPoint.z + standingEndPoint.z) / 2);
            if (abs(inBetweenPoint.z - standingEndPoint.z) < 50)
                inBetweenPoint.z += 50;
            if (highLift)
                inBetweenPoint.z += 150;

            standingInBetweenPoints[i] = inBetweenPoint;

            SCPA[i][0] = standingStartPoints[i];
            SCPA[i][1] = standingInBetweenPoints[i];
            SCPA[i][2] = standingEndPoint;
        }

        for (int i = 0; i < 6; i++)
        {
            legStates[i] = Standing;
        }
    }

    // update distance from ground constantly
    for (int i = 0; i < 6; i++)
    {
        SCPA[i][2] = standingEndPoint;
    }

    // readjusting. This takes about a second
    while (standLoops < 2)
    {
        standProgress += 25;
        if (highLift)
        {
            standProgress += 40 - 50 * ((float)standProgress / points);
        }

        float t = (float)standProgress / points;
        if (t > 1)
        {
            t = 1;
        }

        if (moveAllAtOnce)
        {
            for (int i = 0; i < 6; i++)
            {
                moveToPos(i, GetPointOnBezierCurve(SCPA[i], 3, t));
            }

            if (standProgress > points)
            {
                standProgress = 0;
                standLoops = 2;
            }
        }

        else
        {
            for (int i = 0; i < 3; i++)
            {
                if (currentLegs[i] != -1)
                {
                    moveToPos(currentLegs[i], GetPointOnBezierCurve(SCPA[currentLegs[i]], 3, t));
                }
            }

            if (standProgress > points)
            {
                standProgress = 0;
                standLoops++;
                set3HighestLeg();
            }
        }
    }

    // constantly move to the standing end position
    for (int i = 0; i < 6; i++)
    {
        moveToPos(i, GetPointOnBezierCurve(SCPA[i], 3, 1));
    }
    return;
}

void set3HighestLeg()
{

    currentLegs[0] = -1;
    currentLegs[1] = -1;
    currentLegs[2] = -1;
    for (int j = 0; j < 3; j++)
    {
        for (int i = 0; i < 6; i++)
        { // go through the legs
            // if the leg is already on the list of current legs, skip it
            if (currentLegs[0] == i || currentLegs[1] == i || currentLegs[2] == i)
                continue;

            // if the leg is already in position, don't add it
            if (currentPoints[i] == standingEndPoint)
                continue;

            // if the legs z is greater than the leg already there, add it
            if (currentLegs[j] == -1 || currentPoints[i].z > currentPoints[currentLegs[j]].z)
            {
                currentLegs[j] = i;
            }
        }
    }
}

/**
 * @brief Function to put the hexapod into driving mode.
 */
void stateCar()
{
    leftSlider = (int)rc_data.slider2;
    globalSpeedMultiplier = (leftSlider + 10.0) * 0.01;
    globalRotationMultiplier = map(rc_data.slider2, 0, 100, 40, 130) * 0.01;

    if (currentState != Car || previousGait != currentGait)
    {
        currentState = Car;

        // Initialize Leg States
        for (int i = 0; i < 6; i++)
        {
            legStates[i] = Reset;
        }

        switch (currentGait)
        {
        case Tri:
            cycleProgress[0] = 0;
            cycleProgress[1] = (points / 2);
            cycleProgress[2] = 0;
            cycleProgress[3] = (points / 2);
            cycleProgress[4] = 0;
            cycleProgress[5] = (points / 2);

            pushFraction = 3.1 / 6.0;
            speedMultiplier = 1;
            strideLengthMultiplier = 1.2;
            liftHeightMultiplier = 1.1;
            maxStrideLength = 240;
            maxSpeed = 200;
            break;

        case Wave:
            // Offsets
            cycleProgress[0] = 0;
            cycleProgress[1] = (points / 6);
            cycleProgress[2] = (points / 6) * 2;
            cycleProgress[3] = (points / 6) * 5;
            cycleProgress[4] = (points / 6) * 4;
            cycleProgress[5] = (points / 6) * 3;

            // Percentage Time On Ground
            pushFraction = 5.0 / 6.0;

            speedMultiplier = 0.40;
            strideLengthMultiplier = 2;
            liftHeightMultiplier = 1.3;
            maxStrideLength = 150;
            maxSpeed = 160;
            break;

        case Ripple:
            // Offsets
            cycleProgress[0] = 0;
            cycleProgress[1] = (points / 6) * 4;
            cycleProgress[2] = (points / 6) * 2;
            cycleProgress[3] = (points / 6) * 5;
            cycleProgress[4] = (points / 6);
            cycleProgress[5] = (points / 6) * 3;

            // Percentage Time On Ground
            pushFraction = 3.2 / 6.0;

            speedMultiplier = 1;
            strideLengthMultiplier = 1.3;
            liftHeightMultiplier = 1;
            maxStrideLength = 220;
            maxSpeed = 200;
            break;

        case Bi:
            // Offsets
            cycleProgress[0] = 0;
            cycleProgress[1] = (points / 3);
            cycleProgress[2] = (points / 3) * 2;
            cycleProgress[3] = 0;
            cycleProgress[4] = (points / 3);
            cycleProgress[5] = (points / 3) * 2;

            // Percentage Time On Ground
            pushFraction = 2.1 / 6.0;

            speedMultiplier = 4;
            strideLengthMultiplier = 1;
            liftHeightMultiplier = 1.8;
            maxStrideLength = 230;
            maxSpeed = 130;
            break;

        case Quad:
            // Offsets
            cycleProgress[0] = 0;
            cycleProgress[1] = (points / 3);
            cycleProgress[2] = (points / 3) * 2;
            cycleProgress[3] = 0;
            cycleProgress[4] = (points / 3);
            cycleProgress[5] = (points / 3) * 2;

            // Percentage Time On Ground
            pushFraction = 4.1 / 6.0;

            speedMultiplier = 1;
            strideLengthMultiplier = 1.2;
            liftHeightMultiplier = 1.8;
            maxStrideLength = 220;
            maxSpeed = 200;
            break;

        case Hop:
            // Offsets
            cycleProgress[0] = 0;
            cycleProgress[1] = 0;
            cycleProgress[2] = 0;
            cycleProgress[3] = 0;
            cycleProgress[4] = 0;
            cycleProgress[5] = 0;

            // Percentage Time On Ground
            pushFraction = 3 / 6.0;

            speedMultiplier = 1;
            strideLengthMultiplier = 1.6;
            liftHeightMultiplier = 2.5;
            maxStrideLength = 240;
            maxSpeed = 200;
            break;
        }
    }

    for (int i = 0; i < 6; i++)
    {
        tArray[i] = (float)cycleProgress[i] / points;
    }

    forwardAmount = joy1CurrentMagnitude;
    turnAmount = joy2CurrentVector.x;

    moveToPos(0, getGaitPoint(0, pushFraction));
    moveToPos(1, getGaitPoint(1, pushFraction));
    moveToPos(2, getGaitPoint(2, pushFraction));
    moveToPos(3, getGaitPoint(3, pushFraction));
    moveToPos(4, getGaitPoint(4, pushFraction));
    moveToPos(5, getGaitPoint(5, pushFraction));

    float progressChangeAmount = (max(abs(forwardAmount), abs(turnAmount)) * speedMultiplier) * globalSpeedMultiplier;

    progressChangeAmount = constrain(progressChangeAmount, 0, maxSpeed * globalSpeedMultiplier);

    for (int i = 0; i < 6; i++)
    {
        cycleProgress[i] += progressChangeAmount;

        if (cycleProgress[i] >= points)
        {
            cycleProgress[i] = cycleProgress[i] - points;
        }
    }
}

Vector3 getGaitPoint(int leg, float pushFraction)
{

    float rotateStrideLength = joy2CurrentVector.x * globalRotationMultiplier;
    Vector2 v = joy1CurrentVector * Vector2(1, strideLengthMultiplier);
    v.y = constrain(v.y, -maxStrideLength / 2, maxStrideLength / 2);
    v = v * globalSpeedMultiplier;

    float weightSum = abs(forwardAmount) + abs(turnAmount);

    float t = tArray[leg];

    // if(leg == 0)print_value("cycleProgress[leg]",cycleProgress[leg]);

    // Propelling
    if (t < pushFraction)
    {
        if (legStates[leg] != Propelling)
            setCycleStartPoints(leg);
        legStates[leg] = Propelling;

        ControlPoints[0] = cycleStartPoints[leg];
        ControlPoints[1] = Vector3(v.x * strideMultiplier[leg] + distanceFromCenter, -v.y * strideMultiplier[leg], distanceFromGround).rotate(legPlacementAngle * rotationMultiplier[leg], Vector2(distanceFromCenter, 0));
        ControlPointsAmount = 2;
        Vector3 straightPoint = GetPointOnBezierCurve(ControlPoints, ControlPointsAmount, mapFloat(t, 0, pushFraction, 0, 1));

        RotateControlPoints[0] = cycleStartPoints[leg];
        RotateControlPoints[1] = {distanceFromCenter + 40, 0, distanceFromGround};
        RotateControlPoints[2] = {distanceFromCenter, rotateStrideLength, distanceFromGround};
        RotateControlPointsAmount = 3;
        Vector3 rotatePoint = GetPointOnBezierCurve(RotateControlPoints, RotateControlPointsAmount, mapFloat(t, 0, pushFraction, 0, 1));

        // if(leg == 0)print_value("pushing point",(straightPoint*abs(forwardAmount) + rotatePoint*abs(turnAmount))/ weightSum);

        return (straightPoint * abs(forwardAmount) + rotatePoint * abs(turnAmount)) / weightSum;
    }

    // Lifting
    else
    {
        if (legStates[leg] != Lifting)
            setCycleStartPoints(leg);
        legStates[leg] = Lifting;

        ControlPoints[0] = cycleStartPoints[leg];
        ControlPoints[1] = cycleStartPoints[leg] + Vector3(0, 0, liftHeight * liftHeightMultiplier);
        ControlPoints[2] = Vector3(-v.x * strideMultiplier[leg] + distanceFromCenter, (v.y + strideOvershoot) * strideMultiplier[leg], distanceFromGround + landHeight).rotate(legPlacementAngle * rotationMultiplier[leg], Vector2(distanceFromCenter, 0));
        ControlPoints[3] = Vector3(-v.x * strideMultiplier[leg] + distanceFromCenter, v.y * strideMultiplier[leg], distanceFromGround).rotate(legPlacementAngle * rotationMultiplier[leg], Vector2(distanceFromCenter, 0));
        ControlPointsAmount = 4;
        Vector3 straightPoint = GetPointOnBezierCurve(ControlPoints, ControlPointsAmount, mapFloat(t, pushFraction, 1, 0, 1));

        RotateControlPoints[0] = cycleStartPoints[leg];
        RotateControlPoints[1] = cycleStartPoints[leg] + Vector3(0, 0, liftHeight * liftHeightMultiplier);
        RotateControlPoints[2] = {distanceFromCenter + 40, 0, distanceFromGround + liftHeight * liftHeightMultiplier};
        RotateControlPoints[3] = {distanceFromCenter, -(rotateStrideLength + strideOvershoot), distanceFromGround + landHeight};
        RotateControlPoints[4] = {distanceFromCenter, -rotateStrideLength, distanceFromGround};
        RotateControlPointsAmount = 5;
        Vector3 rotatePoint = GetPointOnBezierCurve(RotateControlPoints, RotateControlPointsAmount, mapFloat(t, pushFraction, 1, 0, 1));

        // if(leg == 0)print_value("lifting point",(straightPoint*abs(forwardAmount) + rotatePoint*abs(turnAmount))/ weightSum);

        return (straightPoint * abs(forwardAmount) + rotatePoint * abs(turnAmount)) / weightSum;
    }
}

void moveToPos(int leg, Vector3 pos){
  currentPoints[leg] = pos;
  
  float dis = Vector3(0,0,0).distanceTo(pos);
  if(dis > legLength){
    print_value("Point impossible to reach", pos, false);
    print_value("Distance",dis, true);
    return;
  }

  float x = pos.x;
  float y = pos.y;
  float z = pos.z;

  float o1 = offsets[leg].x;
  float o2 = offsets[leg].y;
  float o3 = offsets[leg].z;

  float theta1 = atan2(y,x) * (180 / PI) + o1; // base angle
  float l = sqrt(x*x + y*y); // x and y extension 
  float l1 = l - a1;
  float h = sqrt(l1*l1 + z*z);

  float phi1 = acos(constrain((pow(h,2) + pow(a2,2) - pow(a3,2)) / (2*h*a2),-1,1));
  float phi2 = atan2(z, l1);
  float theta2 = (phi1 + phi2) * 180 / PI + o2;
  float phi3 = acos(constrain((pow(a2,2) + pow(a3,2) - pow(h,2)) / (2*a2*a3),-1,1));
  float theta3 = 180 - (phi3 * 180 / PI) + o3;

  targetRot = Vector3(theta1,theta2,theta3);
  
  int coxaMicroseconds = angleToMicroseconds(targetRot.x);
  int femurMicroseconds = angleToMicroseconds(targetRot.y);
  int tibiaMicroseconds = angleToMicroseconds(targetRot.z);

  switch(leg){
    case 0:
      coxa1.writeMicroseconds(coxaMicroseconds);
      femur1.writeMicroseconds(femurMicroseconds);
      tibia1.writeMicroseconds(tibiaMicroseconds);
      break;

    case 1:
      coxa2.writeMicroseconds(coxaMicroseconds);
      femur2.writeMicroseconds(femurMicroseconds);
      tibia2.writeMicroseconds(tibiaMicroseconds);
      break;

    case 2:
      coxa3.writeMicroseconds(coxaMicroseconds);
      femur3.writeMicroseconds(femurMicroseconds);
      tibia3.writeMicroseconds(tibiaMicroseconds);
      break;

    case 3:
      coxa4.writeMicroseconds(coxaMicroseconds);
      femur4.writeMicroseconds(femurMicroseconds);
      tibia4.writeMicroseconds(tibiaMicroseconds);
      break;

    case 4:
      coxa5.writeMicroseconds(coxaMicroseconds);
      femur5.writeMicroseconds(femurMicroseconds);
      tibia5.writeMicroseconds(tibiaMicroseconds);
      break;

    case 5:
      coxa6.writeMicroseconds(coxaMicroseconds);
      femur6.writeMicroseconds(femurMicroseconds);
      tibia6.writeMicroseconds(tibiaMicroseconds);
      break;

    default:
      break;
  }
  return; 
}