#include "movement.h"
#include "../reporting/reporting.h"

// PCA9685 outputs = 12-bit = 4096 steps
// 2.5% of 20ms = 0.5ms ; 12.5% of 20ms = 2.5ms
// 2.5% of 4096 = 102 steps; 12.5% of 4096 = 512 steps

// Setup the Servo drivers
Adafruit_PWMServoDriver pcaPanel1 = Adafruit_PWMServoDriver(pwmDriver1Address);
Adafruit_PWMServoDriver pcaPanel2 = Adafruit_PWMServoDriver(pwmDriver2Address);


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

Legs legs;

/**
 * Linearly interpolates between two values.
 *
 * @param a The starting value.
 * @param b The ending value.
 * @param f The interpolation factor (0.0 to 1.0).
 * @return The interpolated value between a and b.
 */
float lerp(float a, float b, float f)
{
    return a * (1.0 - f) + (b * f);
}

/**
 * Linearly interpolates between two vectors.
 *
 * @param a The starting vector.
 * @param b The ending vector.
 * @param f The interpolation factor (0.0 to 1.0).
 * @return The interpolated vector.
 */
Vector2 lerp(Vector2 a, Vector2 b, float f)
{
    return Vector2(lerp(a.x, b.x, f), lerp(a.y, b.y, f));
}

/**
 * Calculates the hypotenuse of a right triangle using the Pythagorean theorem.
 *
 * @param x The length of one side of the triangle.
 * @param y The length of the other side of the triangle.
 * @return The length of the hypotenuse.
 */
float calculateHypotenuse(float x, float y)
{
    float result = sqrt(pow(x, 2) + pow(y, 2));
    return result;
}

/**
 * Maps a value from one range to another range.
 *
 * @param x The value to be mapped.
 * @param in_min The minimum value of the input range.
 * @param in_max The maximum value of the input range.
 * @param out_min The minimum value of the output range.
 * @param out_max The maximum value of the output range.
 * @return The mapped value.
 */
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Converts an angle to a pulse value.
 *
 * @param ang The angle to be converted.
 * @return The pulse value corresponding to the given angle.
 */
int angleToPulse(int ang)
{
    int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX); // map angle of 0 to 180 to Servo min and Servo max
    Serial.print("Angle: ");
    Serial.print(ang);
    Serial.print(" pulse: ");
    Serial.println(pulse);
    return pulse;
}

/**
 * Converts an angle in degrees to microseconds.
 * 
 * @param angle The angle in degrees.
 * @return The corresponding time in microseconds.
 */
int angleToMicroseconds(double angle) {
  double val = 500.0 + (((2500.0 - 500.0) / 180.0) * angle);
  return (int)val;
}


void stateCalibration()
{
    Serial.println("Setting Calibration state");
    currentState = Calibrate;

    Serial.println("Preparing to pull up legs!");
    bool legsUp = true;

    Serial.println("Checking last state!");
    for (int i = 0; i < 6; i++)
    {
        if (currentPoints[i].z < inBetweenZ)
            legsUp = false;
    }

    Serial.println("Moving legs!");
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

/**
 * @brief Attaches the servos to the appropriate pins.
 */
void setupServos()
{

    Serial.println("Setting up PCA Panels");
    pcaPanel1.begin();
    pcaPanel2.begin();
    Serial.println("PCA Panels initiated");

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

    stateInit();
}

// Standing Control Points Array
Vector3 SCPA[6][10];

Vector3 standingStartPoints[6];     // the points the legs are at in the beginning of the standing state
Vector3 standingInBetweenPoints[6]; // the middle points of the bezier curves that the legs will follow to smoothly transition to the end points
Vector3 standingEndPoint;

int currentLegs[3] = {-1, -1, -1};
int standLoops = 0;

void setCycleStartPoints(int leg){
  cycleStartPoints[leg] = currentPoints[leg];    
}

void setCycleStartPoints(){
  for(int i = 0; i < 6; i++){
    cycleStartPoints[i] = currentPoints[i]; 
  }     
}

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
    // leftSlider = (int)rc_data.slider2;
    leftSlider = 44;
    globalSpeedMultiplier = (leftSlider + 10.0) * 0.01;
    // globalRotationMultiplier = map(rc_data.slider2, 0, 100, 40, 130) * 0.01;
    // globalRotationMultiplier = map(44, 0, 100, 40, 130) * 0.01;

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

void moveToPos(int leg, Vector3 pos)
{
    currentPoints[leg] = pos;

    float dis = Vector3(0, 0, 0).distanceTo(pos);
    if (dis > legLength)
    {
        print_value("Point impossible to reach", pos, false);
        print_value("Distance", dis, true);
        return;
    }

    float x = pos.x;
    float y = pos.y;
    float z = pos.z;

    float o1 = offsets[leg].x;
    float o2 = offsets[leg].y;
    float o3 = offsets[leg].z;

    float theta1 = atan2(y, x) * (180 / PI) + o1; // base angle
    float l = sqrt(x * x + y * y);                // x and y extension
    float l1 = l - a1;
    float h = sqrt(l1 * l1 + z * z);

    float phi1 = acos(constrain((pow(h, 2) + pow(a2, 2) - pow(a3, 2)) / (2 * h * a2), -1, 1));
    float phi2 = atan2(z, l1);
    float theta2 = (phi1 + phi2) * 180 / PI + o2;
    float phi3 = acos(constrain((pow(a2, 2) + pow(a3, 2) - pow(h, 2)) / (2 * a2 * a3), -1, 1));
    float theta3 = 180 - (phi3 * 180 / PI) + o3;

    targetRot = Vector3(theta1, theta2, theta3);

    int coxaMicroseconds = angleToMicroseconds(targetRot.x);
    int femurMicroseconds = angleToMicroseconds(targetRot.y);
    int tibiaMicroseconds = angleToMicroseconds(targetRot.z);

    Leg legsArray[6] = {legs.leg1, legs.leg2, legs.leg3, legs.leg4, legs.leg5, legs.leg6};
    legsArray[leg].coxa.writeMs(coxaMicroseconds);
    legsArray[leg].femur.writeMs(femurMicroseconds);
    legsArray[leg].tibia.writeMs(tibiaMicroseconds);
    return;
}