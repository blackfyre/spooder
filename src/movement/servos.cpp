#include "servos.h"

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

namespace servos
{

constexpr uint8_t DRIVER1_ADDRESS = 0x40; // legs 1-3
constexpr uint8_t DRIVER2_ADDRESS = 0x41; // legs 4-6
constexpr float SERVO_FRAME_HZ = 50;
constexpr int JOINTS_PER_LEG = 3;
constexpr int LEGS_PER_DRIVER = 3;
constexpr uint16_t FULL_OFF = 4096;   // PCA9685 full-off bit
constexpr uint16_t NOT_WRITTEN = 0xFFFF;

// Both drivers sit on Wire (the shared bus, also exposed as globalWire).
static Adafruit_PWMServoDriver driver1(DRIVER1_ADDRESS);
static Adafruit_PWMServoDriver driver2(DRIVER2_ADDRESS);

struct ServoChannel
{
  Adafruit_PWMServoDriver *driver;
  uint8_t channel;
};

struct Leg
{
  ServoChannel coxa;
  ServoChannel femur;
  ServoChannel tibia;
};

static Leg legs[kinematics::LEG_COUNT];
// Last pulse count written per leg and joint; unchanged pulses are skipped.
static uint16_t lastCount[kinematics::LEG_COUNT][JOINTS_PER_LEG];
// Microseconds per PCA9685 tick at the configured prescale.
static float tickMicroseconds = 0;
static bool enabled = true;

static void invalidateCache()
{
  for (int leg = 0; leg < kinematics::LEG_COUNT; leg++)
    for (int j = 0; j < JOINTS_PER_LEG; j++)
      lastCount[leg][j] = NOT_WRITTEN;
}

static void writeChannel(int leg, int joint, const ServoChannel &ch, uint16_t micros)
{
  uint16_t count = (uint16_t)(micros / tickMicroseconds + 0.5f);
  if (count >= FULL_OFF)
    count = FULL_OFF - 1;
  if (lastCount[leg][joint] == count)
    return;
  // setPWM returns 0 on success; a failed write stays uncached and is retried.
  if (ch.driver->setPWM(ch.channel, 0, count) == 0)
    lastCount[leg][joint] = count;
}

void setup()
{
  Serial.println("Setting up PCA Panels");
  driver1.begin();
  driver2.begin();
  driver1.setPWMFreq(SERVO_FRAME_HZ);
  driver2.setPWMFreq(SERVO_FRAME_HZ);

  // Both drivers use the same prescale; read it back once instead of on
  // every write as writeMicroseconds() would.
  uint8_t prescale = driver1.readPrescale();
  tickMicroseconds = (prescale + 1) * 1000000.0f / driver1.getOscillatorFrequency();
  Serial.print("PCA Panels initiated, prescale ");
  Serial.println(prescale);

  for (int leg = 0; leg < kinematics::LEG_COUNT; leg++)
  {
    Adafruit_PWMServoDriver *driver = leg < LEGS_PER_DRIVER ? &driver1 : &driver2;
    uint8_t base = (leg % LEGS_PER_DRIVER) * JOINTS_PER_LEG;
    legs[leg] = Leg{{driver, base}, {driver, (uint8_t)(base + 1)}, {driver, (uint8_t)(base + 2)}};
  }
  invalidateCache();
  enabled = true;
}

void writeLeg(int leg, const kinematics::JointAngles &angles)
{
  if (!enabled)
    setOutputsEnabled(true);
  const Leg &l = legs[leg];
  writeChannel(leg, 0, l.coxa, kinematics::angleToMicroseconds(angles.coxa));
  writeChannel(leg, 1, l.femur, kinematics::angleToMicroseconds(angles.femur));
  writeChannel(leg, 2, l.tibia, kinematics::angleToMicroseconds(angles.tibia));
}

void setOutputsEnabled(bool on)
{
  if (on == enabled)
    return;
  invalidateCache();
  if (on)
  {
    enabled = true;
    return;
  }
  // Only report outputs as off once every channel accepted full-off; otherwise
  // the caller retries on the next tick.
  bool allOff = true;
  for (int leg = 0; leg < kinematics::LEG_COUNT; leg++)
  {
    const ServoChannel *chs[JOINTS_PER_LEG] = {&legs[leg].coxa, &legs[leg].femur, &legs[leg].tibia};
    for (int j = 0; j < JOINTS_PER_LEG; j++)
      if (chs[j]->driver->setPWM(chs[j]->channel, 0, FULL_OFF) != 0)
        allOff = false;
  }
  enabled = !allOff;
}

bool outputsEnabled()
{
  return enabled;
}

} // namespace servos
