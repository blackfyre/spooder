#include "gamepad_mapping.h"

#include <math.h>
#include "movement/gait.h"

namespace gamepad_mapping
{

static int8_t toPercent(float v)
{
  float p = roundf(v * 100.0f);
  if (p > 100)
    p = 100;
  if (p < -100)
    p = -100;
  return (int8_t)p;
}

void stickToVector(int32_t x, int32_t y, int8_t &outX, int8_t &outY)
{
  float fx = (float)x / AXIS_MAX;
  float fy = (float)y / AXIS_MAX;
  float m = sqrtf(fx * fx + fy * fy);
  if (m <= DEAD_ZONE)
  {
    outX = 0;
    outY = 0;
    return;
  }
  // Rescaled so the output rises from zero at the dead-zone edge, then clamped
  // to full scale.
  float scaled = (m - DEAD_ZONE) / (1.0f - DEAD_ZONE);
  if (scaled > 1.0f)
    scaled = 1.0f;
  float k = scaled / m;
  int ix = toPercent(fx * k);
  int iy = toPercent(fy * k);
  // Rounding each axis can push a full diagonal just past 100; pull the
  // larger component back until the magnitude is within range.
  while (ix * ix + iy * iy > 100 * 100)
  {
    int &big = (ix < 0 ? -ix : ix) >= (iy < 0 ? -iy : iy) ? ix : iy;
    big += big > 0 ? -1 : 1;
  }
  outX = (int8_t)ix;
  outY = (int8_t)iy;
}

static uint8_t stepDial(uint8_t value, int delta)
{
  int v = (int)value + delta;
  if (v < 0)
    v = 0;
  if (v > 100)
    v = 100;
  return (uint8_t)v;
}

// Tracks a two-button hold; returns true once per hold when it reaches the
// hold time.
static bool comboFired(bool down, bool &held, bool &fired, uint32_t &sinceMs, uint32_t nowMs)
{
  if (!down)
  {
    held = false;
    fired = false;
    return false;
  }
  if (!held)
  {
    held = true;
    sinceMs = nowMs;
  }
  if (!fired && nowMs - sinceMs >= COMBO_HOLD_MS)
  {
    fired = true;
    return true;
  }
  return false;
}

static RequestedMode requestedMode(const Mapper &m)
{
  if (m.calibrating)
    return RequestedMode::Calibrate;
  if (m.sleeping)
    return RequestedMode::Sleep;
  return m.baseMode;
}

Desired update(Mapper &m, const PadState &pad, uint32_t nowMs)
{
  Desired d = {};

  if (!pad.connected)
  {
    m.wasConnected = false;
  }
  else
  {
    if (!m.wasConnected)
    {
      // A fresh connection starts in Stand; buttons already held do not count
      // as presses.
      m.wasConnected = true;
      m.baseMode = RequestedMode::Stand;
      m.sleeping = false;
      m.calibrating = false;
      m.prevButtons = pad.buttons;
      m.prevDpad = pad.dpad;
      m.prevMisc = pad.misc;
      m.calibrateHeld = m.clearHeld = false;
      m.calibrateFired = m.clearFired = false;
    }

    uint16_t pressed = pad.buttons & ~m.prevButtons;
    uint8_t dpadPressed = pad.dpad & ~m.prevDpad;
    uint8_t miscPressed = pad.misc & ~m.prevMisc;
    m.prevButtons = pad.buttons;
    m.prevDpad = pad.dpad;
    m.prevMisc = pad.misc;

    // Calibrate: miscBack + miscHome held. While miscBack is held, miscHome
    // is part of the combination and does not toggle Stand/Walk.
    bool backHeld = (pad.misc & MISC_BACK) != 0;
    bool calibrateDown = backHeld && (pad.misc & MISC_HOME);
    if (comboFired(calibrateDown, m.calibrateHeld, m.calibrateFired, m.calibrateSinceMs, nowMs))
    {
      m.calibrating = !m.calibrating;
      m.sleeping = false;
      m.baseMode = RequestedMode::Stand; // leaving Calibrate returns to Stand
      m.modeRequestCounter++;
    }
    else if ((miscPressed & MISC_HOME) && !backHeld)
    {
      if (m.calibrating)
        m.calibrating = false; // leaving Calibrate returns to Stand (set on entry)
      else
        m.baseMode = m.baseMode == RequestedMode::Walk ? RequestedMode::Stand : RequestedMode::Walk;
      m.modeRequestCounter++;
    }

    // The gesture layer: while L2 is held, a, b, x, y and R2 request
    // gestures instead of their own functions.
    bool layer = (pad.buttons & BUTTON_L2) != 0;
    if (layer)
    {
      static const struct
      {
        uint16_t button;
        uint8_t gesture;
      } GESTURE_BUTTONS[] = {{BUTTON_A, protocol::GESTURE_SAY_HI}, {BUTTON_B, protocol::GESTURE_WIGGLE},
                             {BUTTON_X, protocol::GESTURE_DANCE},  {BUTTON_Y, protocol::GESTURE_ATTACK},
                             {BUTTON_R2, protocol::GESTURE_BALL}};
      for (const auto &g : GESTURE_BUTTONS)
        if (pressed & g.button)
        {
          m.gesture = g.gesture;
          m.gestureRequestCounter++;
        }
      pressed &= (uint16_t)~(BUTTON_A | BUTTON_B | BUTTON_X | BUTTON_Y | BUTTON_R2);
    }

    if (pressed & BUTTON_Y)
    {
      if (m.calibrating)
      {
        // Sleep from Calibrate; leaving Sleep then returns to Stand.
        m.calibrating = false;
        m.sleeping = true;
      }
      else
      {
        m.sleeping = !m.sleeping;
      }
      m.modeRequestCounter++;
    }

    if (pressed & BUTTON_R1)
      m.gait = (uint8_t)((m.gait + 1) % gait::GAIT_COUNT);
    if (pressed & BUTTON_L1)
      m.gait = (uint8_t)((m.gait + gait::GAIT_COUNT - 1) % gait::GAIT_COUNT);
    if (dpadPressed & DPAD_UP)
      m.height = stepDial(m.height, DIAL_STEP);
    if (dpadPressed & DPAD_DOWN)
      m.height = stepDial(m.height, -DIAL_STEP);
    if (dpadPressed & DPAD_RIGHT)
      m.speed = stepDial(m.speed, DIAL_STEP);
    if (dpadPressed & DPAD_LEFT)
      m.speed = stepDial(m.speed, -DIAL_STEP);
    if (pressed & BUTTON_X)
      m.stride = m.stride == StrideMode::Dynamic ? StrideMode::Fixed : StrideMode::Dynamic;
    if (pressed & BUTTON_B)
      m.stickMode = m.stickMode == StickMode::Steer ? StickMode::Crab : StickMode::Steer;

    bool clearDown = (pad.buttons & BUTTON_THUMB_L) && (pad.buttons & BUTTON_THUMB_R);
    if (comboFired(clearDown, m.clearHeld, m.clearFired, m.clearSinceMs, nowMs))
      m.clearCounter++;

    int8_t leftX, leftY, rightX, unusedY;
    stickToVector(pad.axisX, -pad.axisY, leftX, leftY);
    stickToVector(pad.axisRX, 0, rightX, unusedY);
    // Positive rotate turns the body anticlockwise, so a stick pushed right
    // (positive) turns it clockwise with a negative rotate.
    int rotate = -rightX;
    if (m.stickMode == StickMode::Steer)
    {
      // Walk on Y, steer on X. Reversing mirrors the steering, as a car's;
      // a slightly backward stick, within the threshold, neither reverses
      // nor mirrors, so a stick held sideways turns on the spot steadily.
      bool reversing = leftY < -REVERSE_THRESHOLD;
      d.translateX = 0;
      d.translateY = (leftY < 0 && !reversing) ? 0 : leftY;
      rotate += reversing ? leftX : -leftX;
    }
    else
    {
      d.translateX = leftX;
      d.translateY = leftY;
    }
    d.rotate = (int8_t)(rotate > 100 ? 100 : (rotate < -100 ? -100 : rotate));
  }

  d.controllerConnected = pad.connected;
  d.mode = pad.connected ? requestedMode(m) : RequestedMode::Sleep;
  d.height = m.height;
  d.speed = m.speed;
  d.gait = m.gait;
  d.stride = m.stride;
  d.stickMode = m.stickMode;
  d.clearCounter = m.clearCounter;
  d.modeRequestCounter = m.modeRequestCounter;
  d.gesture = m.gesture;
  d.gestureRequestCounter = m.gestureRequestCounter;
  return d;
}

protocol::Command toCommand(const Desired &d, bool emergencyStop)
{
  protocol::Command c = {};
  c.mode = (uint8_t)d.mode;
  c.translateX = d.translateX;
  c.translateY = d.translateY;
  c.rotate = d.rotate;
  c.height = d.height;
  c.speed = d.speed;
  c.gait = d.gait;
  c.stride = (uint8_t)d.stride;
  c.flags = (d.controllerConnected ? protocol::FLAG_CONTROLLER_CONNECTED : 0) |
            (emergencyStop ? protocol::FLAG_EMERGENCY_STOP : 0);
  c.clearCounter = d.clearCounter;
  c.modeRequestCounter = d.modeRequestCounter;
  c.gesture = d.gesture;
  c.gestureRequestCounter = d.gestureRequestCounter;
  return c;
}

} // namespace gamepad_mapping
