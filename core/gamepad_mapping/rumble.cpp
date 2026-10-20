#include "rumble.h"

#include "protection/protection.h"

namespace rumble
{

Pattern pattern(Event e)
{
  switch (e)
  {
  case Event::FaultLatched:
    return {600, 128, 255, 4};
  case Event::EstopOn:
    return {250, 0, 255, 3};
  case Event::LinkLost:
    return {400, 200, 120, 2};
  case Event::GestureRefused:
    return {160, 160, 0, 1};
  case Event::EstopOff:
  case Event::LinkRegained:
    return {90, 120, 0, 1};
  case Event::None:
    break;
  }
  return {0, 0, 0, 0};
}

static void detected(State &s, Event e, uint32_t nowMs)
{
  s.waiting[(int)e] = true;
  s.waitingAtMs[(int)e] = nowMs;
}

Request update(State &s, const Inputs &in)
{
  Request none = {false, Event::None, 0, 0, 0};
  if (!in.gamepadConnected)
  {
    s = State(); // a new connection starts afresh
    return none;
  }
  if (!s.started)
  {
    // A fresh connection: what is already latched or switched on is known.
    s.started = true;
    s.wasFresh = in.telemetryFresh;
    s.faults = in.telemetryFresh ? in.faults : 0;
    s.emergencyStop = in.emergencyStop;
    s.gestureCounter = in.gestureRequestCounter;
    return none;
  }

  // --- what happened -------------------------------------------------------
  if (in.emergencyStop != s.emergencyStop)
    detected(s, in.emergencyStop ? Event::EstopOn : Event::EstopOff, in.nowMs);
  s.emergencyStop = in.emergencyStop;

  if (in.telemetryFresh && !s.wasFresh)
  {
    detected(s, Event::LinkRegained, in.nowMs);
    s.faults = in.faults; // the baseline after a gap
  }
  else if (!in.telemetryFresh && s.wasFresh)
  {
    detected(s, Event::LinkLost, in.nowMs);
  }
  else if (in.telemetryFresh)
  {
    uint16_t latched = in.faults & (uint16_t)~s.faults & (uint16_t)~protection::EMERGENCY_STOP;
    if (latched)
      detected(s, Event::FaultLatched, in.nowMs);
    s.faults = in.faults;
  }
  s.wasFresh = in.telemetryFresh;

  if (in.gestureRequestCounter != s.gestureCounter)
  {
    s.gestureCounter = in.gestureRequestCounter;
    s.gesturePending = true;
    s.pendingGesture = in.gesture;
    // The same gesture already playing does not count as acting on this one.
    s.sawNotPlaying = in.playingGesture != in.gesture;
    s.gestureAtMs = in.nowMs;
  }
  if (s.gesturePending)
  {
    if (!in.telemetryFresh)
      s.gesturePending = false; // the link loss says it all
    else if (in.playingGesture != s.pendingGesture)
      s.sawNotPlaying = true;
    if (s.gesturePending && s.sawNotPlaying && in.playingGesture == s.pendingGesture)
      s.gesturePending = false; // the robot plays it
    else if (s.gesturePending && in.nowMs - s.gestureAtMs >= GESTURE_REFUSED_MS)
    {
      s.gesturePending = false;
      detected(s, Event::GestureRefused, in.nowMs);
    }
  }

  // --- what plays ----------------------------------------------------------
  if (s.playing && in.nowMs - s.playStartMs >= s.playDurationMs)
    s.playing = false;
  for (int e = 1; e < EVENT_COUNT; e++)
    if (s.played[e] && in.nowMs - s.lastAtMs[e] >= REPEAT_MS)
      s.played[e] = false; // the cooldown is over (and stays over across a timer wrap)
  int best = 0;
  for (int e = 1; e < EVENT_COUNT; e++)
  {
    if (!s.waiting[e])
      continue;
    // Too old, or too soon after the last time it played: dropped.
    if (in.nowMs - s.waitingAtMs[e] >= REPEAT_MS || (s.played[e] && s.waitingAtMs[e] - s.lastAtMs[e] < REPEAT_MS))
    {
      s.waiting[e] = false;
      continue;
    }
    if (best == 0 || pattern((Event)e).priority > pattern((Event)best).priority)
      best = e;
  }
  if (best == 0)
    return none;
  Pattern p = pattern((Event)best);
  if (s.playing && s.playPriority > p.priority)
    return none; // it waits for the more important rumble
  s.waiting[best] = false;
  s.played[best] = true;
  s.lastAtMs[best] = in.nowMs;
  s.playing = true;
  s.playStartMs = in.nowMs;
  s.playDurationMs = p.durationMs;
  s.playPriority = p.priority;
  return {true, (Event)best, p.durationMs, p.weak, p.strong};
}

} // namespace rumble
