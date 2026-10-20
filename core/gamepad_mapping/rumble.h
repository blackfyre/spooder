#ifndef RUMBLE_H
#define RUMBLE_H

// Gamepad rumble feedback on the bridge: turns what the bridge knows (the
// robot's telemetry, its own e-stop toggle, its gesture requests) into
// rumble requests, one pattern per event. Hardware-free: the bridge plays
// the requests through Bluepad32; host tests step it directly.
#include <stdint.h>

namespace rumble
{

enum class Event : uint8_t
{
  None,
  FaultLatched,   // a protection fault newly latched (the e-stop's excepted)
  EstopOn,        // the bridge's own e-stop switched on
  LinkLost,       // telemetry stopped while a gamepad is connected
  GestureRefused, // a gesture request not playing GESTURE_REFUSED_MS later
  EstopOff,
  LinkRegained,
};
constexpr int EVENT_COUNT = 7;

struct Pattern
{
  uint16_t durationMs;
  uint8_t weak, strong; // motor magnitudes
  uint8_t priority;     // a higher one replaces a lower one in progress
};

/**
 * @brief The pattern for @p e (design D2).
 */
Pattern pattern(Event e);

constexpr uint32_t GESTURE_REFUSED_MS = 600;
constexpr uint32_t REPEAT_MS = 1000; // the same event never rumbles again sooner

struct Inputs
{
  uint32_t nowMs;
  bool gamepadConnected;
  bool telemetryFresh;
  uint16_t faults;               // the robot's latched faults (protection::Fault bits)
  bool emergencyStop;            // the bridge's own toggle
  uint8_t gestureRequestCounter; // the bridge's latest gesture request
  uint8_t gesture;
  uint8_t playingGesture;        // what telemetry reports playing
};

struct Request
{
  bool play;
  Event event;
  uint16_t durationMs;
  uint8_t weak, strong;
};

struct State
{
  bool started = false; // a baseline is taken (a gamepad is connected)
  bool wasFresh = false;
  uint16_t faults = 0;
  bool emergencyStop = false;
  uint8_t gestureCounter = 0;
  bool gesturePending = false;
  uint8_t pendingGesture = 0;
  bool sawNotPlaying = false; // the pending gesture was seen not playing since the request
  uint32_t gestureAtMs = 0;
  // The rumble playing, by its start and duration (wrap-safe).
  bool playing = false;
  uint32_t playStartMs = 0;
  uint16_t playDurationMs = 0;
  uint8_t playPriority = 0;
  // Events waiting to play, with when they happened; dropped after REPEAT_MS.
  bool waiting[EVENT_COUNT] = {};
  uint32_t waitingAtMs[EVENT_COUNT] = {};
  // When each event last played, for the repeat limit.
  bool played[EVENT_COUNT] = {};
  uint32_t lastAtMs[EVENT_COUNT] = {};
};

/**
 * @brief Steps the detector; returns the rumble to play now, if any. When a
 * gamepad connects, the current faults and e-stop, and when telemetry becomes
 * fresh again,
 * the faults are the baseline: nothing rumbles for them. Events that happen
 * together, or during a more important rumble, wait (for at most REPEAT_MS)
 * and play strongest first. A disconnect forgets everything.
 */
Request update(State &s, const Inputs &in);

} // namespace rumble

#endif // RUMBLE_H
