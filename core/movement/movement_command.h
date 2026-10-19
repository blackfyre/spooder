#ifndef MOVEMENT_COMMAND_H
#define MOVEMENT_COMMAND_H

// The request a controller hands to movement. Movement decides whether and
// when to act on it; a command never sets movement state directly.
// Hardware-free: compiled for the board and for host tests.
#include <stdint.h>
#include "vectors.h"

enum class RequestedMode : uint8_t
{
  Stand,
  Walk,
  Sleep,
  Calibrate,
};

enum class StrideMode : uint8_t
{
  Dynamic, // stride and rotation length scale with input magnitude
  Fixed,   // constant stride and rotation length; input sets direction only
};

struct MovementCommand
{
  RequestedMode mode = RequestedMode::Stand;
  Vector2 translate;        // magnitude 0..100
  float rotate = 0;         // -100..100
  float height = 0;         // body height dial, 0..100; higher raises the body
  float speed = 44;         // speed dial, 0..100
  int gait = 0;             // index into gait::Gait; unknown values are ignored
  StrideMode stride = StrideMode::Dynamic;
  uint32_t issuedAtMs = 0;  // millis() when the controller produced the command
  // Gesture requests: each change of gestureSeq requests `gesture` (a
  // gesture::Gesture id) once. The body takes the first value it sees as the
  // baseline, so a restart never replays an old request.
  uint8_t gesture = 0;
  uint32_t gestureSeq = 0;
  // Advances with every explicit mode request (the source in control's), so
  // movement can tell a new Stand request from a continuing one.
  uint32_t modeRequestSeq = 0;
};

#endif // MOVEMENT_COMMAND_H
