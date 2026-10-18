#ifndef GAMEPAD_MAPPING_H
#define GAMEPAD_MAPPING_H

// Turns gamepad input into the desired movement state the bridge sends to the
// robot. Holds the toggles, dials and counters across frames, detects presses
// and timed holds, and applies the stick dead zone.
// Hardware-free: the bridge fills PadState from Bluepad32; host tests fill it
// directly.
#include <stdint.h>
#include "movement/movement_command.h"
#include "protocol/protocol.h"

namespace gamepad_mapping
{

// PadState::buttons bits (Bluepad32 names).
constexpr uint16_t BUTTON_A = 1u << 0;
constexpr uint16_t BUTTON_B = 1u << 1;
constexpr uint16_t BUTTON_X = 1u << 2;
constexpr uint16_t BUTTON_Y = 1u << 3;
constexpr uint16_t BUTTON_L1 = 1u << 4;
constexpr uint16_t BUTTON_R1 = 1u << 5;
constexpr uint16_t BUTTON_THUMB_L = 1u << 8;
constexpr uint16_t BUTTON_THUMB_R = 1u << 9;
// PadState::dpad bits.
constexpr uint8_t DPAD_UP = 1u << 0;
constexpr uint8_t DPAD_DOWN = 1u << 1;
constexpr uint8_t DPAD_RIGHT = 1u << 2;
constexpr uint8_t DPAD_LEFT = 1u << 3;
// PadState::misc bits.
constexpr uint8_t MISC_HOME = 1u << 0;
constexpr uint8_t MISC_BACK = 1u << 1;

constexpr int AXIS_MAX = 512;           // Bluepad32 axes span -511..512
constexpr float DEAD_ZONE = 0.08f;      // radial, fraction of full scale
constexpr uint32_t COMBO_HOLD_MS = 1000;
constexpr int DIAL_STEP = 10;

struct PadState
{
  bool connected;
  int32_t axisX, axisY;   // left stick; Bluepad32 Y is negative for up
  int32_t axisRX, axisRY; // right stick
  uint16_t buttons;       // BUTTON_*
  uint8_t dpad;           // DPAD_*
  uint8_t misc;           // MISC_*
};

struct Desired
{
  RequestedMode mode;
  int8_t translateX, translateY; // -100..100, + Y = forward
  int8_t rotate;                 // -100..100
  uint8_t height, speed;         // 0..100
  uint8_t gait;
  StrideMode stride;
  bool controllerConnected;
  uint8_t clearCounter;
  uint8_t modeRequestCounter;
};

struct Mapper
{
  // Settings kept across disconnects.
  uint8_t height = 0;
  uint8_t speed = 44;
  uint8_t gait = 0;
  StrideMode stride = StrideMode::Dynamic;
  uint8_t clearCounter = 0;
  uint8_t modeRequestCounter = 0;

  // Mode request: base mode plus the Sleep and Calibrate toggles.
  RequestedMode baseMode = RequestedMode::Stand;
  bool sleeping = false;
  bool calibrating = false;

  // Input history.
  bool wasConnected = false;
  uint16_t prevButtons = 0;
  uint8_t prevDpad = 0;
  uint8_t prevMisc = 0;
  bool calibrateHeld = false;
  bool calibrateFired = false;
  uint32_t calibrateSinceMs = 0;
  bool clearHeld = false;
  bool clearFired = false;
  uint32_t clearSinceMs = 0;
};

/**
 * @brief Advances the mapping by one pad sample taken at @p nowMs.
 */
Desired update(Mapper &m, const PadState &pad, uint32_t nowMs);

/**
 * @brief Converts a stick pair (Bluepad32 units, Y already pointing up) to
 * -100..100 per axis with the radial dead zone and magnitude clamp applied.
 */
void stickToVector(int32_t x, int32_t y, int8_t &outX, int8_t &outY);

/**
 * @brief The command frame payload for @p d.
 */
protocol::Command toCommand(const Desired &d, bool emergencyStop);

} // namespace gamepad_mapping

#endif // GAMEPAD_MAPPING_H
