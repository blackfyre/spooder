// Gamepad bridge: a Bluepad32 gamepad becomes command frames to the robot,
// and the robot's telemetry becomes the status screen.
//
// Bluepad32 (BTstack) runs on core 0. setup()/loop() run on core 1, with the
// link task above loop() so screen redraws never delay command frames.
#include "sdkconfig.h"

#include <Arduino.h>
#include <Bluepad32.h>

#include "display.h"
#include "gamepad_mapping/gamepad_mapping.h"
#include "link.h"

// The mapping's button bits are Bluepad32's.
static_assert(gamepad_mapping::BUTTON_A == BUTTON_A && gamepad_mapping::BUTTON_B == BUTTON_B &&
                  gamepad_mapping::BUTTON_X == BUTTON_X && gamepad_mapping::BUTTON_Y == BUTTON_Y &&
                  gamepad_mapping::BUTTON_L1 == BUTTON_SHOULDER_L &&
                  gamepad_mapping::BUTTON_R1 == BUTTON_SHOULDER_R &&
                  gamepad_mapping::BUTTON_THUMB_L == BUTTON_THUMB_L &&
                  gamepad_mapping::BUTTON_THUMB_R == BUTTON_THUMB_R,
              "button bits must match Bluepad32");
static_assert(gamepad_mapping::DPAD_UP == DPAD_UP && gamepad_mapping::DPAD_DOWN == DPAD_DOWN &&
                  gamepad_mapping::DPAD_RIGHT == DPAD_RIGHT && gamepad_mapping::DPAD_LEFT == DPAD_LEFT,
              "d-pad bits must match Bluepad32");
static_assert(gamepad_mapping::MISC_HOME == MISC_BUTTON_SYSTEM && gamepad_mapping::MISC_BACK == MISC_BUTTON_SELECT,
              "misc bits must match Bluepad32");

// One controller slot; a second controller is refused.
static ControllerPtr controller = nullptr;
static gamepad_mapping::Mapper mapper;

static void onConnected(ControllerPtr ctl)
{
  if (controller != nullptr)
  {
    Console.printf("second controller refused: %s\n", ctl->getModelName().c_str());
    ctl->disconnect();
    return;
  }
  controller = ctl;
  Console.printf("controller connected: %s\n", ctl->getModelName().c_str());
}

static void onDisconnected(ControllerPtr ctl)
{
  if (ctl != controller)
    return;
  controller = nullptr;
  Console.println("controller disconnected");
}

static gamepad_mapping::PadState readPad()
{
  gamepad_mapping::PadState pad = {};
  if (controller == nullptr || !controller->isConnected() || !controller->isGamepad())
    return pad; // not connected
  pad.connected = true;
  pad.axisX = controller->axisX();
  pad.axisY = controller->axisY();
  pad.axisRX = controller->axisRX();
  pad.axisRY = controller->axisRY();
  pad.buttons = controller->buttons();
  pad.dpad = controller->dpad();
  pad.misc = controller->miscButtons();
  return pad;
}

void setup()
{
  Console.printf("spooder bridge, Bluepad32 %s\n", BP32.firmwareVersion());
  display::setup(); // logo first
  robot_link::start();
  display::bootMessage("Robot link: UART1 26/27");
  BP32.setup(&onConnected, &onDisconnected);
  // No virtual devices (mouse emulation from touchpads).
  BP32.enableVirtualDevice(false);
  display::bootMessage("Bluetooth: Bluepad32");
  display::bootMessage("Ready!");
  display::bootMessage("Waiting on gamepad...");
}

void loop()
{
  BP32.update();
  uint32_t now = millis();

  gamepad_mapping::PadState pad = readPad();
  gamepad_mapping::Desired desired = gamepad_mapping::update(mapper, pad, now);
  robot_link::setDesired(desired);

  display::update(now, pad.connected, desired, robot_link::status());

  // Yield so the idle task runs (watchdog) between gamepad updates.
  delay(1);
}
