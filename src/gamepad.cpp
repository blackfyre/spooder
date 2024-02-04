#include "gamepad.h"

ControllerPtr myControllers[BP32_MAX_CONTROLLERS];

/**
 * @brief Initializes the gamepad.
 * 
 * This function sets up the necessary configurations and parameters for the gamepad.
 * It should be called once at the beginning of the program.
 */
void setupGamepad() {
  String fv = BP32.firmwareVersion();
  Serial.print("Firmware version installed: ");
  Serial.println(fv);

  // To get the BD Address (MAC address) call:
  const uint8_t* addr = BP32.localBdAddress();
  Serial.print("BD Address: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(addr[i], HEX);
    if (i < 5)
      Serial.print(":");
    else
      Serial.println();
  }

  // BP32.pinMode(27, OUTPUT);
  // BP32.digitalWrite(27, 0);

  // This call is mandatory. It setups Bluepad32 and creates the callbacks.
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();
}



/**
 * @brief Callback function called when a controller is connected.
 * 
 * @param ctl The connected controller object.
 */
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.print("CALLBACK: Controller is connected, index=");
      Serial.println(i);
      addToLogBuffer("Gamepad found!");
      myControllers[i] = ctl;
      foundEmptySlot = true;

      BP32.digitalWrite(LED_B, LED_ON);

      // Optional, once the gamepad is connected, request further info about the
      // gamepad.
      ControllerProperties properties = ctl->getProperties();
      char buf[80];
      sprintf(buf,
              "BTAddr: %02x:%02x:%02x:%02x:%02x:%02x, VID/PID: %04x:%04x, "
              "flags: 0x%02x",
              properties.btaddr[0], properties.btaddr[1], properties.btaddr[2],
              properties.btaddr[3], properties.btaddr[4], properties.btaddr[5],
              properties.vendor_id, properties.product_id, properties.flags);
      Serial.println(buf);
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println(
      "CALLBACK: Controller connected, but could not found empty slot");
  }
}

/**
 * @brief Callback function called when a controller is disconnected.
 * 
 * @param ctl The disconnected controller.
 */
void onDisconnectedController(ControllerPtr ctl) {
  bool foundGamepad = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      BP32.digitalWrite(LED_B, LED_OFF);
      Serial.print("CALLBACK: Controller is disconnected from index=");
      addToLogBuffer("Gamepad lost!");
      Serial.println(i);
      myControllers[i] = nullptr;
      foundGamepad = true;
      break;
    }
  }

  if (!foundGamepad) {
    Serial.println(
      "CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

/**
 * Processes the gamepad input.
 * 
 * @param gamepad The gamepad controller object.
 */
void processGamepad(ControllerPtr gamepad) {
  // There are different ways to query whether a button is pressed.
  // By query each button individually:
  //  a(), b(), x(), y(), l1(), etc...
  BP32.digitalWrite(LED_R, LED_OFF);
  BP32.digitalWrite(LED_G, LED_OFF);


  if (gamepad->a()) {  // it's b actually on 8-bitdo
    BP32.digitalWrite(LED_R, LED_ON);
    addToLogBuffer("B");
  }

  if (gamepad->b()) {
    BP32.digitalWrite(LED_R, LED_ON);
    addToLogBuffer("A");
  }

  if (gamepad->x()) {
    BP32.digitalWrite(LED_G, LED_ON);
    addToLogBuffer("Y");
  }

  if (gamepad->y()) {
    BP32.digitalWrite(LED_G, LED_ON);
    addToLogBuffer("X");
  }

  // Another way to query the buttons, is by calling buttons(), or
  // miscButtons() which return a bitmask.
  // Some gamepads also have DPAD, axis and more.
  // char buf[256];
  // snprintf(buf, sizeof(buf) - 1,
  //          "idx=%d, dpad: 0x%02x, buttons: 0x%04x, "
  //          "axis L: %4li, %4li, axis R: %4li, %4li, "
  //          "brake: %4ld, throttle: %4li, misc: 0x%02x",
  //          gamepad->index(),       // Gamepad Index
  //          gamepad->dpad(),        // DPAD
  //          gamepad->buttons(),     // bitmask of pressed buttons
  //          gamepad->axisX(),       // (-511 - 512) left X Axis
  //          gamepad->axisY(),       // (-511 - 512) left Y axis
  //          gamepad->axisRX(),      // (-511 - 512) right X axis
  //          gamepad->axisRY(),      // (-511 - 512) right Y axis
  //          gamepad->brake(),       // (0 - 1023): brake button
  //          gamepad->throttle(),    // (0 - 1023): throttle (AKA gas) button
  //          gamepad->miscButtons()  // bitmak of pressed "misc" buttons
  // );
  // Serial.println(buf);

  // You can query the axis and other properties as well. See
  // Controller.h For all the available functions.
}

/**
 * @brief Function that handles the gamepad loop.
 * 
 * This function is responsible for continuously processing the gamepad input and updating the game state accordingly.
 * It should be called in the main loop of the program.
 */
void loopGamepad() {
  // This call fetches all the controller info from the NINA (ESP32) module.
  // Just call this function in your main loop.
  // The controllers pointer (the ones received in the callbacks) gets updated
  // automatically.
  BP32.update();

  // It is safe to always do this before using the controller API.
  // This guarantees that the controller is valid and connected.
  for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
    ControllerPtr myController = myControllers[i];

    if (myController && myController->isConnected()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      }
    }
  }
}