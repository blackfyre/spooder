// SPDX-License-Identifier: Apache-2.0
// Copyright 2019 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "sdkconfig.h"

#include <stddef.h>
#include <stdlib.h>

// BTstack related
#include <btstack_port_esp32.h>
#include <btstack_run_loop.h>
#include <btstack_stdio_esp32.h>

// Bluepad32 related
#include <arduino_platform.h>
#include <uni.h>

// Spooder: the setup-mode and WiFi drive boot paths (setup_mode.cpp,
// wifi_drive.cpp).
#include "setup_mode.h"
#include "wifi_drive.h"

//
// Autostart
//
#if CONFIG_AUTOSTART_ARDUINO
void initBluepad32()
{
#else
int app_main(void)
{
#endif // !CONFIG_AUTOSTART_ARDUINO
  // Spooder: the boot mode is decided once, before either radio starts.
  // Setup mode and WiFi drive skip BTstack and Bluepad32 entirely, so
  // Bluetooth never starts; the bridge leaves either only by restarting.
  // With no default case, a new boot mode fails the build (-Wswitch) until
  // it has a path here. Only Bluetooth drive goes on to BTstack and
  // Bluepad32 below; a value no case names (memory corruption) aborts, so the
  // bridge restarts instead of falling into a radio path.
  enum bridge_boot_mode boot = bridge_boot_mode_select(setup_mode_requested());
  switch (boot)
  {
  case BRIDGE_BOOT_SETUP:
    setup_mode_start();
#if !CONFIG_AUTOSTART_ARDUINO
    return 0;
#else
    return;
#endif
  case BRIDGE_BOOT_WIFI:
    wifi_drive_start();
#if !CONFIG_AUTOSTART_ARDUINO
    return 0;
#else
    return;
#endif
  case BRIDGE_BOOT_BLUETOOTH:
    goto bluetooth_drive;
  }
  abort();

bluetooth_drive:

  // hci_dump_open(NULL, HCI_DUMP_STDOUT);

// Don't use BTstack buffered UART. It conflicts with the console.
#ifndef CONFIG_ESP_CONSOLE_UART_NONE
#ifndef CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
  btstack_stdio_init();
#endif // CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
#endif // CONFIG_ESP_CONSOLE_UART_NONE

  // Configure BTstack for ESP32 VHCI Controller
  btstack_init();

  // hci_dump_init(hci_dump_embedded_stdout_get_instance());

  // Must be called before uni_init()
  uni_platform_set_custom(get_arduino_platform());

  // Init Bluepad32.
  uni_init(0 /* argc */, NULL /* argv */);

  // Does not return.
  btstack_run_loop_execute();
#if !CONFIG_AUTOSTART_ARDUINO
  return 0;
#endif // !CONFIG_AUTOSTART_ARDUINO
}
