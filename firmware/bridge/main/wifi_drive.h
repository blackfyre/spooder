#ifndef BRIDGE_WIFI_DRIVE_H
#define BRIDGE_WIFI_DRIVE_H

// The bridge's drive mode switch (GPIO32 to ground, internal pull-up) and
// its WiFi drive boot path: a WPA2 access point through which one handheld
// drives the robot with link-protocol command frames over UDP, the robot's
// frames relayed back to it. Like setup mode, it is a boot path of its own,
// chosen once in app_main (core/wifi_drive/drive_mode), and it never starts
// Bluetooth: WiFi and Bluetooth never share the radio.
//
// Tasks in WiFi drive:
// - robot_link, priority 5, core 1: as in Bluetooth drive, but commands
//   come from the handheld controller, judged every period, and the robot's
//   frames are queued for the relay.
// - udp, priority 2, core 1: reads the handheld's datagrams (at most a
//   window's budget per 20 ms window), stamping each on receipt, and sends
//   the relayed frames.
// - wifi-drive, priority 1, core 1: the screen and the buttons, as loop()
//   is in Bluetooth drive.
// - WiFi and lwIP: core 0.
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

enum bridge_boot_mode
{
  BRIDGE_BOOT_BLUETOOTH,
  BRIDGE_BOOT_WIFI,
  BRIDGE_BOOT_SETUP,
};

/**
 * @brief Called in app_main before either radio starts: configures the
 * switch pin with its pull-up, lets it settle, reads it and decides the
 * boot mode (drive_mode::select). Setup mode ignores the switch.
 */
enum bridge_boot_mode bridge_boot_mode_select(bool setupRequested);

/**
 * @brief The switch now: true when closed to ground (WiFi drive). Reads
 * the pin through ESP-IDF, so it needs no Arduino pin setup.
 */
bool drive_switch_closed(void);

/**
 * @brief Starts Arduino, the display, the robot link, the access point, the
 * UDP task and the WiFi drive task. Returns once they run; never starts
 * Bluetooth.
 */
void wifi_drive_start(void);

#ifdef __cplusplus
}
#endif

#endif // BRIDGE_WIFI_DRIVE_H
