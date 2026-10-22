#ifndef BRIDGE_DISPLAY_H
#define BRIDGE_DISPLAY_H

// Status screen on the T-Display's ST7789 (135×240, used landscape).
// At power-up: the Spooder logo, then a start-up log, then the status screen
// (an active emergency stop skips straight to it). The status screen has
// three pages (core/status/status_screen) in the Blackfyre dark palette;
// update() redraws at most every REFRESH_MS, and only the regions (header,
// body, status strip, page marker) whose content changed, each drawn off
// screen in one reusable sprite.
// In setup mode there is no logo: setupStart() draws the joining QR code on
// the right, and setupUpdate() keeps the text on the left current.
#include <stdint.h>
#include "gamepad_mapping/gamepad_mapping.h"
#include "link.h"
#include "status/page_button.h"

namespace display
{

constexpr uint32_t REFRESH_MS = 250;
constexpr uint32_t LOGO_MS = 1500;
constexpr uint32_t BOOT_LOG_MS = 1500; // after the logo

/**
 * @brief Starts the panel and shows the logo.
 */
void setup();

/**
 * @brief Adds a line to the start-up log (shown after the logo).
 */
void bootMessage(const char *text, bool ok = true);

/**
 * @brief Records the power history and link rates, and redraws @p page when
 * due. Call on every loop() pass. @p gamepadBattery is Bluepad32's level
 * (0 = not reported).
 */
void update(uint32_t nowMs, bool gamepadConnected, uint8_t gamepadBattery, page_button::Page page,
            const gamepad_mapping::Desired &desired, const robot_link::Status &linkStatus);

/**
 * @brief True once the status screen is shown (the start-up sequence is over):
 * from then on the left button switches pages.
 */
bool statusShown();

/**
 * @brief True while the boot logo is shown (the setup-mode button counts
 * only then).
 */
bool showingLogo();

/**
 * @brief Keeps the logo up while @p hold (the setup-mode button is down), so
 * a hold that starts late in the logo can still complete.
 */
void holdLogo(bool hold);

// --- setup mode ---------------------------------------------------------------

/**
 * @brief Starts the panel for setup mode (no logo) and draws the joining QR
 * code for @p joinCode on the right.
 */
void setupStart(const char *joinCode);

/**
 * @brief Redraws the setup screen's text (at most every REFRESH_MS, changed
 * lines only).
 */
void setupUpdate(uint32_t nowMs, const char *ssid, const char *password, bool portalConnected,
                 const robot_link::Status &linkStatus);

} // namespace display

#endif // BRIDGE_DISPLAY_H
