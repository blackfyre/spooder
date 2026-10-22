#ifndef STATUS_SCREEN_H
#define STATUS_SCREEN_H

// The bridge's status screen: what each page shows (the drive, power and
// link pages, and the status strip on all of them), and the setup-mode
// screen's text lines.
// Hardware-free: the bridge lays the content out; host tests check it.
#include <stdint.h>
#include "gamepad_mapping/gamepad_mapping.h"
#include "protocol/protocol.h"
#include "status/page_button.h"
#include "status/peak_hold.h"
#include "status/power_history.h"

namespace status_screen
{

constexpr int LINE_COUNT = 8;
constexpr int LINE_CHARS = 28; // a start-up log or setup line

enum class Tone : uint8_t
{
  Normal,
  Good,
  Warn,
  Bad,
  Muted, // labels and absent values
};

struct Line
{
  char text[LINE_CHARS + 1];
  Tone tone;
};

constexpr int TEXT_CHARS = 28; // one field of a page
constexpr int STRIP_LINE_CHARS = 37; // the status strip: 222 px at its 6 px font
constexpr int STRIP_LINES = 2;

struct Text
{
  char text[TEXT_CHARS + 1];
  Tone tone;
};

struct Bar
{
  bool present;       // false: no value to show (link lost)
  float fraction;     // 0..1 of the bar's fixed scale
  bool held;          // a held extreme to mark
  float heldFraction; // where, on the same scale
};

// Fixed bar scales.
constexpr uint16_t BAR_MIN_MV = 4500, BAR_MAX_MV = 7500;
constexpr uint16_t BAR_MAX_MA = 15000; // the board's measurable range
// Power page traces: the current from 0 to at least this, the voltage over at
// least this span.
constexpr uint16_t TRACE_MIN_MA = 2000;
constexpr uint16_t TRACE_MIN_SPAN_MV = 500;

// What a gamepad battery of Bluepad32's 1 (empty) to 255 (full) shows as;
// 0 is not reported.
int batteryPercent(uint8_t battery);

// Frame rates over about a second, from the counters' differences.
struct Rates
{
  bool robotValid; // the robot's counts need fresh telemetry at both ends
  uint16_t robotAccepted, robotRejected;
  bool bridgeValid;
  uint16_t bridgeAccepted, bridgeRejected;
};

struct RateMeter
{
  bool started = false;
  uint32_t atMs = 0;
  bool robotKnown = false;
  uint16_t robotAccepted = 0, robotRejected = 0;
  uint32_t bridgeAccepted = 0, bridgeRejected = 0;
  Rates rates = {};
};

constexpr uint32_t RATE_PERIOD_MS = 1000;

/**
 * @brief Updates the rates once RATE_PERIOD_MS has passed since the last
 * update: the robot's decoder counts on the bridge link (from telemetry,
 * 16-bit, wrapping) and the bridge's own decoder counts.
 */
void updateRates(RateMeter &m, uint32_t nowMs, bool telemetryFresh, const protocol::Telemetry &t,
                 uint32_t bridgeAccepted, uint32_t bridgeRejected);

struct Inputs
{
  bool gamepadConnected;
  uint8_t gamepadBattery; // Bluepad32's level, 0 = not reported
  bool emergencyStop;     // the bridge's own e-stop toggle
  gamepad_mapping::Desired desired;
  bool telemetryFresh;    // robot values are current only while fresh
  protocol::Telemetry telemetry;
  uint32_t telemetryAgeMs;
  Rates rates;
  const power_history::History *history; // may be null: no history yet
  peak_hold::Hold currentPeak; // the held highest current, mA
  peak_hold::Hold voltageLow;  // the held lowest voltage, mV
};

struct Header
{
  Text mode;      // body mode and gait, or the gesture playing
  Text gamepad;   // connection and battery
  Text link;      // link state
};

struct Strip
{
  char lines[STRIP_LINES][STRIP_LINE_CHARS + 1]; // the second often empty
  Tone tone; // Bad and Warn fill the strip
};

struct DriveBody
{
  Text voltage, current;
  Bar voltageBar, currentBar;
  Text height, speed;
  Bar heightBar, speedBar;
  Text modes;   // stride and stick mode
  Text output;  // servo output on or off
  Text sensors; // empty until telemetry reports sensors
};

struct PowerBody
{
  Text voltage, minVoltage; // newest, and the window's lowest
  Text current, maxCurrent; // newest, and the window's highest
  Text note;
  // The traces' scales.
  uint16_t traceLowMv, traceHighMv, traceHighMa;
};

struct LinkBody
{
  Text robot;   // the robot's decoder, accepted and rejected per second
  Text bridge;  // the bridge's decoder
  Text age;     // of the newest telemetry
  Text source;  // which operator source the robot follows
  Text gamepad; // connection and battery
};

struct View
{
  page_button::Page page;
  Header header;
  Strip strip;
  DriveBody drive;   // filled on the drive page
  PowerBody power;   // on the power page
  LinkBody link;     // on the link page
};

/**
 * @brief Composes @p page: the header and status strip, as on every page,
 * and that page's body.
 */
void composePage(const Inputs &in, page_button::Page page, View &out);

// The setup-mode screen: text on the left, the joining QR code (135 px) on
// the right. Lines hold at most SETUP_LINE_CHARS characters and must fit
// SETUP_TEXT_WIDTH_PX at the screen font.
constexpr int SETUP_LINE_CHARS = 14;
constexpr int SETUP_TEXT_WIDTH_PX = 100; // left of the 135 px QR code

struct SetupInputs
{
  const char *ssid;      // the access point's network name
  const char *password;
  const char *address;   // the portal's address
  bool portalConnected;  // a portal page polled within the last second
  bool emergencyStop;    // the bridge's own e-stop toggle
  bool telemetryFresh;
  protocol::Telemetry telemetry;
};

/**
 * @brief The setup screen: SETUP MODE, network name, password, address,
 * portal client, link state, e-stop and, while the link is up, the body mode.
 */
void composeSetup(const SetupInputs &in, Line out[LINE_COUNT]);

/**
 * @brief Short name of one protection fault bit, or nullptr if unknown.
 */
const char *faultName(uint16_t bit);

} // namespace status_screen

#endif // STATUS_SCREEN_H
