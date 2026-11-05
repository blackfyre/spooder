#ifndef VIRTUAL_ROBOT_WIFI_BRIDGE_H
#define VIRTUAL_ROBOT_WIFI_BRIDGE_H

// The virtual bridge in WiFi drive (--wifi-bridge): a UDP socket standing
// for the bridge's access point and UDP task, with the bridge firmware's own
// rules (core/wifi_drive/handheld_controller, relay_queue and
// core/gamepad_mapping's command frame). Each tick it reads the datagrams
// that arrived, builds the bridge's command frame for the bridge link's
// decoder, and relays the robot's bridge-link frames to the locked handheld.
// See openspec/specs/virtual-robot (Virtual bridge in WiFi drive).
#include <stdint.h>
#include <string>
#include <vector>
#include "wifi_drive/handheld_controller.h"
#include "wifi_drive/relay_queue.h"

class WifiBridge
{
public:
  // Datagrams read per tick, at most: the tick's work stays bounded however
  // fast a sender floods. The receive buffer is the kernel's minimum, which
  // holds fewer, so each tick empties it.
  static constexpr int READS_PER_TICK = 32;
  static constexpr const char *DEFAULT_ADDRESS = "127.0.0.1";

  ~WifiBridge() { close(); }

  /**
   * @brief Parses "[ADDR:]PORT" (ADDR an IPv4 address, default
   * DEFAULT_ADDRESS; PORT 0..65535, 0 for any free port).
   * @return false if it is not one
   */
  static bool parse(const std::string &spec, uint32_t &address, uint16_t &port);

  /**
   * @brief Binds the socket (network byte order @p address, host order
   * @p port) with the smallest receive buffer the kernel allows.
   * @return false with @p error set on failure
   */
  bool open(uint32_t address, uint16_t port, std::string &error);
  void close();
  bool isOpen() const { return fd_ >= 0; }
  // The bound address, as "ADDR:PORT".
  std::string boundAddress() const;

  /**
   * @brief Starts a tick at robot time @p nowMs: reads the pending
   * datagrams (at most READS_PER_TICK) into the controller, reports session
   * changes, and appends the bridge's command frame (its e-stop @p gpioStop
   * ORed with the handheld's) to @p bridgeBytes. Then begins the relay's poll.
   */
  void beforeTick(uint32_t nowMs, bool gpioStop, std::vector<uint8_t> &bridgeBytes);

  /**
   * @brief One frame the robot sent on the bridge link during the tick.
   */
  void robotFrame(uint32_t nowMs, const uint8_t *frame, int length);

  /**
   * @brief Ends the tick: ends the relay's poll (its frames arrive whole, so
   * the decoder is always empty) and sends the queued frames.
   */
  void afterTick(uint32_t nowMs);

  // The controller, for the status command.
  const handheld_controller::Controller &controller() const { return controller_; }

private:
  void track(uint32_t nowMs);

  int fd_ = -1;
  handheld_controller::Controller controller_;
  relay_queue::Queue relay_;
  uint8_t seq_ = 0;
  // Session reporting.
  bool live_ = false;
  uint32_t seenEpoch_ = 0;
  uint32_t acceptedAtStart_ = 0;
  int64_t lastTickStartNs_ = 0;
  int64_t longestTickNs_ = 0; // the session's longest interval between tick starts
  int charged_ = 0;           // datagrams charged to this tick's window budget
  int budgetMax_ = 0;         // the session's most in one window
};

#endif // VIRTUAL_ROBOT_WIFI_BRIDGE_H
