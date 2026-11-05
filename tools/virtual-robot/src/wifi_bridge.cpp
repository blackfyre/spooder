#include "wifi_bridge.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "gamepad_mapping/gamepad_mapping.h"
#include "protocol/protocol.h"
#include "robot/robot_tick.h"

using handheld_controller::Outcome;

static int64_t monotonicNs()
{
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * 1000000000 + t.tv_nsec;
}

static std::string format(uint32_t address, uint16_t port)
{
  char ip[INET_ADDRSTRLEN] = "?";
  struct in_addr a = {address};
  inet_ntop(AF_INET, &a, ip, sizeof ip);
  return std::string(ip) + ":" + std::to_string(port);
}

bool WifiBridge::parse(const std::string &spec, uint32_t &address, uint16_t &port)
{
  std::string host = DEFAULT_ADDRESS;
  std::string portText = spec;
  size_t colon = spec.rfind(':');
  if (colon != std::string::npos)
  {
    host = spec.substr(0, colon);
    portText = spec.substr(colon + 1);
  }
  if (portText.empty() || portText.size() > 5 || portText.find_first_not_of("0123456789") != std::string::npos)
    return false;
  long p = strtol(portText.c_str(), nullptr, 10);
  if (p > 65535)
    return false;
  struct in_addr a;
  if (inet_pton(AF_INET, host.c_str(), &a) != 1)
    return false;
  address = a.s_addr;
  port = (uint16_t)p;
  return true;
}

bool WifiBridge::open(uint32_t address, uint16_t port, std::string &error)
{
  close();
  int fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd < 0)
  {
    error = std::string("wifi bridge socket: ") + strerror(errno);
    return false;
  }
  // The kernel raises this to its minimum: a backlog of a few datagrams,
  // fewer than READS_PER_TICK, as the bridge's lwIP mailbox of 6 is smaller
  // than one window's budget.
  int size = 0;
  if (setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &size, sizeof size) != 0)
  {
    error = std::string("wifi bridge SO_RCVBUF: ") + strerror(errno);
    ::close(fd);
    return false;
  }
  struct sockaddr_in at = {};
  at.sin_family = AF_INET;
  at.sin_addr.s_addr = address;
  at.sin_port = htons(port);
  if (bind(fd, (struct sockaddr *)&at, sizeof at) != 0)
  {
    error = "wifi bridge " + format(address, port) + ": " + strerror(errno);
    ::close(fd);
    return false;
  }
  fd_ = fd;
  return true;
}

void WifiBridge::close()
{
  if (fd_ >= 0)
    ::close(fd_);
  fd_ = -1;
}

std::string WifiBridge::boundAddress() const
{
  struct sockaddr_in at = {};
  socklen_t len = sizeof at;
  if (fd_ < 0 || getsockname(fd_, (struct sockaddr *)&at, &len) != 0)
    return "?";
  return format(at.sin_addr.s_addr, ntohs(at.sin_port));
}

// Reports a session's start or end since the last call, with the robot time.
void WifiBridge::track(uint32_t nowMs)
{
  bool now = handheld_controller::live(controller_, nowMs);
  // beforeTick() judges a lapse before it reads a datagram, so one call sees
  // at most one accepted datagram; a change of epoch with a session live
  // before and after is an end and the start that datagram made.
  uint32_t epoch = controller_.epoch;
  if (epoch == seenEpoch_ && now == live_)
    return;
  seenEpoch_ = epoch;
  uint32_t accepted = handheld_controller::count(controller_, Outcome::Accepted);
  if (live_)
  {
    // The datagram that started the next session is not this one's.
    uint32_t ended = now ? accepted - 1 : accepted;
    printf("session end t=%u accepted=%u budget-max=%d longest-tick-ms=%.1f\n", nowMs, ended - acceptedAtStart_,
           budgetMax_, (double)longestTickNs_ / 1e6);
    live_ = false;
  }
  if (now)
  {
    // A session starts only with an accepted frame: count that one.
    acceptedAtStart_ = accepted - 1;
    longestTickNs_ = 0;
    budgetMax_ = charged_;
    live_ = true;
    printf("session start t=%u from %s\n", nowMs, format(controller_.sender.address, controller_.sender.port).c_str());
  }
  fflush(stdout);
}

void WifiBridge::beforeTick(uint32_t nowMs, bool gpioStop, std::vector<uint8_t> &bridgeBytes)
{
  int64_t startNs = monotonicNs();
  if (live_ && lastTickStartNs_ != 0 && startNs - lastTickStartNs_ > longestTickNs_)
    longestTickNs_ = startNs - lastTickStartNs_;
  lastTickStartNs_ = startNs;

  track(nowMs); // a lapse, judged before any datagram of this tick

  // Each tick is one rate window (LOOP_PERIOD_MS == WINDOW_MS): the
  // datagrams the controller charges to its budget, decoded or not.
  static_assert(robot_tick::LOOP_PERIOD_MS == handheld_controller::WINDOW_MS, "one window per tick");
  charged_ = 0;
  // Larger than any valid datagram, so a longer one stays malformed.
  uint8_t buf[2 * protocol::MAX_FRAME_SIZE];
  for (int i = 0; fd_ >= 0 && i < READS_PER_TICK; i++)
  {
    struct sockaddr_in from = {};
    socklen_t len = sizeof from;
    ssize_t n = recvfrom(fd_, buf, sizeof buf, MSG_TRUNC, (struct sockaddr *)&from, &len);
    if (n < 0)
      break; // EAGAIN: the backlog is empty (or an error, retried next tick)
    int length = n > (ssize_t)sizeof buf ? (int)sizeof buf : (int)n;
    handheld_controller::Sender sender = {from.sin_addr.s_addr, ntohs(from.sin_port)};
    if (handheld_controller::accept(controller_, buf, length, sender, nowMs) != Outcome::RateLimited)
      charged_++;
    // Before track(): a datagram that ends the session (a jump) is charged
    // to it.
    if (live_ && charged_ > budgetMax_)
      budgetMax_ = charged_;
    track(nowMs);
  }

  bool stop = gpioStop || handheld_controller::emergencyStop(controller_);
  protocol::Command c = gamepad_mapping::toCommand(handheld_controller::desired(controller_, nowMs), stop);
  uint8_t frame[protocol::MAX_FRAME_SIZE];
  int n = protocol::encodeCommand(c, seq_++, frame);
  bridgeBytes.insert(bridgeBytes.end(), frame, frame + n);

  relay_queue::beginPoll(relay_, controller_, nowMs);
}

void WifiBridge::robotFrame(uint32_t nowMs, const uint8_t *frame, int length)
{
  relay_queue::relayFor(relay_, controller_, nowMs, frame, length);
}

void WifiBridge::afterTick(uint32_t nowMs)
{
  relay_queue::endPoll(relay_, true);
  uint8_t out[protocol::MAX_FRAME_SIZE];
  handheld_controller::Sender to;
  int n;
  while (fd_ >= 0 && (n = relay_queue::takeFor(relay_, controller_, nowMs, out, to)) > 0)
  {
    struct sockaddr_in at = {};
    at.sin_family = AF_INET;
    at.sin_addr.s_addr = to.address;
    at.sin_port = htons(to.port);
    // Never waits: a frame the socket cannot take is lost, as over the air.
    sendto(fd_, out, (size_t)n, MSG_DONTWAIT, (struct sockaddr *)&at, sizeof at);
  }
  track(nowMs);
}
