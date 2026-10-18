// The virtual robot: the robot's shared control tick (core/robot/robot_tick)
// on the host, with pseudo-terminals for its links and simulated hardware.
// See openspec/changes/add-virtual-robot and `virtual-robot --help`.
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <string>
#include <vector>

#include "config.h"
#include "pty.h"
#include "robot/robot_tick.h"
#include "sim.h"

using robot_tick::Link;

// The board reads at most this much per link and tick (link's receive ring,
// usb_link::MAX_BYTES_PER_POLL).
constexpr int MAX_BYTES_PER_TICK = 256;

static const char USAGE[] =
    "usage: virtual-robot [options]\n"
    "  --link PATH         keep a symbolic link to the USB terminal at PATH\n"
    "  --bridge-pty        open a terminal for the bridge link (UART1) too\n"
    "  --bridge-link PATH  keep a symbolic link to the bridge terminal at PATH\n"
    "  --speed N           tick N times faster than real time\n"
    "  --stepped           tick only on 'step N' commands\n"
    "  --supply V          initial supply voltage (default 6.0)\n"
    "  --echo              copy the debug text to standard error\n"
    "commands on standard input:\n"
    "  supply V | current A | current auto | load K\n"
    "  bridge off|idle|gamepad|clear | estop [on|off]\n"
    "  unplug | plug | step N | status | quit\n";

static volatile sig_atomic_t stopRequested = 0;
static void onSignal(int) { stopRequested = 1; }

struct Options
{
  std::string link;
  bool bridgePty = false;
  std::string bridgeLink;
  double speed = 1;
  bool stepped = false;
  float supplyV = sim::DEFAULT_SUPPLY_V;
  bool echo = false;
};

class VirtualBoard final : public robot_tick::Hardware
{
public:
  Pty usb;
  Pty bridge; // open only with --bridge-pty
  sim::Servos servos;
  led_pattern::State leds = {};
  bool echo = false;

  void setInhibited(bool inhibited) override { servos.inhibited = inhibited; }
  void stopServos() override { servos.stop(); }
  void applyServos(const body::BodyState &body, const joint_calibration::Table &applied) override
  {
    servos.apply(body, applied);
  }
  bool servosOn() override { return servos.on; }
  void showLeds(const led_pattern::State &state) override { leds = state; }
  bool sendFrame(Link link, const uint8_t *frame, int length) override
  {
    if (link == Link::Usb)
      return usb.writeFrame(frame, length);
    // Without a terminal, the bridge link is a UART with no one listening:
    // the frame goes out (and is lost), as on the board.
    return bridge.isOpen() ? bridge.writeFrame(frame, length) : true;
  }
  void log(const char *line) override
  {
    usb.writeText(line);
    if (echo)
      fputs(line, stderr);
  }
};

static robot_tick::RobotState robot;
static VirtualBoard board;
static sim::Supply supply;
static sim::Bridge simBridge;
static Options opts;
static uint32_t nowMs = 0;
static joint_calibration::Table lastApplied;
static bool lastOn = false;

static void say(const char *format, ...) __attribute__((format(printf, 1, 2)));
static void say(const char *format, ...)
{
  va_list args;
  va_start(args, format);
  vfprintf(stdout, format, args);
  va_end(args);
  fputc('\n', stdout);
  fflush(stdout);
}

static bool openUsb()
{
  std::string error;
  if (!board.usb.open(opts.link, error))
  {
    say("error %s", error.c_str());
    return false;
  }
  say("usb %s", board.usb.path().c_str());
  return true;
}

// Reports what the hardware would have done this tick.
static void observe()
{
  for (int j = 0; j < joint_calibration::JOINT_COUNT; j++)
  {
    const joint_calibration::Joint &a = robot.applied.joints[j];
    const joint_calibration::Joint &b = lastApplied.joints[j];
    if (a.offsetDeg != b.offsetDeg || a.pulse0Us != b.pulse0Us || a.pulse180Us != b.pulse180Us ||
        a.reversed != b.reversed)
      say("applied t=%u joint=%d offset=%.4f pulse0=%.3f pulse180=%.3f reversed=%d", nowMs, j, a.offsetDeg,
          a.pulse0Us, a.pulse180Us, a.reversed ? 1 : 0);
  }
  lastApplied = robot.applied;
  if (board.servos.on != lastOn)
  {
    lastOn = board.servos.on;
    say("output t=%u %s", nowMs, lastOn ? "on" : "off");
  }
}

static void tickOnce()
{
  static uint8_t usbBytes[MAX_BYTES_PER_TICK];
  static uint8_t bridgeBytes[MAX_BYTES_PER_TICK];
  static std::vector<uint8_t> simulated;
  robot_tick::Inputs in = {};
  in.bytes[(int)Link::Usb] = usbBytes;
  in.byteCount[(int)Link::Usb] = board.usb.read(usbBytes, MAX_BYTES_PER_TICK);
  if (board.bridge.isOpen())
  {
    in.bytes[(int)Link::Bridge] = bridgeBytes;
    in.byteCount[(int)Link::Bridge] = board.bridge.read(bridgeBytes, MAX_BYTES_PER_TICK);
  }
  else
  {
    simulated.clear();
    simBridge.frame(simulated);
    in.bytes[(int)Link::Bridge] = simulated.data();
    in.byteCount[(int)Link::Bridge] = (int)simulated.size();
  }
  in.power = supply.read(board.servos);
  nowMs += robot_tick::LOOP_PERIOD_MS;
  robot_tick::tick(robot, in, nowMs, board);
  observe();
}

static const char *const MODE_NAMES[] = {"initialise", "stand", "walk", "sleep", "calibrate"};

static bool parseFloat(const char *s, float &out)
{
  char *end;
  errno = 0;
  double v = strtod(s, &end);
  if (errno || end == s || *end || !isfinite(v))
    return false;
  out = (float)v;
  return true;
}

// Handles one command line; returns false to quit.
static bool command(char *line)
{
  char *argv[4] = {};
  int argc = 0;
  for (char *t = strtok(line, " \t\r\n"); t && argc < 4; t = strtok(nullptr, " \t\r\n"))
    argv[argc++] = t;
  if (argc == 0)
    return true;
  std::string c = argv[0];
  float v;
  if (c == "quit" || c == "exit")
    return false;
  if (c == "supply" && argc == 2 && parseFloat(argv[1], v) && v >= 0)
  {
    supply.voltageV = v;
    say("ok supply %.2f", v);
  }
  else if (c == "current" && argc == 2 && strcmp(argv[1], "auto") == 0)
  {
    supply.fixedCurrent = false;
    say("ok current auto");
  }
  else if (c == "current" && argc == 2 && parseFloat(argv[1], v) && v >= 0)
  {
    supply.fixedCurrent = true;
    supply.fixedCurrentA = v;
    say("ok current %.2f", v);
  }
  else if (c == "load" && argc == 2 && parseFloat(argv[1], v) && v >= 0)
  {
    supply.load = v;
    say("ok load %.2f", v);
  }
  else if (c == "bridge" && argc == 2 && !board.bridge.isOpen())
  {
    std::string a = argv[1];
    if (a == "off")
      simBridge.setState(sim::Bridge::State::Off);
    else if (a == "idle")
      simBridge.setState(sim::Bridge::State::Idle);
    else if (a == "gamepad")
      simBridge.setState(sim::Bridge::State::Gamepad);
    else if (a == "clear")
      simBridge.clear();
    else
    {
      say("error bridge off|idle|gamepad|clear");
      return true;
    }
    say("ok bridge %s", argv[1]);
  }
  else if (c == "estop" && !board.bridge.isOpen() &&
           (argc == 1 || (argc == 2 && (strcmp(argv[1], "on") == 0 || strcmp(argv[1], "off") == 0))))
  {
    simBridge.estop = argc == 1 || strcmp(argv[1], "on") == 0;
    if (simBridge.estop && simBridge.state == sim::Bridge::State::Off)
      simBridge.setState(sim::Bridge::State::Idle); // the stop travels in the bridge's frames
    say("ok estop %s", simBridge.estop ? "on" : "off");
  }
  else if (c == "unplug" && argc == 1)
  {
    board.usb.close();
    say("ok unplug");
  }
  else if (c == "plug" && argc == 1)
  {
    if (board.usb.isOpen() || openUsb())
      say("ok plug");
  }
  else if (c == "step" && argc == 2 && opts.stepped)
  {
    int n = atoi(argv[1]);
    for (int i = 0; i < n; i++)
      tickOnce();
    say("tick t=%u", nowMs);
  }
  else if (c == "status" && argc == 1)
  {
    protection::Reading r = supply.read(board.servos);
    say("status t=%u mode=%s running=%d output=%s faults=0x%02x supply=%.2f current=%.2f", nowMs,
        MODE_NAMES[(int)robot.body.mode], robot.supervisor.bodyRunning ? 1 : 0, board.servos.on ? "on" : "off",
        robot.protection.faults, r.voltageV, r.currentA);
  }
  else
    say("error unknown command: %s (see --help)", argv[0]);
  return true;
}

// Reads complete lines from standard input; returns false on quit or end of input.
static bool readCommands()
{
  static std::string pending;
  char buf[512];
  ssize_t n = read(STDIN_FILENO, buf, sizeof buf);
  if (n == 0)
    return false; // end of input: the controlling program has gone
  if (n < 0)
    return errno == EAGAIN || errno == EINTR;
  pending.append(buf, (size_t)n);
  size_t at;
  while ((at = pending.find('\n')) != std::string::npos)
  {
    std::string line = pending.substr(0, at);
    pending.erase(0, at + 1);
    std::vector<char> copy(line.begin(), line.end());
    copy.push_back('\0');
    if (!command(copy.data()))
      return false;
  }
  return true;
}

static int64_t monotonicNs()
{
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * 1000000000 + t.tv_nsec;
}

static bool parseArgs(int argc, char **argv)
{
  for (int i = 1; i < argc; i++)
  {
    std::string a = argv[i];
    auto value = [&](const char *name) -> const char * {
      if (i + 1 >= argc)
      {
        fprintf(stderr, "virtual-robot: %s needs a value\n", name);
        exit(2);
      }
      return argv[++i];
    };
    if (a == "--link")
      opts.link = value("--link");
    else if (a == "--bridge-pty")
      opts.bridgePty = true;
    else if (a == "--bridge-link")
    {
      opts.bridgePty = true;
      opts.bridgeLink = value("--bridge-link");
    }
    else if (a == "--speed")
    {
      float s;
      if (!parseFloat(value("--speed"), s) || s <= 0)
        return false;
      opts.speed = s;
    }
    else if (a == "--stepped")
      opts.stepped = true;
    else if (a == "--supply")
    {
      if (!parseFloat(value("--supply"), opts.supplyV) || opts.supplyV < 0)
        return false;
    }
    else if (a == "--echo")
      opts.echo = true;
    else if (a == "--help" || a == "-h")
    {
      fputs(USAGE, stdout);
      exit(0);
    }
    else
      return false;
  }
  return true;
}

int main(int argc, char **argv)
{
  if (!parseArgs(argc, argv))
  {
    fputs(USAGE, stderr);
    return 2;
  }
  signal(SIGINT, onSignal);
  signal(SIGTERM, onSignal);
  signal(SIGHUP, onSignal);
  signal(SIGPIPE, SIG_IGN);

  board.echo = opts.echo;
  supply.voltageV = opts.supplyV;
  robot_tick::init(robot, robot_tick::InputMode::Links, config::POWER_LIMITS, nowMs);
  lastApplied = robot.applied;
  if (!openUsb())
    return 1;
  if (opts.bridgePty)
  {
    std::string error;
    if (!board.bridge.open(opts.bridgeLink, error))
    {
      say("error %s", error.c_str());
      return 1;
    }
    say("bridge %s", board.bridge.path().c_str());
  }
  say("ready %s", opts.stepped ? "stepped" : "running");

  const int64_t periodNs = (int64_t)(robot_tick::LOOP_PERIOD_MS * 1e6 / opts.speed);
  int64_t next = monotonicNs() + periodNs;
  bool running = true;
  while (running && !stopRequested)
  {
    // Wait for a command, room for held output, or the next deadline, to the
    // nanosecond (a faster --speed has sub-millisecond periods).
    struct pollfd p[3] = {{STDIN_FILENO, POLLIN, 0}};
    Pty *held[2] = {};
    nfds_t n = 1;
    for (Pty *pty : {&board.usb, &board.bridge})
      if (pty->backlogged())
      {
        held[n - 1] = pty;
        p[n++] = {pty->fd(), POLLOUT, 0};
      }
    struct timespec wait = {0, 0};
    struct timespec *timeout = nullptr; // stepped: commands only
    if (!opts.stepped)
    {
      int64_t left = next - monotonicNs();
      if (left > 0)
        wait = {(time_t)(left / 1000000000), (long)(left % 1000000000)};
      timeout = &wait;
    }
    int r = ppoll(p, n, timeout, nullptr);
    for (nfds_t i = 1; r > 0 && i < n; i++)
      if (p[i].revents)
        held[i - 1]->flush(); // sends what fits, or drops it all if no one is connected
    if (r > 0 && (p[0].revents & (POLLIN | POLLHUP)))
      running = readCommands();
    if (!running || opts.stepped)
      continue;
    if (monotonicNs() >= next)
    {
      tickOnce();
      next += periodNs;
      if (monotonicNs() > next)
        next = monotonicNs(); // overran: start the next period now, as the board does
    }
  }
  board.usb.close();
  board.bridge.close();
  return 0;
}
