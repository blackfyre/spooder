#include "wifi_drive.h"

#include <Arduino.h>
#include <WiFi.h>
#include <atomic>
#include <driver/gpio.h>
#include <esp_bt.h>
#include <esp_mac.h>
#include <esp_rom_sys.h>
#include <lwip/sockets.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "display.h"
#include "identity/build_identity.h"
#include "link.h"
#include "setup/access_point.h"
#include "setup/setup_entry.h"
#include "setup_mode.h"
#include "status/page_button.h"
#include "wifi_drive/drive_mode.h"
#include "wifi_drive/handheld_controller.h"

// --- the drive mode switch ------------------------------------------------------

namespace
{
constexpr gpio_num_t SWITCH_PIN = GPIO_NUM_32; // to ground; open is Bluetooth drive
constexpr uint32_t SWITCH_SETTLE_US = 1000;    // the pull-up charging the pin
} // namespace

bool drive_switch_closed(void)
{
  return gpio_get_level(SWITCH_PIN) == 0;
}

enum bridge_boot_mode bridge_boot_mode_select(bool setupRequested)
{
  gpio_config_t io = {};
  io.pin_bit_mask = 1ULL << SWITCH_PIN;
  io.mode = GPIO_MODE_INPUT;
  io.pull_up_en = GPIO_PULLUP_ENABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&io);
  esp_rom_delay_us(SWITCH_SETTLE_US);
  switch (drive_mode::select(setupRequested, drive_switch_closed()))
  {
  case drive_mode::Mode::Setup:
    return BRIDGE_BOOT_SETUP;
  case drive_mode::Mode::Wifi:
    return BRIDGE_BOOT_WIFI;
  default:
    return BRIDGE_BOOT_BLUETOOTH;
  }
}

// --- WiFi drive -----------------------------------------------------------------

namespace wifi_drive
{

constexpr int MAX_STATIONS = 1;
constexpr int WIFI_CHANNEL = 1;
static const char PASSWORD_KEY[] = "drive_pass"; // in NVS namespace "spooder"
// The left button (GPIO0, active low, board pull-up), as in Bluetooth drive:
// held during the logo it restarts into setup mode; afterwards it pages.
constexpr int SETUP_BUTTON_PIN = 0;
constexpr uint32_t TASK_PERIOD_MS = 2;
constexpr uint32_t COUNTS_LOG_MS = 10000; // the outcome counts on the console
// One byte more than a command frame, so a longer datagram reads as longer
// (lwIP truncates to the buffer) and is discarded.
constexpr int DATAGRAM_BUFFER = protocol::COMMAND_FRAME_SIZE + 1;

static char ssid[access_point::SSID_SIZE];
static char password[access_point::PASSWORD_LENGTH + 1];

static setup_entry::Hold setupHold;
static page_button::State pageButton;
static drive_mode::Notice switchNotice;

static int sock = -1;
static std::atomic<uint32_t> sendFailures; // relay datagrams lwIP refused

static void openSocket()
{
  sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock < 0)
  {
    printf("WiFi drive: no UDP socket\n");
    return;
  }
  sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(handheld_controller::PORT);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(sock, (sockaddr *)&addr, sizeof(addr)) != 0)
  {
    printf("WiFi drive: UDP port %u not bound\n", handheld_controller::PORT);
    close(sock);
    sock = -1;
    return;
  }
  // One window: the task gets back to the relay at least that often.
  timeval timeout = {};
  timeout.tv_usec = handheld_controller::WINDOW_MS * 1000;
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
}

// Sends every frame waiting for the relay, one per datagram, never waiting.
static void relay()
{
  uint8_t frame[protocol::MAX_FRAME_SIZE];
  handheld_controller::Sender to;
  int n;
  while ((n = robot_link::handheldTakeRelay(frame, to)) > 0)
  {
    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(to.port);
    addr.sin_addr.s_addr = to.address;
    if (sendto(sock, frame, n, MSG_DONTWAIT, (sockaddr *)&addr, sizeof(addr)) != n)
      sendFailures++;
  }
}

// Reads the handheld's datagrams, at most a window's budget per window: once
// it is spent the rest stay in lwIP's bounded receive mailbox
// (CONFIG_LWIP_UDP_RECVMBOX_SIZE, smaller than the budget) until the next
// window, so none waits longer than one window.
static void udpTask(void *)
{
  uint8_t datagram[DATAGRAM_BUFFER];
  for (;;)
  {
    relay();
    uint32_t now = millis();
    if (robot_link::handheldBudgetSpent(now))
    {
      uint32_t untilNext = handheld_controller::WINDOW_MS - now % handheld_controller::WINDOW_MS;
      vTaskDelay(pdMS_TO_TICKS(untilNext) > 0 ? pdMS_TO_TICKS(untilNext) : 1);
      continue;
    }
    sockaddr_in from = {};
    socklen_t fromLen = sizeof(from);
    int n = recvfrom(sock, datagram, sizeof(datagram), 0, (sockaddr *)&from, &fromLen);
    if (n < 0)
      continue; // the timeout: no datagram this window
    handheld_controller::Sender sender = {from.sin_addr.s_addr, ntohs(from.sin_port)};
    robot_link::handheldAccept(datagram, n, sender); // stamped on receipt
  }
}

static void logCounts(uint32_t now)
{
  static uint32_t lastMs;
  static uint32_t shown[handheld_controller::OUTCOME_COUNT + 2];
  if (now - lastMs < COUNTS_LOG_MS)
    return;
  lastMs = now;
  static robot_link::HandheldView view;
  robot_link::handheldView(view);
  uint32_t current[handheld_controller::OUTCOME_COUNT + 2];
  memcpy(current, view.counts, sizeof(view.counts));
  current[handheld_controller::OUTCOME_COUNT] = view.relayDropped;
  current[handheld_controller::OUTCOME_COUNT + 1] = sendFailures.load();
  if (memcmp(current, shown, sizeof(current)) == 0)
    return;
  memcpy(shown, current, sizeof(current));
  using handheld_controller::Outcome;
  printf("WiFi drive: accepted %lu, rate-limited %lu, malformed %lu, other sender %lu, stale %lu, jump %lu; relay "
         "dropped %lu, send failed %lu\n",
         (unsigned long)current[(int)Outcome::Accepted], (unsigned long)current[(int)Outcome::RateLimited],
         (unsigned long)current[(int)Outcome::Malformed], (unsigned long)current[(int)Outcome::OtherSender],
         (unsigned long)current[(int)Outcome::Stale], (unsigned long)current[(int)Outcome::Jump],
         (unsigned long)view.relayDropped, (unsigned long)current[handheld_controller::OUTCOME_COUNT + 1]);
}

// The screen and the buttons, as loop() is in Bluetooth drive.
static void driveTask(void *)
{
  for (;;)
  {
    uint32_t now = millis();
    if (display::showingLogo())
    {
      bool pressed = digitalRead(SETUP_BUTTON_PIN) == LOW;
      if (setup_entry::update(setupHold, pressed, now))
      {
        printf("setup button held: restarting into setup mode\n");
        setup_mode::restartIntoSetup();
      }
      display::holdLogo(setup_entry::held(setupHold));
    }
    else if (display::statusShown())
    {
      page_button::update(pageButton, digitalRead(SETUP_BUTTON_PIN) == LOW, now);
    }

    static robot_link::HandheldView view;
    robot_link::handheldView(view);
    robot_link::Status link = robot_link::status();
    // The screen shows the e-stop the command frames carry: the bridge's
    // own toggle or the handheld's flag.
    link.emergencyStop = link.emergencyStop || view.emergencyStop;

    display::Drive drive = {};
    drive.mode = drive_mode::Mode::Wifi;
    drive.handheld = view.state;
    drive.network = ssid;
    drive.password = password;
    drive.switchNotice = drive_mode::update(switchNotice, drive_mode::Mode::Wifi, drive_switch_closed(), now);
    display::update(now, false, 0, pageButton.page, view.desired, link, drive);
    logCounts(now);
    vTaskDelay(pdMS_TO_TICKS(TASK_PERIOD_MS));
  }
}

} // namespace wifi_drive

void wifi_drive_start(void)
{
  using namespace wifi_drive;
  char banner[build_identity::BANNER_SIZE];
  build_identity::banner(banner, sizeof(banner), "bridge", build_identity::ID);
  printf("%s", banner);
  // Bluetooth is never started in WiFi drive; its controller memory goes
  // back to the heap for WiFi.
  esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
  initArduino();
  printf("spooder bridge: WiFi drive (Bluetooth off)\n");

  display::setup(build_identity::ID); // logo first
  pinMode(SETUP_BUTTON_PIN, INPUT_PULLUP);
  robot_link::startHandheld(); // command frames from here on
  display::bootMessage("Robot link: UART1 26/27");

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
  access_point::driveSsid(mac, ssid);
  setup_mode::loadPassword(PASSWORD_KEY, password); // before WiFi starts

  WiFi.persistent(false); // nothing of the WiFi configuration goes to flash
  WiFi.mode(WIFI_AP);
  bool up = WiFi.softAP(ssid, password, WIFI_CHANNEL, 0, MAX_STATIONS);
  if (!up)
    printf("WiFi drive: access point failed to start\n");
  display::bootMessage("WiFi drive (Bluetooth off)");
  char line[32];
  snprintf(line, sizeof(line), "Net %s", ssid);
  display::bootMessage(line, up);
  openSocket();
  display::bootMessage(sock >= 0 ? "Handheld: UDP 52040" : "Handheld: no UDP port", sock >= 0);
  printf("WiFi drive: access point %s, UDP port %u\n", ssid, handheld_controller::PORT);
  display::bootMessage("Ready!");
  display::bootMessage("Waiting on handheld...");

  // Below the link task (priority 5) on its core, so neither the datagrams
  // nor the screen ever delay command frames; the UDP task above the screen,
  // so a slow redraw never delays reading.
  if (sock >= 0)
    xTaskCreatePinnedToCore(udpTask, "udp", 4096, nullptr, 2, nullptr, ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(driveTask, "wifi-drive", 8192, nullptr, 1, nullptr, ARDUINO_RUNNING_CORE);
}
