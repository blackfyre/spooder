# firmware/bridge/

The gamepad bridge: the ESP-IDF PlatformIO project for the TTGO T-Display (`mise run build-bridge`, `upload-bridge`, `monitor-bridge`). It builds with the pioarduino platform's own PlatformIO core in the git-ignored `.pio-core/` at the repository root (mise's PlatformIO only bootstraps it; the two cores pin different SCons versions). The first `build-bridge` downloads ESP-IDF and the Xtensa toolchain.

- `main/main.c`: `app_main`; a restart into setup mode skips BTstack and Bluepad32 there.
- `main/bridge.cpp`: Bluepad32 callbacks (one controller slot) and `loop()`: gamepad → mapping → link, the screen, and the GPIO0 hold into setup mode during the logo.
- `main/link`: `robot_link`, a FreeRTOS task above `loop()` that sends a command frame every 20 ms, samples the GPIO35 e-stop button and decodes telemetry and calibration reports. In setup mode its commands come from the portal controller, and it sends calibration sets. (POSIX `link()` is visible on ESP-IDF, hence the name `robot_link`.)
- `main/display`: LovyanGFX ST7789 driver drawing the `status_screen` pages, at most every 250 ms, changed regions only, each through one reusable sprite; `main/screen_draw` draws each region in the Blackfyre dark palette onto any LovyanGFX surface (shared with `tools/bridge-screenshots`); the smooth fonts in `main/fonts/` come from `scripts/make_fonts.py` (`uv run`, by hand); in setup mode, the setup screen with the joining QR code.
- `main/setup_mode`: the setup-mode boot path (RTC magic), the WiFi access point, the DNS and HTTP servers and the setup task. `main/portal_page.h` is the portal page, one hand-written HTML file.

## Rules

- The bridge never runs WiFi and Bluetooth together: setup mode is its own boot path and never starts Bluetooth; normal operation never starts WiFi.
- Values already in the generated `sdkconfig.ttgo-t-display` win over `sdkconfig.defaults`. After changing the defaults, delete that file once so they apply (the setup portal needs `CONFIG_ESP_WIFI_IRAM_OPT=n` to link).
- The partition table is `partitions.csv`, selected by `board_build.partitions` in `platformio.ini`: the pioarduino platform ignores the sdkconfig partition setting. The app is about 1.6 MB; the stock single-app tables (1 MB and 1.5 MB) do not fit it. The platform's own size check misses that (without an `ota_0` partition its limit is the whole 4 MB flash), so `scripts/check_image_size.py` checks the built image against the built table's factory partition, and that table against `partitions.csv`, on every build and before every upload; it prints the headroom. Its logic is tested by `mise run test-bridge-build`, which `mise run test` runs.
