cmake_minimum_required(VERSION 3.13)

# Pinned SDKs live in external/ (git submodules); mise sets PICO_SDK_PATH,
# PIMORONI_PICO_PATH and PICO_BOARD=pimoroni_servo2040.
include(pico_sdk_import.cmake)
include(pimoroni_pico_import.cmake)

project(spooder C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

include(libraries/servo2040/servo2040)
include(drivers/analog/analog)
include(drivers/analogmux/analogmux)

# Hardware-free code shared with the host tests (core/).
set(SPOODER_CORE ${CMAKE_CURRENT_LIST_DIR}/../../core)
add_library(spooder_core STATIC
  ${SPOODER_CORE}/movement/body.cpp
  ${SPOODER_CORE}/movement/calibration_defaults.cpp
  ${SPOODER_CORE}/movement/gait.cpp
  ${SPOODER_CORE}/movement/joint_calibration.cpp
  ${SPOODER_CORE}/movement/kinematics.cpp
  ${SPOODER_CORE}/protocol/link_receiver.cpp
  ${SPOODER_CORE}/protocol/protocol.cpp
  ${SPOODER_CORE}/protection/protection.cpp
  ${SPOODER_CORE}/protection/resume_gate.cpp
  ${SPOODER_CORE}/protection/supervisor.cpp
  ${SPOODER_CORE}/status/led_pattern.cpp
)
target_include_directories(spooder_core PUBLIC ${SPOODER_CORE})
target_compile_options(spooder_core PRIVATE -Wall -Wextra)

# Power-protection limits: required, chosen for the servos and supply fitted.
set(SPOODER_POWER_LIMITS "${CMAKE_CURRENT_LIST_DIR}/src/power_limits.h" CACHE FILEPATH
    "Header defining the SPOODER_* power limits (see src/power_limits.example.h)")
if(NOT EXISTS "${SPOODER_POWER_LIMITS}")
  message(FATAL_ERROR
    "No power limits file at ${SPOODER_POWER_LIMITS}.\n"
    "Copy src/power_limits.example.h to src/power_limits.h and set the limits for your "
    "servos and supply, or pass -DSPOODER_POWER_LIMITS=<file> / set SPOODER_POWER_LIMITS.")
endif()

# Command source: the gamepad bridge link (default), or the USB serial console
# for bring-up and as a rollback.
option(SPOODER_SERIAL_COMMANDS "Take commands from the USB serial console instead of the bridge link" OFF)

add_executable(spooder
  src/main.cpp
  src/power_monitor.cpp
  src/servo_output.cpp
  src/status_leds.cpp
)
if(SPOODER_SERIAL_COMMANDS)
  target_sources(spooder PRIVATE src/serial_commands.cpp)
  target_compile_definitions(spooder PRIVATE SPOODER_SERIAL_COMMANDS)
else()
  target_sources(spooder PRIVATE src/link.cpp)
endif()

target_compile_options(spooder PRIVATE -Wall)
# -Wextra only for our sources: the SDK's interface libraries compile into
# this target too.
get_target_property(SPOODER_SOURCES spooder SOURCES)
set_source_files_properties(${SPOODER_SOURCES} PROPERTIES COMPILE_OPTIONS -Wextra)
target_compile_definitions(spooder PRIVATE
  SPOODER_POWER_LIMITS_HEADER="${SPOODER_POWER_LIMITS}"
  # A connected host that is not reading must not stall the control loop:
  # drop debug output after 1 ms instead of the default 500 ms wait.
  PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)
target_link_libraries(spooder
  pico_stdlib
  hardware_uart
  servo2040
  analogmux
  analog
  spooder_core
)

# USB console only: the default UART pins (GPIO0/1) are servo outputs 1 and 2.
pico_enable_stdio_usb(spooder 1)
pico_enable_stdio_uart(spooder 0)

pico_add_extra_outputs(spooder)
