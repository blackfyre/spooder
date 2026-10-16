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

# Hardware-free code shared with the host tests (core/).
set(SPOODER_CORE ${CMAKE_CURRENT_LIST_DIR}/../../core)
add_library(spooder_core STATIC
  ${SPOODER_CORE}/movement/body.cpp
  ${SPOODER_CORE}/movement/gait.cpp
  ${SPOODER_CORE}/movement/kinematics.cpp
)
target_include_directories(spooder_core PUBLIC ${SPOODER_CORE})
target_compile_options(spooder_core PRIVATE -Wall -Wextra)

add_executable(spooder
  src/main.cpp
)

target_compile_options(spooder PRIVATE -Wall)
target_link_libraries(spooder
  pico_stdlib
  servo2040
  spooder_core
)

# USB console only: the default UART pins (GPIO0/1) are servo outputs 1 and 2.
pico_enable_stdio_usb(spooder 1)
pico_enable_stdio_uart(spooder 0)

pico_add_extra_outputs(spooder)
