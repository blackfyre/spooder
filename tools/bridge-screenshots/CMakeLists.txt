cmake_minimum_required(VERSION 3.13)

# Screenshots of the bridge's status screen: the bridge's own drawing code
# (firmware/bridge/main/screen_draw) and page content (core/status) on the
# host, with LovyanGFX's Linux back end, into PNGs. Built and run by
# `mise run bridge-screenshots`.
project(bridge_screenshots C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SPOODER_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
set(SPOODER_CORE ${SPOODER_ROOT}/core)
set(BRIDGE_MAIN ${SPOODER_ROOT}/firmware/bridge/main)
set(LGFX ${SPOODER_ROOT}/external/LovyanGFX/src)

# LovyanGFX for the host, as its own framebuffer example builds it; only
# sprites are used, never a panel.
file(GLOB LGFX_SOURCES CONFIGURE_DEPENDS
  ${LGFX}/lgfx/Fonts/efont/*.c
  ${LGFX}/lgfx/Fonts/IPA/*.c
  ${LGFX}/lgfx/Fonts/lvgl/*.c
  ${LGFX}/lgfx/utility/*.c
  ${LGFX}/lgfx/v1/*.cpp
  ${LGFX}/lgfx/v1/misc/*.cpp
  ${LGFX}/lgfx/v1/panel/*.cpp
  ${LGFX}/lgfx/v1/lv_font/*.c
)
add_library(lgfx_host STATIC ${LGFX_SOURCES})
target_include_directories(lgfx_host PUBLIC ${LGFX})
target_compile_definitions(lgfx_host PUBLIC LGFX_LINUX_FB)
target_compile_options(lgfx_host PRIVATE -w) # third-party code

add_executable(bridge-screenshots
  src/main.cpp
  ${BRIDGE_MAIN}/screen_draw.cpp
  ${SPOODER_CORE}/status/page_button.cpp
  ${SPOODER_CORE}/status/peak_hold.cpp
  ${SPOODER_CORE}/status/power_history.cpp
  ${SPOODER_CORE}/status/status_screen.cpp
)
target_include_directories(bridge-screenshots PRIVATE ${SPOODER_CORE} ${BRIDGE_MAIN})
target_compile_options(bridge-screenshots PRIVATE -Wall)
target_link_libraries(bridge-screenshots PRIVATE lgfx_host pthread)
