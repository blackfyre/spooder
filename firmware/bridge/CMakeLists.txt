# ESP-IDF project for the gamepad bridge. Components come from the pinned
# Bluepad32 template (Arduino core, Bluepad32, BTstack) and LovyanGFX.
cmake_minimum_required(VERSION 3.16)

set(SPOODER_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
set(BP32_TEMPLATE ${SPOODER_ROOT}/external/bluepad32-template)

set(ENV{BTSTACK_ROOT} ${BP32_TEMPLATE}/external/btstack)
set(EXTRA_COMPONENT_DIRS
  ${BP32_TEMPLATE}/components
  ${SPOODER_ROOT}/external/LovyanGFX
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
include($ENV{BTSTACK_ROOT}/port/esp32/components/btstack/cmake/BTstack_project.cmake)

project(spooder_bridge)
