# Bridge sources plus the hardware-free core shared with the robot.
set(SPOODER_CORE ${CMAKE_CURRENT_LIST_DIR}/../../../core)

idf_component_register(
  SRCS
    "main.c"
    "bridge.cpp"
    "display.cpp"
    "link.cpp"
    "${SPOODER_CORE}/gamepad_mapping/gamepad_mapping.cpp"
    "${SPOODER_CORE}/protocol/protocol.cpp"
    "${SPOODER_CORE}/status/status_screen.cpp"
  INCLUDE_DIRS "." "${SPOODER_CORE}"
  REQUIRES bluepad32 bluepad32_arduino arduino btstack LovyanGFX
)
