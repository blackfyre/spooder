# Bridge sources plus the hardware-free core shared with the robot.
set(SPOODER_CORE ${CMAKE_CURRENT_LIST_DIR}/../../../core)

idf_component_register(
  SRCS
    "main.c"
    "bridge.cpp"
    "display.cpp"
    "screen_draw.cpp"
    "link.cpp"
    "setup_mode.cpp"
    "${SPOODER_CORE}/gamepad_mapping/gamepad_mapping.cpp"
    "${SPOODER_CORE}/gamepad_mapping/rumble.cpp"
    "${SPOODER_CORE}/protocol/protocol.cpp"
    "${SPOODER_CORE}/setup/access_point.cpp"
    "${SPOODER_CORE}/setup/calibration_export.cpp"
    "${SPOODER_CORE}/setup/calibration_sync.cpp"
    "${SPOODER_CORE}/setup/portal_controller.cpp"
    "${SPOODER_CORE}/setup/setup_entry.cpp"
    "${SPOODER_CORE}/status/page_button.cpp"
    "${SPOODER_CORE}/status/peak_hold.cpp"
    "${SPOODER_CORE}/status/power_history.cpp"
    "${SPOODER_CORE}/status/status_screen.cpp"
  INCLUDE_DIRS "." "${SPOODER_CORE}"
  REQUIRES bluepad32 bluepad32_arduino arduino btstack LovyanGFX bootloader_support bt
)
