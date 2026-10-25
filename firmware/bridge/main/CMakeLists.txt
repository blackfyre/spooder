# Bridge sources plus the hardware-free core shared with the robot.
set(SPOODER_CORE ${CMAKE_CURRENT_LIST_DIR}/../../../core)

# Build identity (cmake/build_identity.cmake): PlatformIO compiles the sources
# of this component itself, from CMake's code model, and never runs a CMake
# custom target, so scripts/build_identity.py (a pre script) writes this file
# into the build directory on every build, rewriting it only when it changes.
set(SPOODER_BUILD_IDENTITY_SOURCE ${CMAKE_BINARY_DIR}/build_identity_id.cpp)
if(NOT CMAKE_BUILD_EARLY_EXPANSION) # not scriptable in ESP-IDF's requirements pass
  set_source_files_properties(${SPOODER_BUILD_IDENTITY_SOURCE} PROPERTIES GENERATED TRUE)
endif()

idf_component_register(
  SRCS
    "main.c"
    "bridge.cpp"
    "display.cpp"
    "screen_draw.cpp"
    "link.cpp"
    "setup_mode.cpp"
    "${SPOODER_BUILD_IDENTITY_SOURCE}"
    "${SPOODER_CORE}/gamepad_mapping/gamepad_mapping.cpp"
    "${SPOODER_CORE}/gamepad_mapping/rumble.cpp"
    "${SPOODER_CORE}/identity/build_identity.cpp"
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

# Warnings are errors in Spooder's own sources: this component's files, the
# bridge's and the shared core/ ones, never another component's (PRIVATE).
# ESP-IDF enables neither -Wall nor -Wextra and switches some of their warnings
# off or keeps them non-fatal, and a specific -W<name> outranks -Wall/-Wextra,
# so each of those is named again here, after ESP-IDF's options. A warning
# raised inside an SDK header is handled at its include (see CLAUDE.md).
target_compile_options(${COMPONENT_LIB} PRIVATE
  -Wall -Wextra -Werror
  -Wframe-address -Wunused-parameter -Wsign-compare
  $<$<COMPILE_LANGUAGE:C>:-Wenum-conversion -Wold-style-declaration>
  -Werror=unused-function -Werror=unused-variable -Werror=unused-but-set-variable
  -Werror=deprecated-declarations -Werror=extra
)
