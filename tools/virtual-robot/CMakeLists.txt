cmake_minimum_required(VERSION 3.13)

# The virtual robot: the robot's shared control tick (core/) on the host,
# with pseudo-terminals for its links. Built natively: `mise run virtual-robot`.
project(virtual_robot CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SPOODER_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
set(SPOODER_CORE ${SPOODER_ROOT}/core)
set(SPOODER_FIRMWARE ${SPOODER_ROOT}/firmware/servo2040/src)

# The same limits header the firmware is built with (through its config.h and
# static checks), unless another one is given.
set(SPOODER_POWER_LIMITS "${SPOODER_FIRMWARE}/power_limits.h" CACHE FILEPATH
    "Header defining the SPOODER_* power limits (see firmware/servo2040/src/power_limits.example.h)")
if(NOT EXISTS "${SPOODER_POWER_LIMITS}")
  message(FATAL_ERROR "No power limits file at ${SPOODER_POWER_LIMITS}; pass -DSPOODER_POWER_LIMITS=<file>.")
endif()

# Build identity: the virtual robot's own build (see
# firmware/servo2040/CMakeLists.txt and cmake/build_identity.cmake).
set(SPOODER_BUILD_IDENTITY_SOURCE ${CMAKE_CURRENT_BINARY_DIR}/build_identity_id.cpp)
add_custom_target(virtual_robot_build_identity
  COMMAND ${CMAKE_COMMAND} -DREPO_ROOT=${SPOODER_ROOT} -DOUTPUT=${SPOODER_BUILD_IDENTITY_SOURCE}
          -P ${SPOODER_ROOT}/cmake/build_identity.cmake
  BYPRODUCTS ${SPOODER_BUILD_IDENTITY_SOURCE}
  COMMENT "Build identity"
  VERBATIM)

add_executable(virtual-robot
  src/main.cpp
  ${SPOODER_BUILD_IDENTITY_SOURCE}
  src/pty.cpp
  src/sim.cpp
  src/wifi_bridge.cpp
  # The robot's own logic, the same sources as the firmware's spooder_core.
  # The bridge's WiFi drive rules (--wifi-bridge), the bridge firmware's own.
  ${SPOODER_CORE}/gamepad_mapping/gamepad_mapping.cpp
  ${SPOODER_CORE}/wifi_drive/handheld_controller.cpp
  ${SPOODER_CORE}/wifi_drive/relay_queue.cpp
  ${SPOODER_CORE}/movement/body.cpp
  ${SPOODER_CORE}/movement/calibration_defaults.cpp
  ${SPOODER_CORE}/movement/gait.cpp
  ${SPOODER_CORE}/movement/gesture.cpp
  ${SPOODER_CORE}/movement/joint_calibration.cpp
  ${SPOODER_CORE}/movement/kinematics.cpp
  ${SPOODER_CORE}/protocol/link_receiver.cpp
  ${SPOODER_CORE}/protocol/protocol.cpp
  ${SPOODER_CORE}/protection/protection.cpp
  ${SPOODER_CORE}/protection/resume_gate.cpp
  ${SPOODER_CORE}/protection/supervisor.cpp
  ${SPOODER_CORE}/robot/bench_model.cpp
  ${SPOODER_CORE}/robot/robot_tick.cpp
  ${SPOODER_CORE}/identity/build_identity.cpp
  ${SPOODER_CORE}/status/led_pattern.cpp
)
add_dependencies(virtual-robot virtual_robot_build_identity)
target_include_directories(virtual-robot PRIVATE ${SPOODER_CORE} ${SPOODER_FIRMWARE})
# The same servo profile choice as the firmware.
set(SPOODER_SERVO_PROFILE "MG996R" CACHE STRING "Servo model fitted (see core/movement/servo_profile.h)")
target_compile_definitions(virtual-robot PRIVATE
  SPOODER_POWER_LIMITS_HEADER="${SPOODER_POWER_LIMITS}"
  SPOODER_SERVO_PROFILE="${SPOODER_SERVO_PROFILE}"
)
target_compile_options(virtual-robot PRIVATE -Wall -Wextra)
