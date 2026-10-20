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

add_executable(virtual-robot
  src/main.cpp
  src/pty.cpp
  src/sim.cpp
  # The robot's own logic, the same sources as the firmware's spooder_core.
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
  ${SPOODER_CORE}/robot/robot_tick.cpp
  ${SPOODER_CORE}/status/led_pattern.cpp
)
target_include_directories(virtual-robot PRIVATE ${SPOODER_CORE} ${SPOODER_FIRMWARE})
# The same servo profile choice as the firmware.
set(SPOODER_SERVO_PROFILE "MG996R" CACHE STRING "Servo model fitted (see core/movement/servo_profile.h)")
target_compile_definitions(virtual-robot PRIVATE
  SPOODER_POWER_LIMITS_HEADER="${SPOODER_POWER_LIMITS}"
  SPOODER_SERVO_PROFILE="${SPOODER_SERVO_PROFILE}"
)
target_compile_options(virtual-robot PRIVATE -Wall -Wextra)
