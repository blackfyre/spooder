#ifndef LINK_H
#define LINK_H

// The gamepad bridge link on UART1: GPIO20 TX, GPIO21 RX (J6 pins 14 and 13),
// 115200 8N1. Bytes move under interrupts; framing and the meaning of frames
// live in core/protocol.
#include <stdint.h>
#include "movement/joint_calibration.h"
#include "protocol/link_receiver.h"
#include "protocol/protocol.h"

namespace link
{

void setup();

/**
 * @brief Drains received bytes, decodes them and returns this tick's input
 * (from the newest valid command). Calibration sets are queued for
 * processCalibration(). Bounded by the receive buffer size.
 */
link_receiver::Input poll(uint32_t nowMs);

/**
 * @brief Runs this link's queued calibration sets through the guards into
 * @p requested (see link_receiver::processCalibration).
 * @return the number of sets processed
 */
int processCalibration(joint_calibration::Table &requested, bool calibrating, bool allowOutput);

/**
 * @brief Queue a frame without blocking, whole or not at all: dropped if the
 * 256-byte transmit queue has no room for all of it.
 * @return whether the frame was queued
 */
bool sendTelemetry(const protocol::Telemetry &t);
bool sendCalibrationReport(const joint_calibration::Table &table);

uint32_t validFrames();
uint32_t discardedFrames();
uint32_t droppedBytes(); // receive buffer overflows and UART errors

} // namespace link

#endif // LINK_H
