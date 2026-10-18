#ifndef USB_LINK_H
#define USB_LINK_H

// Link frames over the USB serial port, sharing it with the debug text that
// printf writes. Frames and text are both written from the core 0 loop, so
// neither is ever written inside the other. Not compiled in --serial builds,
// where the USB serial keys own the port's input.
#include <stdint.h>
#include "movement/joint_calibration.h"
#include "protocol/link_receiver.h"
#include "protocol/protocol.h"

namespace usb_link
{

// Received bytes decoded per poll, at most; the rest wait in the USB stack.
constexpr int MAX_BYTES_PER_POLL = 256;

/**
 * @brief Reads pending bytes without blocking, decodes them and returns this
 * tick's input (from the newest valid command). Calibration sets are queued
 * for processCalibration().
 */
link_receiver::Input poll(uint32_t nowMs);

/**
 * @brief Runs this link's queued calibration sets through the guards into
 * @p requested (see link_receiver::processCalibration).
 * @return the number of sets processed
 */
int processCalibration(joint_calibration::Table &requested, bool calibrating, bool allowOutput);

/**
 * @brief Writes a frame if a host is connected and the USB transmit buffer
 * has room for all of it; otherwise drops it whole. Never waits.
 * @return whether the frame was written
 */
bool sendTelemetry(const protocol::Telemetry &t);
bool sendCalibrationReport(const joint_calibration::Table &table);
// USB only: the bridge link (link.h) has no pose sender.
bool sendPose(const protocol::Pose &p);

uint32_t validFrames();
uint32_t discardedFrames();

} // namespace usb_link

#endif // USB_LINK_H
