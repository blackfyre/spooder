#ifndef LINK_H
#define LINK_H

// The gamepad bridge link on UART1: GPIO20 TX, GPIO21 RX (J6 pins 14 and 13),
// 115200 8N1. Bytes move under interrupts; framing and the meaning of frames
// live in core/protocol.
#include <stdint.h>
#include "protocol/link_receiver.h"
#include "protocol/protocol.h"

namespace link
{

void setup();

/**
 * @brief Drains received bytes, decodes them and returns this tick's input
 * (from the newest valid command). Bounded by the receive buffer size.
 */
link_receiver::Input poll(uint32_t nowMs);

/**
 * @brief Queues a telemetry frame without blocking. Skipped if the previous
 * frame has not finished sending.
 * @return whether the frame was queued
 */
bool sendTelemetry(const protocol::Telemetry &t);

uint32_t validFrames();
uint32_t discardedFrames();
uint32_t droppedBytes(); // receive buffer overflows and UART errors

} // namespace link

#endif // LINK_H
