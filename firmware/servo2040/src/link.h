#ifndef LINK_H
#define LINK_H

// The gamepad bridge link on UART1: GPIO20 TX, GPIO21 RX (J6 pins 14 and 13),
// 115200 8N1. Bytes move under interrupts; framing, decoding and the meaning
// of frames live in core (protocol, link_receiver, robot/robot_tick).
#include <stdint.h>

namespace link
{

// Receive ring size: read() returns at most this many bytes.
constexpr int RX_BUFFER_SIZE = 256;

void setup();

/**
 * @brief Moves the bytes received since the last call into @p out, at most
 * @p max, in arrival order. Never blocks.
 * @return the number of bytes
 */
int read(uint8_t *out, int max);

/**
 * @brief Queues a frame without blocking, whole or not at all: dropped if the
 * 256-byte transmit queue has no room for all of it.
 * @return whether the frame was queued
 */
bool sendFrame(const uint8_t *frame, int length);

uint32_t droppedBytes(); // receive buffer overflows and UART errors

} // namespace link

#endif // LINK_H
