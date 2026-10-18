#ifndef USB_LINK_H
#define USB_LINK_H

// Link frames over the USB serial port, sharing it with the debug text that
// printf writes. Frames and text are both written from the core 0 loop, so
// neither is ever written inside the other. Not compiled in --serial builds,
// where the USB serial keys own the port's input. Decoding lives in core
// (robot/robot_tick); this adapter only moves bytes.
#include <stdint.h>

namespace usb_link
{

// Received bytes read per tick, at most; the rest wait in the USB stack.
constexpr int MAX_BYTES_PER_POLL = 256;

/**
 * @brief Reads pending bytes without blocking, at most @p max.
 * @return the number of bytes
 */
int read(uint8_t *out, int max);

/**
 * @brief Writes a frame if a host is connected and the USB transmit buffer
 * has room for all of it; otherwise drops it whole. Never waits.
 * @return whether the frame was written
 */
bool sendFrame(const uint8_t *frame, int length);

} // namespace usb_link

#endif // USB_LINK_H
