#include "usb_link.h"

#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "protocol/protocol.h"
#include "tusb.h"

namespace usb_link
{

// A whole report must fit the USB transmit buffer (CFG_TUD_CDC_TX_BUFSIZE,
// raised in CMakeLists.txt), or it could never be written whole.
static_assert(CFG_TUD_CDC_TX_BUFSIZE >= protocol::MAX_FRAME_SIZE, "a whole frame fits the USB transmit buffer");

int read(uint8_t *out, int max)
{
  int n = 0;
  while (n < max)
  {
    int c = stdio_getchar_timeout_us(0);
    if (c < 0)
      break; // PICO_ERROR_TIMEOUT: nothing pending
    out[n++] = (uint8_t)c;
  }
  return n;
}

// Writes the frame whole or not at all. stdio's own write would wait up to
// PICO_STDIO_USB_STDOUT_TIMEOUT_US for room and could then stop part-way.
bool sendFrame(const uint8_t *frame, int n)
{
  if (!stdio_usb_connected() || tud_cdc_write_available() < (uint32_t)n)
    return false;
  // No newline, no CR translation: the bytes go out as they are.
  stdio_put_string((const char *)frame, n, false, false);
  return true;
}

} // namespace usb_link
