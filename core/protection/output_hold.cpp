#include "output_hold.h"

namespace protection
{

bool outputAllowed(OutputHold &h, bool inhibited, bool bodyWants, uint32_t wakeCount)
{
  if (inhibited)
  {
    h.held = true;
    h.wakeCountAtInhibit = wakeCount;
    return false;
  }
  if (h.held && wakeCount != h.wakeCountAtInhibit)
    h.held = false;
  return !h.held && bodyWants;
}

} // namespace protection
