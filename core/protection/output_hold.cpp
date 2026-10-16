#include "output_hold.h"

namespace protection
{

bool outputAllowed(OutputHold &h, bool inhibited, bool bodyWants)
{
  bool wake = bodyWants && !h.lastBodyWants;
  h.lastBodyWants = bodyWants;

  if (inhibited)
  {
    h.held = true;
    return false;
  }
  if (h.held && wake)
    h.held = false;
  return !h.held && bodyWants;
}

} // namespace protection
