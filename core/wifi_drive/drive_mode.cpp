#include "drive_mode.h"

namespace drive_mode
{

Mode select(bool setupRequested, bool switchClosed)
{
  if (setupRequested)
    return Mode::Setup;
  return switchClosed ? Mode::Wifi : Mode::Bluetooth;
}

bool update(Notice &n, Mode running, bool switchClosed, uint32_t nowMs)
{
  if (running == Mode::Setup)
  {
    n = Notice();
    return false;
  }
  bool moved = switchClosed != (running == Mode::Wifi);
  if (moved == n.shown)
  {
    n.pending = false; // agrees with what is shown: any change was bounce
    return n.shown;
  }
  if (!n.pending)
  {
    n.pending = true;
    n.pendingSinceMs = nowMs;
  }
  if (nowMs - n.pendingSinceMs >= NOTICE_MS)
  {
    n.shown = moved;
    n.pending = false;
  }
  return n.shown;
}

} // namespace drive_mode
