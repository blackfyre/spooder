#ifndef OUTPUT_HOLD_H
#define OUTPUT_HOLD_H

// Once servo output has been inhibited by protection, it may only come back
// on after the body has woken from Sleep. Clearing a fault therefore never
// resumes output in the middle of whatever the body was doing while output
// was off.
// Hardware-free: compiled for the board and for host tests.
#include <stdint.h>

namespace protection
{

struct OutputHold
{
  bool held = false;          // inhibited, and the body has not woken since
  uint32_t wakeCountAtInhibit = 0;
};

/**
 * @brief Whether servo output may be on this step.
 *
 * @param inhibited protection currently inhibits output
 * @param bodyWants the body wants output on (body::BodyState::outputsEnabled)
 * @param wakeCount body::BodyState::wakeCount; any change since the inhibit
 *        means the body has woken, even within a single step
 */
bool outputAllowed(OutputHold &h, bool inhibited, bool bodyWants, uint32_t wakeCount);

} // namespace protection

#endif // OUTPUT_HOLD_H
