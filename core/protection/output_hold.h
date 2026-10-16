#ifndef OUTPUT_HOLD_H
#define OUTPUT_HOLD_H

// Once servo output has been inhibited by protection, it may only come back
// on at a wake: the body must first stop asking for output (Sleep folded)
// and then ask again. Clearing a fault therefore never resumes output in the
// middle of whatever the body was doing while output was off.
// Hardware-free: compiled for the board and for host tests.

namespace protection
{

struct OutputHold
{
  bool held = false;          // inhibited since the last wake
  bool lastBodyWants = false; // body.outputsEnabled at the previous step
};

/**
 * @brief Whether servo output may be on this step.
 *
 * @param inhibited protection currently inhibits output
 * @param bodyWants the body wants output on (body::BodyState::outputsEnabled)
 */
bool outputAllowed(OutputHold &h, bool inhibited, bool bodyWants);

} // namespace protection

#endif // OUTPUT_HOLD_H
