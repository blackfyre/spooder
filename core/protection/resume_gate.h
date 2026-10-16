#ifndef RESUME_GATE_H
#define RESUME_GATE_H

// After a protection fault, the body stays asleep until the operator makes a
// new movement request: clearing the fault alone never restarts motion, even
// if the input source keeps repeating its last request.
// Hardware-free: compiled for the board and for host tests.
#include <stdint.h>
#include "movement/movement_command.h"

namespace protection
{

struct ResumeGate
{
  bool waiting;          // a fault latched since the last fresh request
  uint32_t requestSeqAtLatch;
};

/**
 * @brief Returns the mode movement should be given.
 *
 * @param faultLatched a fault latched on this step
 * @param faultActive  any fault is currently latched
 * @param requested    the input's requested mode
 * @param requestSeq   counter the input source increments on every explicit
 *                     mode request (key press, button press)
 */
RequestedMode resumeGate(ResumeGate &g, bool faultLatched, bool faultActive, RequestedMode requested,
                         uint32_t requestSeq);

} // namespace protection

#endif // RESUME_GATE_H
