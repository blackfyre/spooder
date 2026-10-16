#include "resume_gate.h"

namespace protection
{

RequestedMode resumeGate(ResumeGate &g, bool faultLatched, bool faultActive, RequestedMode requested,
                         uint32_t requestSeq)
{
  if (faultLatched)
  {
    g.waiting = true;
    g.requestSeqAtLatch = requestSeq;
  }
  if (faultActive)
    return RequestedMode::Sleep;
  if (g.waiting)
  {
    if (requestSeq == g.requestSeqAtLatch)
      return RequestedMode::Sleep;
    g.waiting = false; // a request made after the fault
  }
  return requested;
}

} // namespace protection
