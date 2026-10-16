#include "supervisor.h"

namespace protection
{

Decision supervise(Supervisor &sv, const Result &prot, RequestedMode requested, uint32_t requestSeq)
{
  Decision d = {};
  d.mode = resumeGate(sv.gate, prot.newlyLatched != 0, prot.forceSleep, requested, requestSeq);

  // allowOutput is false only before the supply first settles or while an
  // inhibiting fault is latched; either way the body must not run.
  if (sv.bodyRunning && !prot.allowOutput)
  {
    sv.bodyRunning = false;
    d.stopBody = true;
  }
  // Starting the body drives the servos (Initialise), so only a request to
  // move may start it: a Sleep request leaves a stopped body stopped.
  if (!sv.bodyRunning && prot.allowOutput && !sv.gate.waiting && d.mode != RequestedMode::Sleep)
  {
    sv.bodyRunning = true;
    d.startBody = true;
  }
  d.runBody = sv.bodyRunning;
  return d;
}

} // namespace protection
