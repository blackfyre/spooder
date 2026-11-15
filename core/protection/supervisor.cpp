#include "supervisor.h"

namespace protection
{

Decision supervise(Supervisor &sv, const Result &prot, RequestedMode requested, uint32_t requestSeq)
{
  Decision d = {};
  d.mode = resumeGate(sv.gate, prot.newlyLatched != 0, prot.forceSleep, requested, requestSeq);

  // allowOutput is false only before the supply first settles or while an
  // inhibiting fault is latched; either way the body must not run.
  if (sv.runState == RunState::Running && !prot.allowOutput)
  {
    sv.runState = RunState::Stopped;
    d.stopBody = true;
  }
  // Starting the body drives the servos (Initialise), so only a request to
  // move may start it: a Sleep request leaves a stopped body stopped.
  if (sv.runState != RunState::Running && prot.allowOutput && !sv.gate.waiting && d.mode != RequestedMode::Sleep)
  {
    sv.runState = RunState::Running;
    d.startBody = true;
  }
  d.runBody = sv.runState == RunState::Running;
  return d;
}

} // namespace protection
