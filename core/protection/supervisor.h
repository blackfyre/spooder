#ifndef SUPERVISOR_H
#define SUPERVISOR_H

// Decides each tick whether the body runs and which mode it is given, from
// protection's result and the operator's input.
//
// - A fault that inhibits output stops the body. Recovery is a restart through
//   the boot sequence (Initialise, then Stand), exactly like power-up.
// - The body (re)starts only when output is allowed and no fault is waiting
//   for a new operator request (see resume_gate).
// - Faults that only force Sleep keep the body running; it folds and wakes
//   through the normal Sleep path once a new request arrives.
// Hardware-free: compiled for the board and for host tests.
#include <stdint.h>
#include "movement/movement_command.h"
#include "protection/protection.h"
#include "protection/resume_gate.h"

namespace protection
{

struct Supervisor
{
  ResumeGate gate = {};
  bool bodyRunning = false;
};

struct Decision
{
  bool startBody;     // initialise the body (boot sequence) this tick
  bool stopBody;      // the body stopped this tick: switch servo output off
  bool runBody;       // step the body this tick
  RequestedMode mode; // mode to give the body
};

Decision supervise(Supervisor &sv, const Result &prot, RequestedMode requested, uint32_t requestSeq);

} // namespace protection

#endif // SUPERVISOR_H
