#include "movement.h"

#include <Arduino.h>
#include "body.h"
#include "servos.h"
#include "../reporting/reporting.h"

static body::BodyState bodyState;

void setupMovement()
{
  servos::setup();
  body::init(bodyState);
}

void movementTick(uint32_t nowMs, const MovementCommand &command)
{
  body::step(bodyState, command, nowMs);

  for (int leg = 0; leg < kinematics::LEG_COUNT; leg++)
  {
    if (bodyState.newlyRejected[leg])
    {
      print_value("Point impossible to reach", bodyState.rejectedTarget[leg], false);
      Serial.print(" leg: ");
      Serial.println(leg);
    }
    if (bodyState.legUpdated[leg])
      servos::writeLeg(leg, bodyState.angles[leg]);
  }

  // Sleep switches output off once the legs are folded. Writes above re-enable
  // it implicitly when the body wakes.
  if (!bodyState.outputsEnabled && servos::outputsEnabled())
    servos::setOutputsEnabled(false);
}
