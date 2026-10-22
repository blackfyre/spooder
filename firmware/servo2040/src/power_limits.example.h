// Power-protection limits: starting point for a regulated 6.0 V servo supply.
//
// Copy to power_limits.h (next to this file) and adjust for the servos and
// supply actually fitted, or point SPOODER_POWER_LIMITS at your own file.
// The firmware does not build without a limits file.
//
// Board limits: screw terminals 10 A continuous; the current sense reads up
// to about 15.9 A; the voltage sense reads up to about 11.8 V.
#pragma once

#define SPOODER_TRIP_CURRENT_A 9.0f        // above this for the persistence time: output off
#define SPOODER_TRIP_PERSIST_MS 60u        // rides out the surge of all servos starting; at most 200 ms
#define SPOODER_CEILING_CURRENT_A 13.0f    // instant: output off; above the trip current, at most 15 A
#define SPOODER_SUSTAINED_CURRENT_A 6.0f   // average over the window: Sleep
#define SPOODER_SUSTAINED_WINDOW_MS 2000u  // multiple of 50 ms, at most 10 s
#define SPOODER_MIN_V 5.2f                 // below this for the debounce time: Sleep
#define SPOODER_UNDERVOLT_DEBOUNCE_MS 200u
#define SPOODER_BROWNOUT_V 4.8f            // instant: output off
#define SPOODER_MAX_V 6.8f                 // instant: output off; keep within the servos' rating
