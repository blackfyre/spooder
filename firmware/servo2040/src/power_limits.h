// Power-protection limits for Spooder: 18 × MG996R (180°) servos on a ~6.0 V
// regulated supply (a BEC fed from a 2S LiPo; during bring-up a lab PSU set to
// 6.0 V).
//
// The board senses the BEC output, not the battery: these limits protect the
// servos and the board, not the battery cells. Keep a separate low-voltage
// cut-off on the pack until battery sensing exists.
//
// MG996R: rated 4.8–7.2 V; about 0.5–0.9 A each moving freely and about 2.5 A at
// stall (6 V). 18 of them can exceed the Servo 2040 screw terminals' 10 A
// continuous rating, so the current limits stay below it. Revisit the
// sustained limit with measurements from bring-up.
#pragma once

#define SPOODER_TRIP_CURRENT_A 9.0f        // above this for the persistence time: output off (terminals rated 10 A)
#define SPOODER_TRIP_PERSIST_MS 60u        // above the trip current this long: output off (the stand surge read 11.8 A once)
#define SPOODER_CEILING_CURRENT_A 13.0f    // instant: output off
#define SPOODER_SUSTAINED_CURRENT_A 6.0f   // 2 s average: Sleep; tune from bring-up measurements
#define SPOODER_SUSTAINED_WINDOW_MS 2000u
#define SPOODER_MIN_V 5.2f                 // below this for the debounce time: Sleep
#define SPOODER_UNDERVOLT_DEBOUNCE_MS 500u // rides out sag from several servos starting together
#define SPOODER_BROWNOUT_V 4.8f            // instant: output off (MG996R minimum)
#define SPOODER_MAX_V 6.8f                 // instant: output off (below the MG996R 7.2 V maximum)
