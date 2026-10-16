#ifndef CONFIG_H
#define CONFIG_H

// Build-time configuration for the Servo 2040 firmware.
#include "protection/protection.h"

// Power limits must be chosen for the servos and supply fitted
// (see power_limits.example.h). CMake passes the chosen file in.
#ifdef SPOODER_POWER_LIMITS_HEADER
#include SPOODER_POWER_LIMITS_HEADER
#endif

#if !defined(SPOODER_TRIP_CURRENT_A) || !defined(SPOODER_SUSTAINED_CURRENT_A) ||       \
    !defined(SPOODER_SUSTAINED_WINDOW_MS) || !defined(SPOODER_MIN_V) ||                 \
    !defined(SPOODER_UNDERVOLT_DEBOUNCE_MS) || !defined(SPOODER_BROWNOUT_V) ||          \
    !defined(SPOODER_MAX_V)
#error "configure power limits for your servos and supply: copy src/power_limits.example.h to src/power_limits.h"
#endif

namespace config
{

constexpr protection::Limits POWER_LIMITS = {
    SPOODER_TRIP_CURRENT_A,
    SPOODER_SUSTAINED_CURRENT_A,
    SPOODER_SUSTAINED_WINDOW_MS,
    SPOODER_MIN_V,
    SPOODER_UNDERVOLT_DEBOUNCE_MS,
    SPOODER_BROWNOUT_V,
    SPOODER_MAX_V,
};

static_assert(SPOODER_SUSTAINED_CURRENT_A < SPOODER_TRIP_CURRENT_A, "sustained limit must be below the trip current");
static_assert(SPOODER_TRIP_CURRENT_A <= 15.0f, "trip current beyond the measurable range");
static_assert(SPOODER_BROWNOUT_V < SPOODER_MIN_V && SPOODER_MIN_V < SPOODER_MAX_V, "voltage limits out of order");
static_assert(SPOODER_MAX_V <= 11.0f, "the board supports servo supplies up to 11 V");
static_assert(SPOODER_SUSTAINED_WINDOW_MS % protection::BUCKET_MS == 0 &&
                  SPOODER_SUSTAINED_WINDOW_MS / protection::BUCKET_MS <= (unsigned)protection::MAX_BUCKETS,
              "sustained window must be a multiple of 50 ms, at most 10 s");

} // namespace config

#endif // CONFIG_H
