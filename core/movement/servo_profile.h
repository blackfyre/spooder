#ifndef SERVO_PROFILE_H
#define SERVO_PROFILE_H

// Servo models the legs can be built with, and the one this build is for.
// The joint speed budget limits how fast movement may turn any joint.
// Hardware-free: compiled for the board, the virtual robot and host tests.

namespace servo_profile
{

struct ServoProfile
{
  const char *name;
  float noLoadDegPerS; // rated speed without load at the nominal supply
  float loadShare;     // share of that speed movement may use under load

  constexpr float budgetDegPerS() const { return noLoadDegPerS * loadShare; }
};

// Keep UNKNOWN_PROFILE_MESSAGE below in step with this table.
inline constexpr ServoProfile PROFILES[] = {
    // MG996R: 0.14 s/60° at 6 V. The load share is an estimate; tune it on
    // the bench (current, foot lift) and record the result here.
    {"MG996R", 60.0f / 0.14f, 0.6f},
};
constexpr int PROFILE_COUNT = sizeof(PROFILES) / sizeof(PROFILES[0]);

constexpr bool sameName(const char *a, const char *b)
{
  return *a == *b && (*a == '\0' || sameName(a + 1, b + 1));
}

/**
 * @brief The index of the profile called @p name, or -1 for an unknown name.
 */
constexpr int indexOf(const char *name, int from = 0)
{
  return from == PROFILE_COUNT ? -1 : sameName(PROFILES[from].name, name) ? from : indexOf(name, from + 1);
}

// The build chooses the profile by name (SPOODER_SERVO_PROFILE, a string).
#ifndef SPOODER_SERVO_PROFILE
#define SPOODER_SERVO_PROFILE "MG996R"
#endif

static_assert(indexOf(SPOODER_SERVO_PROFILE) >= 0, "unknown SPOODER_SERVO_PROFILE; known profiles: MG996R");

inline constexpr const ServoProfile &SELECTED = PROFILES[indexOf(SPOODER_SERVO_PROFILE)];

} // namespace servo_profile

#endif // SERVO_PROFILE_H
