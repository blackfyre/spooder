#ifndef PAGE_BUTTON_H
#define PAGE_BUTTON_H

// The bridge's left button (GPIO0) as the status screen's page switch, once
// the start-up sequence is over: a press counts on release, after being down
// for MIN_PRESS_MS to under MAX_PRESS_MS, and shows the next page. Sampling
// starts with the status screen; a press already down then (begun during the
// start-up sequence) never counts: the button arms once seen released.
// Hardware-free: the bridge samples the button; host tests drive it.
#include <stdint.h>

namespace page_button
{

constexpr uint32_t MIN_PRESS_MS = 30; // shorter: contact bounce
constexpr uint32_t MAX_PRESS_MS = 1000;

enum class Page : uint8_t
{
  Drive,
  Power,
  Link,
};
constexpr int PAGE_COUNT = 3;

struct State
{
  Page page = Page::Drive;
  bool armed = false; // seen released since sampling started
  bool down = false;
  uint32_t sinceMs = 0;
};

/**
 * @brief Advances by one button sample at @p nowMs.
 * @return true when the page changed
 */
bool update(State &s, bool pressed, uint32_t nowMs);

} // namespace page_button

#endif // PAGE_BUTTON_H
