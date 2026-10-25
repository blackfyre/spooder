#ifndef BRIDGE_LOVYANGFX_INCLUDE_H
#define BRIDGE_LOVYANGFX_INCLUDE_H

// LovyanGFX, included for Spooder's sources. The main component builds with
// -Wall -Wextra -Werror; LovyanGFX's panel headers (Panel_AMOLED.hpp,
// Panel_FlexibleFrameBuffer.hpp, Panel_FrameBufferBase.hpp, Panel_RA8875.hpp)
// name parameters their inline overrides do not use, which -Wextra reports as
// -Wunused-parameter. That warning is ignored for this include only, so
// Spooder's own code stays under the full set.
#define LGFX_USE_V1
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <LovyanGFX.hpp>
#pragma GCC diagnostic pop

#endif
