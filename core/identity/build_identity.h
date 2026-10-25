#ifndef BUILD_IDENTITY_H
#define BUILD_IDENTITY_H

// The firmware build's identity (`git describe --always --dirty --abbrev=12`
// at build time, or `unknown`) and the text that shows it: the console
// banner and the screen's fitted form. The text is always printable ASCII,
// so it can never put a frame byte (0xA5) or a control character on the
// robot's USB port.
// Hardware-free: host tests drive the functions with literal ids.

namespace build_identity
{

// Defined only by the source cmake/build_identity.cmake generates in each
// firmware build (robot, virtual robot, bridge); the host tests do not link it.
extern const char ID[];

constexpr int MAX_ID_CHARS = 40; // the banner cuts a longer identity here
// Room for the longest banner ("spooder bridge: build " + 40 + "\n") and NUL.
constexpr int BANNER_SIZE = 64;

/**
 * @brief Writes `spooder <component>: build <id>\n` to @p out (always
 * NUL-terminated within @p size). A null or empty @p id is `unknown`; any byte
 * of @p id outside printable ASCII (0x20 to 0x7E) becomes `?`, and @p id is
 * cut at MAX_ID_CHARS. A @p size too small cuts the line.
 * @return the characters written, without the NUL.
 */
int banner(char *out, int size, const char *component, const char *id);

/**
 * @brief Writes @p id fitted to @p width characters to @p out (which holds
 * @p width + 1): as is when it fits, otherwise `..` followed by its end, so the
 * abbreviated hash and any `-dirty` stay visible. The same rules as the
 * banner apply (null or empty is `unknown`, non-printable bytes are `?`), but
 * not the MAX_ID_CHARS cut.
 */
void fit(char *out, int width, const char *id);

} // namespace build_identity

#endif // BUILD_IDENTITY_H
