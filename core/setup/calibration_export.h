#ifndef CALIBRATION_EXPORT_H
#define CALIBRATION_EXPORT_H

// A reported calibration table as the complete text of
// core/movement/calibration_defaults.cpp, in the fixed format pinned by
// test/golden/calibration_defaults.cpp: the portal's Export, to paste into
// the robot's source.
// Hardware-free (snprintf only): compiled for the bridge and for host tests.
#include <stdint.h>
#include "protocol/protocol.h"

namespace calibration_export
{

// The defaults file is about 1.4 KB; this leaves ample room.
constexpr int MAX_TEXT_SIZE = 2048;

/**
 * @brief Writes the defaults file for @p joints (the reported table, in
 * servo output order) into @p out, NUL-terminated.
 * @return the text length, or -1 if @p size is too small (out is then empty)
 */
int write(const protocol::CalibrationJoint joints[protocol::CALIBRATION_JOINTS], char *out, int size);

} // namespace calibration_export

#endif // CALIBRATION_EXPORT_H
