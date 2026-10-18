// Spooder joint calibration defaults, in a fixed format: an export from a
// calibration front-end replaces this whole file. Do not reformat it.
//
// One line per joint, in servo output order (leg 1 coxa, femur, tibia, then
// leg 2 ...):
//   {offset in degrees (two decimals), pulse at 0 degrees in us,
//    pulse at 180 degrees in us, reversed},
// Its format is pinned by a fixture, test/golden/calibration_defaults.cpp.
#include "joint_calibration.h"

namespace joint_calibration
{

const joint_calibration::Joint DEFAULTS[18] = {
  {90.00, 500, 2500, false}, // leg 1 coxa
  {75.00, 500, 2500, false}, // leg 1 femur
  {-18.00, 500, 2500, false}, // leg 1 tibia
  {93.00, 500, 2500, false}, // leg 2 coxa
  {75.00, 500, 2500, false}, // leg 2 femur
  {-15.00, 500, 2500, false}, // leg 2 tibia
  {93.00, 500, 2500, false}, // leg 3 coxa
  {75.00, 500, 2500, false}, // leg 3 femur
  {-18.00, 500, 2500, false}, // leg 3 tibia
  {87.00, 500, 2500, false}, // leg 4 coxa
  {80.00, 500, 2500, false}, // leg 4 femur
  {-26.00, 500, 2500, false}, // leg 4 tibia
  {85.00, 500, 2500, false}, // leg 5 coxa
  {89.00, 500, 2500, false}, // leg 5 femur
  {-16.00, 500, 2500, false}, // leg 5 tibia
  {93.00, 500, 2500, false}, // leg 6 coxa
  {85.00, 500, 2500, false}, // leg 6 femur
  {-24.00, 500, 2500, false}, // leg 6 tibia
};

} // namespace joint_calibration
