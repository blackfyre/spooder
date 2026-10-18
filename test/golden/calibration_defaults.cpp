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
  {-0.50, 2450, 610, true}, // leg 1 coxa
  {0.00, 500, 2500, false}, // leg 1 femur
  {-29.99, 2500, 500, false}, // leg 1 tibia
  {76.50, 612, 2390, true}, // leg 2 coxa
  {105.25, 400, 2600, false}, // leg 2 femur
  {-18.07, 1000, 2000, false}, // leg 2 tibia
  {90.01, 520, 2480, false}, // leg 3 coxa
  {-120.40, 2600, 1600, true}, // leg 3 femur
  {7.00, 1400, 400, false}, // leg 3 tibia
  {87.33, 505, 2495, false}, // leg 4 coxa
  {80.10, 498, 2502, true}, // leg 4 femur
  {-26.00, 700, 2300, false}, // leg 4 tibia
  {85.99, 1500, 2500, false}, // leg 5 coxa
  {-0.01, 500, 1500, false}, // leg 5 femur
  {-16.50, 2400, 600, true}, // leg 5 tibia
  {179.99, 450, 2550, false}, // leg 6 coxa
  {-180.00, 2550, 450, true}, // leg 6 femur
  {12.34, 640, 2360, false}, // leg 6 tibia
};

} // namespace joint_calibration
