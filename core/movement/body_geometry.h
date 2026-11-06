#ifndef BODY_GEOMETRY_H
#define BODY_GEOMETRY_H

// Where each leg is mounted on the body (add-gestures design D3).
// Hardware-free; the desktop's legview has a copy, pinned by
// test/golden/body_geometry.txt.
//
// Body frame: x right, y forward, z up, millimetres, origin at the frame's
// centre, the coxa pivots at z = 0. A leg's mount angle is gait's
// (gait::mountAngleDeg): the pivots lie on those angles from the centre.
#include "kinematics.h"
#include "vectors.h"

namespace body_geometry
{

// Coxa pivots, in servo output order (legs 0-2 on the left, front to rear;
// 3-5 on the right, rear to front). Measured from upstream's frame STLs
// (hardware/aecert-hexapod-v1): each leg's Bearing Axle Mount screws to a
// pair of frame holes 28.5 mm apart, square to the leg, and holds the coxa
// axle centred between them, 9.90 mm outward (its far edge is flush with the
// end of the frame's arm). The pairs lie 87.60 mm (middle legs) and 98.13 mm
// (corner legs) from the centre, so the pivots lie 97.50 and 108.03 mm out.
const Vector3 PIVOTS[kinematics::LEG_COUNT] = {
    Vector3(-60.41, 89.56, 0), Vector3(-97.50, 0, 0), Vector3(-60.41, -89.56, 0),
    Vector3(60.41, -89.56, 0), Vector3(97.50, 0, 0),  Vector3(60.41, 89.56, 0),
};

// The body's underside: the height of its lowest point below the pivots, the
// first thing to touch the ground as the level body is lowered (add-gestures
// design D3). Measured from upstream's STLs (hardware/aecert-hexapod-v1):
// the pivots' z = 0 is the femur servo's shaft, which the coxa side plates
// (Coxa Horn Side, Coxa Bearing Side) hold 9.9 mm below the middle of their
// 28.5 mm hole pair; the plates reach 34.125 mm either side of that middle,
// and their lower ends lie flush with the underside of the Coxa Horn
// Connector, the coxa link under each leg's pivot. So that underside lies
// 34.125 - 9.9 = 24.2 mm below the pivots, under every coxa angle, and
// touches first: Frame Bottom's underside lies 15.75 mm below them.
constexpr float UNDERSIDE_Z = -24.2f;

} // namespace body_geometry

#endif // BODY_GEOMETRY_H
