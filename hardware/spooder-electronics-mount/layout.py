"""The electronics mount's layout (task group 3 of add-electronics-mount).

Where each held part sits, the deck that carries them and holds them, the
spacers under it, the reserved cable routes, the deck's tie holes and slots,
and the sensor provisions. Everything is in the frame top's coordinates (mm,
+y forward, the ring's top at z 25.75), as model.py describes.

The layout follows the wiring (design.md, "Layout follows the wiring"):

- the SZBK07 lies across the rear, its input end (-x) towards the XT60 and
  the fuse on the left, its output end (+x) towards the right-hand corridor
  that runs forward to the Servo 2040's EXT terminal;
- the Servo 2040 lies in front of it, unturned: its EXT terminal and sensor
  header face the corridor on the right, J6 faces the band between it and the
  SZBK07, its servo header faces the lead slots at the front, and its USB-C
  port faces the left, where a cable has room. It stands on standoffs tall
  enough for the IMU breakout under it, near the body's centre;
- the LM2596 lies on the right behind the corridor, with the T-Display above
  it on a holder (task 5.2), so the screen shows through the lid beside the
  SZBK07 rather than over it;
- the XT60 faces out of the left wall, recessed so that the battery's plug in
  it stays inside the legs' keep-clear region;
- the cover (task group 4) stands on the deck's rim: an intake fan position
  low behind a grille in the left wall, an exhaust position high behind a
  grille in the front-right corner face, the air channel between them over
  the SZBK07, the window over the T-Display, and three screws down through
  the lid into posts on the deck.

Sizes come from measurements.py; the numbers here are layout choices, each
with its reason.
"""

from __future__ import annotations

import math

from build123d import Axis, Location, Plane, Polygon, Solid, extrude

import measurements as M
from model import (FANS, IMU, J6, LM2596_OUTPUT_ROUTE, SCHOTTKY, SENSOR_HEADER, SERVO_2040, SIDES, SZBK07, AirChannel,
                   Assembly, Box6, Exposure, Fan, Floating, Held, ImuPad, Opening, Part, Route, Screw, Support, Terminal, Vent, box,
                   box6, teardrop, turned)

UP = (0.0, 0.0, 1.0)
DOWN = (0.0, 0.0, -1.0)

# --------------------------------------------------------------------------- the box

# Contact gap between faces that touch, so booleans leave no slivers. Source:
# the spooder-printed-parts skill ("Coplanar contacts").
TOUCH = 0.05
# Clearance round a held part, per side. Source: the skill's FIT.
FIT = 0.3
# The cover's wall. Source: the skill ("Walls").
WALL = 2.4
# The deck's and the cover's outer footprint: |x| <= BOX_X, |y| <= BOX_Y and
# |x| + |y| <= BOX_DIAG (45-degree corner faces, design.md "Shape"). The
# corners stay within r 71.6, which leaves the bumpers (task 5.1) about 7 mm
# inside the keep-clear radius of 78.8; the side walls reach out where the
# circle is widest, so the strips beside the Servo 2040 take the parts that
# face its edges. Source: chosen.
BOX_X = 65.0
BOX_Y = 62.0
BOX_DIAG = 95.0
# The space inside the cover's walls, which every held part lies in.
INNER = WALL + FIT
INNER_X = BOX_X - INNER
INNER_Y = BOX_Y - INNER
INNER_DIAG = BOX_DIAG - INNER * math.sqrt(2)

# The deck: a plate on the frame top's ring, its underside just clear of the
# ring's top. Source: measurements (FRAME_RING_Z), TOUCH.
DECK_Z0 = M.FRAME_RING_Z + TOUCH
DECK_T = 3.0  # Source: chosen, stiff enough to span the battery bay.
DECK_Z1 = DECK_Z0 + DECK_T
# Tabs forward and back to the ring's four inserts at y +-73, inside the
# keep-clear tabs (|x| <= 24). Source: measurements (FRAME_RING_INSERTS), the
# spec's "Fit to the frame and the legs".
TAB_X = 23.5
TAB_Y = 76.5
# M3 clearance through the deck at each insert, counterbored so a button head
# sits below the deck's top. Source: the skill ("Fits and fasteners").
M3_D = 3.4
CBORE_D = 6.4
CBORE_DEPTH = 2.0
BUTTON_HEAD_H = 1.65  # Source: ISO 7380 M3 button head, 1.65 tall.
# Features on the deck sink this far into the plate, so the deck is one solid.
# Source: the skill ("Each part is one solid").
EMBED = 0.5

# Spacers under the deck at the four floor inserts, from the floor (z 20) to
# the deck's underside. Printed separately: as posts on the deck they would
# hang in mid-air when the deck prints flat. Source: chosen.
SPACER_OD = 7.0
SPACER_ID = M3_D
SPACER_Z0 = M.FRAME_FLOOR_Z
SPACER_Z1 = DECK_Z0 - TOUCH

# Pilot holes for self-tapping screws into PETG. Source: the skill (M2.5
# pilot 2.2); M2 by the same ratio.
PILOT_M25_D = 2.2
PILOT_M2_D = 1.6
# Round a board's mounting hole its parts leave room for a screw head and a
# screwdriver, and its board is this thick there. Source: chosen, a Ø4
# screwdriver (the spec's "Access and service") with fit; a 1.6 mm board.
SCREW_CLEAR_R = 2.3
MODULE_PCB_T = 1.6
# Cable-tie holes: 3.4 along the tie, 2.0 across. Source: the skill (TIE_HOLE).
TIE_L = 3.4
TIE_W = 2.0

# --------------------------------------------------------------------------- held parts

# SZBK07 across the rear: 60 along x, 54 along y, short ends at -x (input)
# and +x (output). Its rear corners clear the inner corner faces. Source:
# chosen.
SZ_X0 = -28.0
SZ_Y0 = -55.5
SZ_BOX = (SZ_X0, SZ_X0 + M.SZBK07_SIZE[0], SZ_Y0, SZ_Y0 + M.SZBK07_SIZE[1],
          DECK_Z1 + TOUCH, DECK_Z1 + TOUCH + M.SZBK07_SIZE[2])
SZ_MID_Y = (SZ_BOX[2] + SZ_BOX[3]) / 2
SZ_TERM = (SZ_MID_Y - M.SZBK07_TERMINAL_SPAN / 2, SZ_MID_Y + M.SZBK07_TERMINAL_SPAN / 2)
SZ_TERM_DEPTH = 4.0  # Source: chosen, a screw terminal block's depth.
# Its stops: L-shaped at the rear corners, ribs at the front corners, low
# enough for wires to pass over the ribs. The cover holds it down (task 4.x).
STOP_T = 2.7  # Source: chosen, the skill's wall plus fit.
STOP_H = 6.0  # Source: chosen.
RIB_T = 1.5  # Source: chosen, three perimeters.
RIB_H = 3.0  # Source: chosen.

# Servo 2040, unturned, its lower-left corner here. Its rear edge leaves a band
# behind it for J6's wires; its front edge leaves the lead slots in front of
# the servo header over the frame's open front. Source: chosen.
S2_X0 = -31.0
S2_Y0 = 6.0
# Standoff height: the IMU breakout on its bosses fits under the board's
# underside leads with 1 mm to spare. Source: chosen, from IMU_BOSS_H,
# LSM6DSO_HEIGHT and S2040_UNDERSIDE_LEADS.
IMU_BOSS_H = 3.0
S2_STANDOFF = IMU_BOSS_H + M.LSM6DSO_HEIGHT + M.S2040_UNDERSIDE_LEADS + 1.0
S2_STANDOFF_D = 6.0  # Source: chosen, round an M2.5 pilot.
S2_Z0 = DECK_Z1 + S2_STANDOFF + TOUCH  # the PCB's underside
S2_Z1 = S2_Z0 + M.S2040_PCB_THICKNESS  # its top face
S2_TOP = S2_Z0 + M.S2040_HEIGHT  # its tallest component
# The servo header's strip along the board's top edge: board y from here up
# holds only the header and the mounting holes; the plugs go on it.
S2_HEADER_STRIP_Y = 33.5  # Source: drawing, the signal row (35.56) less a plug's half-length past it.


# --------------------------------------------------------------------------- the cover's height

# The front-right corner frame: u along (1, 1)/sqrt 2, out through the
# front-right corner face, v along (-1, 1)/sqrt 2; a box given in it is
# placed with `turned(b, 45)`.
DIAG = 45.0
SQRT2 = math.sqrt(2)
# The exhaust fan stands in the front-right corner, its outer face on the
# limit for held parts (INNER_DIAG), clear of the LM2596's output route in
# front of the module (OUT_FRONT_Y1) and of the servo leads' route. It
# overhangs the Servo 2040's front-right corner, so it stands on its holder
# above the board. Source: chosen (design.md "Assumptions", task group 4).
EXH_U1 = (BOX_DIAG - INNER * SQRT2) / SQRT2
EXH_U0 = EXH_U1 - M.FAN_SIZE[2]
OUT_FRONT_Y1 = 28.6  # the LM2596 output route's front edge, past the diode's tie holes. Source: chosen.
EXH_V0 = (OUT_FRONT_Y1 + FIT + TOUCH) * SQRT2 - EXH_U0
EXH_V1 = EXH_V0 + M.FAN_SIZE[0]
EXH_Z0 = S2_Z1 + 1.0  # 1 mm above the board's top face
EXH_Z1 = EXH_Z0 + M.FAN_SIZE[1]
EXH_LIP_Z = ((EXH_Z0 + 19.5, EXH_Z0 + 26.0), (EXH_Z0 + 5.8, EXH_Z0 + 15.5))  # the exhaust's lips (fan_holders)
# The cover stands on the deck's rim. Its top edge is chamfered at 45
# degrees, and the inside follows it a wall's thickness in. Source: chosen
# (design.md "Shape"), small enough to leave the lid flat over the window.
COVER_Z0 = DECK_Z1 + TOUCH
COVER_CHAMFER = 3.0
COVER_INNER_CHAMFER = COVER_CHAMFER - WALL * (2 - SQRT2)


def under_chamfer(gap: float) -> float:
    """How far the inside of the cover's top chamfer comes down at `gap`
    from the inside of a wall."""
    return max(0.0, COVER_INNER_CHAMFER - gap)


# The lid: its underside FIT above the exhaust fan, the tallest held part,
# where the inside of the top chamfer comes down over the fan's outer face;
# as thick as a wall. Source: the skill (FIT, "Walls").
LID_Z0 = EXH_Z1 + FIT + under_chamfer((BOX_DIAG - WALL * SQRT2) / SQRT2 - EXH_U1)
LID_T = WALL
LID_Z1 = LID_Z0 + LID_T


def s2(bx: float, by: float) -> tuple[float, float]:
    """A point on the Servo 2040's drawing, in the robot's frame."""
    return S2_X0 + bx, S2_Y0 + by


# T-Display above the LM2596, screen up, its long side along y, as high as
# its header pins allow under the lid, so the screen sits close under its
# window: its holder (task 5.2) stands over the LM2596, at least a plate
# thick between the module and the board's tallest underside part.
# Source: chosen.
TD_X0 = 36.7
TD_Y0 = -(M.TD_PCB[3] - M.TD_PCB[2]) / 2
TD_HOLDER_PLATE = 2.0  # Source: chosen, the holder's plate (task 5.2).
TD_Z0 = (LID_Z0 - FIT - M.TD_HEADER_PIN_DEPTH  # PCB underside, its header pins under the chamfer by the right wall
         - under_chamfer(BOX_X - WALL - (TD_X0 - (M.TD_HEADER_ROWS[0][0]))))
# The holder's plate lies FIT under the board's underside parts; its pads
# rise from it to the bare spots. Source: the skill (FIT), TOUCH.
TDH_PLATE_Z1 = TD_Z0 - M.TD_UNDERSIDE_TALLEST - FIT
TDH_PLATE_Z0 = TDH_PLATE_Z1 - TD_HOLDER_PLATE


def td(x: float, y: float) -> tuple[float, float]:
    """A point in the T-Display model's frame, turned screen up (a half turn
    about its y axis: x to -x), in the robot's frame."""
    return TD_X0 - x, TD_Y0 + (y - M.TD_PCB[2])


def td_box(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> Box6:
    """A box in the model's frame (z up from the PCB's screen face, the
    model's -z), in the robot's frame."""
    (ax, ay), (bx, by) = td(x0, y0), td(x1, y1)
    return (min(ax, bx), max(ax, bx), min(ay, by), max(ay, by), TD_Z0 + z0, TD_Z0 + z1)


# LM2596 on the right behind the corridor, its length along y: input pads at
# the rear end, output pads at the front, under the T-Display's holder. It
# stands on two white standoffs (task 5.3) loose in pockets in the deck, as
# tall as leaves FIT under the holder's plate, so the holder keeps it and
# them from lifting out of the pockets. Source: chosen.
LM_X0 = 39.5
LM_Y0 = -27.0
LM_BOSS_D = 6.0  # Source: chosen.
# Pockets in the deck that locate the loose holders (task 5.3): this deep,
# this much wider than the part on each side. Deeper than any holder can
# rise before what lies over it stops it (check_captive): the most is the
# IMU's pad, about 1 mm up to the Servo 2040's underside leads; the LM2596's
# standoffs rise about 0.8 mm, pushing the module, the T-Display's holder and
# the board until its header pins meet the cover. Source: chosen, half the
# deck; the fit between the skill's sliding fit (0.15) and FIT.
POCKET_DEPTH = 1.5
POCKET_FIT = 0.2
POCKET_Z = DECK_Z1 - POCKET_DEPTH  # a pocket's floor
LM_BOSS_H = TDH_PLATE_Z0 - FIT - M.LM2596_SIZE[2] - DECK_Z1 - TOUCH
LM_BOX = (LM_X0, LM_X0 + M.LM2596_SIZE[1], LM_Y0, LM_Y0 + M.LM2596_SIZE[0],
          DECK_Z1 + LM_BOSS_H + TOUCH, DECK_Z1 + LM_BOSS_H + TOUCH + M.LM2596_SIZE[2])
LM_PAD_DEPTH = 2 * M.LM2596_PAD_INSET  # the pad region at each end
# Its wires are soldered to pads on its board: their ends lie within this of
# the board's underside, and the routes to them rise no higher, under the
# exhaust fan holder's right lip (EXH_LIP_Z). Source: measurements.
LM_WIRE_H = M.LM2596_WIRE_HEIGHT
LM_WIRE_Z1 = LM_BOX[4] + LM_WIRE_H


def lm(mx: float, my: float) -> tuple[float, float]:
    """A point on the LM2596 (length along x, width along y, from its corner
    by the input pads), in the robot's frame: length along +y, width along -x."""
    return LM_X0 + M.LM2596_SIZE[1] - my, LM_Y0 + mx


# The IMU breakout under the Servo 2040, its length along x, centred on the
# body's centreline; its wire end at the rear, towards J6. Source: chosen, to
# stay within 30 mm of the centre and 10 mm from the SZBK07.
IMU_X0 = -M.LSM6DSO_SIZE[0] / 2
IMU_Y0 = 9.0
IMU_BOSS_D = 5.0  # Source: chosen, round an M2 pilot.
IMU_BOX = (IMU_X0, IMU_X0 + M.LSM6DSO_SIZE[0], IMU_Y0, IMU_Y0 + M.LSM6DSO_SIZE[1],
           DECK_Z1 + IMU_BOSS_H + TOUCH, DECK_Z1 + IMU_BOSS_H + TOUCH + M.LSM6DSO_HEIGHT)
IMU_WIRE_END = 2.5  # Source: chosen, the strip of the breakout its wires leave.
IMU_BOSSES = tuple((x, IMU_Y0 + M.LSM6DSO_SIZE[1] / 2)
                   for x in (-M.LSM6DSO_HOLE_SPACING / 2, M.LSM6DSO_HOLE_SPACING / 2))

# XT60 on the left wall, mating face outwards (-x), recessed so the mated
# plug's far end stays inside the keep-clear radius; forward of the left-rear
# wall, which is left for the intake (task 4.3). Source: chosen.
XT_FACE_X = -56.0
XT_Y = 10.0
XT_Z0 = DECK_Z1 + TOUCH
XT_BODY = (XT_FACE_X, XT_FACE_X + M.XT60_SIZE[0], XT_Y - M.XT60_SIZE[1] / 2, XT_Y + M.XT60_SIZE[1] / 2,
           XT_Z0, XT_Z0 + M.XT60_SIZE[2])
XT_CUPS = (XT_BODY[1], XT_BODY[1] + M.XT60_CUP_LENGTH, XT_Y - M.XT60_CUP_SECTION[0] / 2,
           XT_Y + M.XT60_CUP_SECTION[0] / 2, XT_Z0 + 1.0, XT_Z0 + 1.0 + M.XT60_CUP_SECTION[1])
# Its cradle: side walls along the body out to the inner wall as a hood for
# the plug, and a back stop either side of the cups that takes the push of
# plugging in; a cable tie over the body, through the deck, holds it against
# the pull of unplugging. Source: chosen.
XT_WALL = 2.0
XT_WALL_H = 5.0  # below the body's top, so the tie bears on the body
XT_STOP = 2.5

# Fuse holder on its leads, lengthwise along y between the XT60's cradle and
# the left-rear, its rear end feeding the SZBK07 and the LM2596. Source: chosen.
FUSE_X0 = -48.0
FUSE_Y1 = -3.0
FUSE_BOX = (FUSE_X0, FUSE_X0 + M.FUSE_HOLDER_SIZE[1], FUSE_Y1 - M.FUSE_HOLDER_SIZE[0], FUSE_Y1,
            DECK_Z1 + TOUCH, DECK_Z1 + TOUCH + M.FUSE_HOLDER_SIZE[2])

# Schottky diode in its sleeve, along x in the LM2596's output route in front
# of the module. Source: chosen.
DIODE_X0 = 39.3
DIODE_Y = 23.6

# Fan holders (task 4.3; refined in task 5.3): each fan stands in a holder
# on the deck that supports it under its ends and locates its end faces with
# cheeks, a snug fit (task 7.2 test-prints one). Source: chosen.
FAN_CHEEK = 2.4  # Source: the skill ("Walls").
FAN_END = 3.0  # how far under each end its support reaches. Source: chosen.
FAN_HOLD_H = 10.0  # the cheeks' and lips' height up the fan. Source: chosen.
# The intake fan stands on the deck behind a grille in the left wall, low,
# between the XT60's cradle (whose rear wall locates its front end) and the
# left-rear corner face. Source: chosen.
INTAKE_Y1 = XT_BODY[2] - FIT - XT_WALL - FIT
INTAKE_Y0 = INTAKE_Y1 - M.FAN_SIZE[0]
INTAKE_X0 = -(INNER_DIAG + INTAKE_Y0)  # its outer face: its rear-outer corner on the corner face's limit
INTAKE_X1 = INTAKE_X0 + M.FAN_SIZE[2]
INTAKE_Z0 = DECK_Z1 + TOUCH
# A low lip outside its foot, under the grille, and lips inside it at its
# ends, clear of the XT60's tie hole. Source: chosen.
INTAKE_LIP_H = 1.7
INTAKE_LIP_X1 = XT_BODY[0] + 9.0 - TIE_L / 2 - FIT  # the XT60's tie (tie_points) less FIT
INTAKE_LIP_LEN = 4.0
# The exhaust holder's columns under the fan's ends start this far out along
# u, clear of the servo leads' route (left) and the LM2596's output route
# (right). Source: chosen.
EXH_HOLD_U0 = (56.0, 57.5)
# Lips over the exhaust fan's inner face at each end keep it from sliding in
# off its columns (the cover's wall keeps it from sliding out): this thick,
# reaching this far over the face, at these heights (left, right), above the
# servo leads' route on the left and between the LM2596's output route and
# the T-Display on the right. Source: chosen.
EXH_LIP_T = 1.6
EXH_LIP_OVER = 2.0

# The cover's hold-down blocks under the lid at the SZBK07's four corners,
# ending this far above it, less than its ribs are tall (RIB_H): it cannot
# lift out over its stops. Each block's size along x and y keeps it clear of
# the terminal spans and the air channel. Source: chosen.
SZ_HOLD_GAP = 0.5
SZ_HOLD = (6.0, 4.0)
# M3 screws down through the lid into heat-set inserts in posts on the deck,
# at three corners: the front-right one holds the exhaust fan, and no post
# fits beside it clear of the routes. Source: chosen, each clear of held parts,
# routes and the air channel.
COVER_SCREWS = ((-29.8, 53.5), (-38.0, -46.0), (42.0, -40.0))
M3_INSERT_D = 4.4  # Source: the skill (M3_INSERT, Ø4.4 x 5).
M3_INSERT_L = 5.0
POST_D = M3_INSERT_D + 2 * 2.0  # Source: chosen, 2 mm of wall round the insert.
SCREW_TAIL = 6.0  # M3 clearance below the insert for a screw's tip. Source: chosen (an M3 x 10).
# Recessed wall detail: a groove round the walls. Source: chosen (design.md "Shape").
GROOVE_Z = (COVER_Z0 + 7.0, COVER_Z0 + 9.0)
GROOVE_DEPTH = 0.8
# Vents: vertical slots through the wall, narrower than a bridge (BRIDGE) so
# their ends print lid-down; the outlet's taller, to pass the channel's air
# into the corner. Source: chosen.
VENT_SLOTS = 7
VENT_SLOT_W = 2.0
VENT_PITCH = 4.0
VENT_IN_H = 24.0
VENT_OUT_H = 26.0
VENT_SPAN = (VENT_SLOTS - 1) * VENT_PITCH + VENT_SLOT_W
# The left wall opens over the battery plug's route, from its foot. Source: FIT, TOUCH.
XT_NOTCH = (XT_BODY[2] - FIT, XT_BODY[3] + FIT, XT_BODY[5] + TOUCH)
# Flush inlays in the lid's top: the name and three chevrons pointing
# forwards, three layers deep, strokes narrower than a bridge so the body's
# pockets over them print. Source: design.md "Colour through the AMS"; chosen.
INLAY_DEPTH = 0.6
NAME = "SPOODER"
NAME_CENTRE = (-12.0, 0.0)
NAME_SCALE = 1.0  # letters 6 x 10 with 2 mm strokes
NAME_ADVANCE = 8.0
CHEVRONS = ((-12.0, 22.0), (-12.0, 28.0), (-12.0, 34.0))
CHEVRON_ARM = 8.0
INLAY_STROKE = 2.0
# Letters on a 6 x 10 grid, as rectangles (x0, y0, x1, y1). Source: chosen.
LETTERS = {
    "S": ((0, 8, 6, 10), (0, 4, 2, 10), (0, 4, 6, 6), (4, 0, 6, 6), (0, 0, 6, 2)),
    "P": ((0, 0, 2, 10), (0, 8, 6, 10), (4, 4, 6, 10), (0, 4, 6, 6)),
    "O": ((0, 0, 2, 10), (4, 0, 6, 10), (0, 0, 6, 2), (0, 8, 6, 10)),
    "D": ((0, 0, 2, 10), (0, 8, 5, 10), (0, 0, 5, 2), (4, 1.5, 6, 8.5)),
    "E": ((0, 0, 2, 10), (0, 8, 6, 10), (0, 4, 5, 6), (0, 0, 6, 2)),
    "R": ((0, 0, 2, 10), (0, 8, 6, 10), (4, 4, 6, 10), (0, 4, 6, 6), (3.5, 0, 5.5, 4.5)),
}

# --------------------------------------------------------------------------- routes

# Route boxes start just above the deck. Source: TOUCH.
RZ0 = DECK_Z1 + TOUCH
# The corridor up the right from the SZBK07's output end to the EXT terminal
# and the sensor header, between the board's edge and the T-Display above the
# LM2596. Source: chosen.
COR_X = (SZ_BOX[1] + FIT, 36.2)
# The band behind the board, between the SZBK07's front ribs and the board's
# rear standoffs, along which J6's wires come. Source: chosen.
BAND_Y = (SZ_BOX[3] + FIT + RIB_T + TOUCH, S2_Y0 + 2.7 - S2_STANDOFF_D / 2 - 0.1)
# Lead slots for the 18 servo leads, in front of the servo header over the
# frame's open front; each passes a plug. Source: chosen.
SLOT_Y = (48.4, 48.4 + M.SERVO_PLUG_SECTION[1] + 0.5)
SLOT_X = ((-21.0, -3.0), (3.0, 21.0))


def interior_ok(b: Box6) -> bool:
    """Whether a box's footprint lies inside the cover's walls."""
    return (b[0] >= -INNER_X - 1e-6 and b[1] <= INNER_X + 1e-6 and b[2] >= -INNER_Y - 1e-6 and b[3] <= INNER_Y + 1e-6
            and all(abs(x) + abs(y) <= INNER_DIAG + 1e-6 for x in b[:2] for y in b[2:4]))


def octagon(half_x: float, half_y: float, diag: float, z0: float, z1: float) -> Solid:
    """The box's footprint: a rectangle with 45-degree corner faces."""
    from build123d import Plane, Polygon, extrude

    c = half_x + half_y - diag
    pts = [(half_x, -half_y + c), (half_x, half_y - c), (half_x - c, half_y), (-half_x + c, half_y),
           (-half_x, half_y - c), (-half_x, -half_y + c), (-half_x + c, -half_y), (half_x - c, -half_y)]
    return extrude(Plane.XY.offset(z0) * Polygon(*pts, align=None), amount=z1 - z0).solid()


def cyl(r: float, x: float, y: float, z0: float, z1: float) -> Solid:
    from checks import cylinder

    return cylinder(r, (x, y, z0), (0, 0, 1), z1 - z0)


# --------------------------------------------------------------------------- held parts


def servo2040() -> Held:
    """The board: its PCB, the leads under it (clear of the standoffs), and
    its components up to its full height, except along the servo header's
    strip, which the plugs' route holds."""
    x0, y0 = S2_X0, S2_Y0
    x1, y1 = x0 + M.S2040_BOARD[0], y0 + M.S2040_BOARD[1]
    shape = box(x0, x1, y0, y1, S2_Z0, S2_Z1)
    leads = box(x0, x1, y0, y1, S2_Z0 - M.S2040_UNDERSIDE_LEADS, S2_Z0)
    for hx, hy in M.S2040_HOLES:
        x, y = s2(hx, hy)
        leads = leads - cyl(S2_STANDOFF_D / 2 + FIT, x, y, S2_Z0 - 5, S2_Z0 + 1)
    body = box(x0, x1, y0, y0 + S2_HEADER_STRIP_Y, S2_Z1, S2_TOP)
    for hx, hy in M.S2040_HOLES:  # the mounting holes are clear above the board
        body = body - cyl(SCREW_CLEAR_R, *s2(hx, hy), S2_Z1 - 1, S2_TOP + 1)
    return Held(SERVO_2040, shape + leads + body)


def s2_terminals() -> dict[str, Terminal]:
    p = M.S2040_PITCH
    ex, ey = s2(*M.S2040_EXT_CENTRE)
    sx, sy = s2(*M.S2040_SENSOR_CENTRE)
    j6 = [s2(M.S2040_J6_PIN9_X + (9 - n) * p, M.S2040_J6_ROW_Y) for n in (M.S2040_J6_PINS, 1)]
    hx0 = s2(M.S2040_SERVO_COL1_X, 0)[0] - p / 2
    hx1 = s2(M.S2040_SERVO_COL1_X + 2 * M.S2040_SERVO_GROUP_STEP + 5 * p, 0)[0] + p / 2
    hy0 = s2(0, M.S2040_SERVO_ROW_SIGNAL_Y)[1] - p / 2
    hy1 = s2(0, M.S2040_SERVO_ROW_GND_Y)[1] + p / 2
    usb = s2(0, M.S2040_USB_Y)
    top = S2_TOP
    return {
        # The board is unturned, so its wire sides are the robot's.
        "ext": Terminal("ext", SERVO_2040, (ex - 2.5, ex + 2.5, ey - M.S2040_EXT_POLE_PITCH, ey + M.S2040_EXT_POLE_PITCH,
                                            S2_Z1 + 0.1, top), SIDES[M.S2040_EXT_WIRE_SIDE]),
        SENSOR_HEADER: Terminal(SENSOR_HEADER, SERVO_2040,
                                (sx - 1.5 * p, sx + 1.5 * p, sy - 3 * p, sy + 3 * p, S2_Z1 + 0.1, top),
                                SIDES[M.S2040_SENSOR_WIRE_SIDE]),
        # J6's pads, pin 17 (5V) at the left; its wires leave over the board's rear edge.
        J6: Terminal(J6, SERVO_2040, (j6[0][0] - p / 2, j6[1][0] + p / 2, j6[0][1] - p / 2, j6[0][1] + p / 2,
                                          S2_Z1 + 0.1, S2_Z1 + 2.5), (0.0, -1.0, 0.0)),
        # Pins 17 (5V) and 16 (GND), which the LM2596's output feeds.
        "j6 5v": Terminal("j6 5v", SERVO_2040, (j6[0][0] - p / 2, j6[0][0] + 1.5 * p, j6[0][1] - p / 2,
                                                j6[0][1] + p / 2, S2_Z1 + 0.1, S2_Z1 + 2.5), (0.0, -1.0, 0.0)),
        "servo header": Terminal("servo header", SERVO_2040, (hx0, hx1, hy0, hy1, S2_Z0, S2_Z1), (0.0, 0.0, 1.0)),
        "usb-c": Terminal("usb-c", SERVO_2040, (usb[0], usb[0] + 7.0, usb[1] - 4.5, usb[1] + 4.5,
                                                S2_Z1, S2_Z1 + 3.2), (-1.0, 0.0, 0.0)),
    }


def szbk07() -> Held:
    """Held down by the cover: it cannot rise past its lowest stops."""
    return Held(SZBK07, box6(SZ_BOX), held_down=min(STOP_H, RIB_H))


def sz_terminal(end: str) -> Terminal:
    if end == "input":
        x0, x1, side = SZ_BOX[0], SZ_BOX[0] + SZ_TERM_DEPTH, (-1.0, 0.0, 0.0)
    else:
        x0, x1, side = SZ_BOX[1] - SZ_TERM_DEPTH, SZ_BOX[1], (1.0, 0.0, 0.0)
    return Terminal(f"szbk07 {end}", SZBK07, (x0, x1, *SZ_TERM, SZ_BOX[4], SZ_BOX[5]), side)


def lm2596() -> Held:
    """The module's envelope, clear above its mounting holes."""
    shape = box6(LM_BOX)
    for h in M.LM2596_HOLES:
        shape = shape - cyl(SCREW_CLEAR_R, *lm(*h), LM_BOX[4] + MODULE_PCB_T, LM_BOX[5] + 1)
    return Held("lm2596", shape)


def lm_terminal(end: str) -> Terminal:
    if end == "input":
        y0, y1, side = LM_BOX[2], LM_BOX[2] + LM_PAD_DEPTH, (0.0, -1.0, 0.0)
    else:
        y0, y1, side = LM_BOX[3] - LM_PAD_DEPTH, LM_BOX[3], (0.0, 1.0, 0.0)
    return Terminal(f"lm2596 {end}", "lm2596", (LM_BOX[0], LM_BOX[1], y0, y1, LM_BOX[4], LM_WIRE_Z1), side)


def t_display() -> Held:
    """The T-Display screen up: its PCB, the screen and buttons above it, the
    header rows clear on both faces (pins above, wires soldered from below,
    measurements' T-Display notes), and its underside parts except on the
    bare spots, where its holder bears (task 5.2)."""
    t = M.TD_PCB_THICKNESS
    shape = box6(td_box(M.TD_PCB[0], M.TD_PCB[1], M.TD_PCB[2], M.TD_PCB[3], 0, t))
    shape = shape + box6(td_box(*M.TD_SCREEN, t, t + M.TD_SCREEN_DEPTH))
    for b in M.TD_BUTTONS:
        shape = shape + box6(td_box(*b, t, t + M.TD_BUTTON_DEPTH))
    under = box6(td_box(M.TD_PCB[0], M.TD_PCB[1], M.TD_PCB[2], M.TD_PCB[3], -M.TD_UNDERSIDE_TALLEST, 0))
    for b in M.TD_BARE:
        under = under - box6(td_box(*b, -M.TD_UNDERSIDE_TALLEST - 1, 0.5))
    shape = shape + under
    for row in M.TD_HEADER_ROWS:
        shape = shape + box6(td_box(*row, -M.TD_UNDERSIDE_TALLEST, M.TD_HEADER_PIN_DEPTH))
    screen = td_box(*M.TD_SCREEN, t + M.TD_SCREEN_DEPTH, t + M.TD_SCREEN_DEPTH)
    exposed = [Exposure(UP, screen)]
    for b in M.TD_BUTTONS:
        exposed.append(Exposure(UP, td_box(*b, t + M.TD_BUTTON_DEPTH, t + M.TD_BUTTON_DEPTH)))
    # The window shows the board's face between the screen and the buttons too.
    pcb_top = TD_Z0 + t
    for x0, x1, y0, y1 in window_rects():
        exposed.append(Exposure(UP, (x0, x1, y0, y1, pcb_top, pcb_top), must_show=False))
    # Its holder's lips stand TDH_LIP_H above its underside: the lid, over its
    # header pins, must come nearer than that (task 5.2).
    return Held("t-display", shape, exposed=tuple(exposed), held_down=TDH_LIP_H)


def window_rects() -> list[tuple[float, float, float, float]]:
    """The window's opening at the lid's underside, as rectangles: the
    outline of the screen and the buttons beyond its ends, and a notch out to
    the button beside it (each less than 2 mm from the screen, so one
    opening; the skill)."""
    t = M.TD_PCB_THICKNESS
    screen = td_box(*M.TD_SCREEN, t, t)
    buttons = [td_box(*b, t, t) for b in M.TD_BUTTONS]
    ends = [screen] + [b for b in buttons if b[0] < screen[1] and b[1] > screen[0]]
    main = (min(b[0] for b in ends), max(b[1] for b in ends), min(b[2] for b in ends), max(b[3] for b in ends))
    rects = [main]
    for b in buttons:
        if b in ends:
            continue
        if b[0] >= main[1]:
            rects.append((main[1], b[1], b[2], b[3]))
        else:
            rects.append((b[0], main[0], b[2], b[3]))
    return rects


def td_terminal(row: int, name: str, y_range: tuple[float, float] | None = None) -> Terminal:
    """A header row's underside, where its wires are soldered in from below."""
    b = td_box(*M.TD_HEADER_ROWS[row], -M.TD_UNDERSIDE_TALLEST, 0)
    if y_range is not None:
        b = (b[0], b[1], max(b[2], y_range[0]), min(b[3], y_range[1]), b[4], b[5])
    return Terminal(name, "t-display", b, (0.0, 0.0, -1.0))


def imu() -> Held:
    """The breakout's envelope, clear above its mounting holes."""
    shape = box6(IMU_BOX)
    for x, y in IMU_BOSSES:
        shape = shape - cyl(SCREW_CLEAR_R, x, y, IMU_BOX[4] + MODULE_PCB_T, IMU_BOX[5] + 1)
    return Held(IMU, shape)


def imu_terminal() -> Terminal:
    return Terminal("lsm6dso wires", IMU, (IMU_BOX[0], IMU_BOX[1], IMU_BOX[2], IMU_BOX[2] + IMU_WIRE_END,
                                           IMU_BOX[4], IMU_BOX[5]), (0.0, -1.0, 0.0))


def xt60() -> Held:
    face = (XT_BODY[0], XT_BODY[0], XT_BODY[2], XT_BODY[3], XT_BODY[4], XT_BODY[5])
    return Held("xt60", box6(XT_BODY) + box6(XT_CUPS), exposed=(Exposure((-1.0, 0.0, 0.0), face),))


def fuse() -> Held:
    return Held("fuse holder", box6(FUSE_BOX))


def fuse_terminal(end: str) -> Terminal:
    if end == "in":  # the front end, from the XT60
        y0, y1, side = FUSE_BOX[3] - 3.0, FUSE_BOX[3], (0.0, 1.0, 0.0)
    else:
        y0, y1, side = FUSE_BOX[2], FUSE_BOX[2] + 3.0, (0.0, -1.0, 0.0)
    return Terminal(f"fuse {end}", "fuse holder", (FUSE_BOX[0], FUSE_BOX[1], y0, y1, FUSE_BOX[4], FUSE_BOX[5]), side)


def diode() -> Held:
    from checks import cylinder

    r = M.SCHOTTKY_SLEEVE_D / 2
    shape = cylinder(r, (DIODE_X0, DIODE_Y, RZ0 + r), (1, 0, 0), M.SCHOTTKY_SLEEVE_LENGTH)
    return Held(SCHOTTKY, shape, route=LM2596_OUTPUT_ROUTE)


# --------------------------------------------------------------------------- routes


def routes(t: dict[str, Terminal]) -> list[Route]:
    """The reserved cable routes, each with the terminals it reaches."""
    cz1 = TD_Z0 - M.TD_UNDERSIDE_TALLEST - TOUCH  # under the T-Display's underside
    top = S2_TOP  # the board's tallest component
    col = (SZ_BOX[0] - 4.65, SZ_BOX[0] - FIT)  # beside the SZBK07's input end
    band = (col[0], COR_X[1], *BAND_Y, RZ0, top)
    corridor = (*COR_X, SZ_TERM[0], 38.0, RZ0, cz1)
    j6_approach = (t["j6"].region[0] - 0.2, t["j6"].region[1] + 0.2, BAND_Y[0], S2_Y0 - TOUCH, S2_Z1 + TOUCH, top)
    left_strip_cups = (col[0], S2_X0 - 0.35, XT_CUPS[2] - 0.5, XT_CUPS[3] + 0.5, RZ0, XT_BODY[5] + 0.5)
    fuse_rear = (FUSE_BOX[0], col[1], FUSE_BOX[2] - 3.0, FUSE_BOX[2] - FIT, RZ0, FUSE_BOX[5] + 1.0)
    lm_in = (COR_X[0], LM_BOX[1], LM_BOX[2] - 4.0, LM_BOX[2] - TOUCH, RZ0, LM_WIRE_Z1)
    # Up to the inner header row, clear of the LM2596's boss beside it.
    td_link = (COR_X[0], td_terminal(1, "x").region[1] + 0.02, *td_terminal(1, "x").region[2:4], RZ0, cz1)
    out_front = (COR_X[0], LM_BOX[1], LM_BOX[3] + TOUCH, OUT_FRONT_Y1, RZ0, LM_WIRE_Z1)
    td_x = td_terminal(0, "x").region
    out_td = (td_x[1] - 6.0, td_x[1] + 0.3, LM_BOX[3] + TOUCH, LM_BOX[3] + 6.1, RZ0, cz1)
    lead_x = (t["servo header"].region[0] - 0.25, t["servo header"].region[1] + 0.25)
    plug_top = S2_Z1 + M.SERVO_PLUG_HEIGHT
    # The LM2596's output to J6 runs under the board, clear of the IMU and the
    # standoffs, then up at J6's 5V end: every box at least a sleeve across.
    under_y = (IMU_BOX[3] + TOUCH, IMU_BOX[3] + TOUCH + 6.05)
    under_z1 = S2_Z0 - M.S2040_UNDERSIDE_LEADS - 0.3
    j6x = t["j6 5v"].region
    return [
        Route.of("xt60 to fuse and szbk07 input", [
            left_strip_cups,
            (col[0], col[1], SZ_TERM[0], BAND_Y[1], RZ0, SZ_BOX[5]),
            (FUSE_BOX[0], col[1], FUSE_BOX[3] + TOUCH, XT_BODY[2] - FIT - XT_WALL - TOUCH, RZ0, FUSE_BOX[5] + 1.0),
            fuse_rear,
            (col[0], col[1], fuse_rear[2], SZ_TERM[1], RZ0, SZ_BOX[5]),
        ], ends=(Terminal("xt60 cups", "xt60", XT_CUPS, (1.0, 0.0, 0.0)), fuse_terminal("in"), fuse_terminal("out"),
                 sz_terminal("input"))),
        Route.of("szbk07 output to ext", [
            (*COR_X, *SZ_TERM, RZ0, SZ_BOX[5]),
            corridor,
            (S2_X0 + M.S2040_BOARD[0] + TOUCH, COR_X[1], t["ext"].region[2] - 0.5, t["ext"].region[3] + 0.5,
             S2_Z1 + TOUCH, top),
        ], ends=(sz_terminal("output"), t["ext"])),
        Route.of("fused supply to lm2596", [
            fuse_rear,
            (col[0], col[1], fuse_rear[2], BAND_Y[1], RZ0, SZ_BOX[5]),
            band,
            (*COR_X, LM_BOX[2] - 4.0, BAND_Y[1], RZ0, cz1),
            lm_in,
        ], ends=(fuse_terminal("out"), lm_terminal("input"))),
        Route.of(LM2596_OUTPUT_ROUTE, [
            out_front,
            out_td,
            (j6x[0] - 0.2, COR_X[1] + 3.0, *under_y, RZ0, under_z1),
            (j6x[0] - 0.2, j6x[0] + 6.2, BAND_Y[0] - RIB_T - TOUCH - 0.1, under_y[1], RZ0, under_z1),
            (j6x[0] - 0.2, j6x[0] + 6.2, BAND_Y[0] - RIB_T - TOUCH - 0.1, S2_Y0 - TOUCH, RZ0, top),
        ], ends=(lm_terminal("output"), t["j6 5v"], td_terminal(0, "t-display 5v", (LM_BOX[3] + 0.2, LM_BOX[3] + 1.6)))),
        Route.of("t-display link to j6", [td_link, corridor, band, j6_approach],
                 ends=(td_terminal(1, "t-display link"), t["j6"])),
        Route.of("servo leads", [
            # Over the header, and on past the board's edge, above its PCB, into the drop to the slots.
            (*lead_x, t["servo header"].region[2] - 0.3, S2_Y0 + M.S2040_BOARD[1] + 0.6, S2_Z1 + TOUCH, plug_top),
            (*lead_x, S2_Y0 + M.S2040_BOARD[1] + TOUCH, SLOT_Y[1] + 1.5, RZ0, plug_top),
        ], ends=(t["servo header"],) + tuple(
            Terminal(f"lead slot {i + 1}", None, (x0, x1, *SLOT_Y, RZ0, RZ0 + 1.0), (0.0, 0.0, -1.0))
            for i, (x0, x1) in enumerate(SLOT_X))),
        Route.of("end-stop wires", [
            (S2_X0 + M.S2040_BOARD[0] + TOUCH, COR_X[1], t["sensor header"].region[2] - 0.5,
             t["sensor header"].region[3] + 0.5, RZ0, top),
        ], ends=(Terminal("end-stop entry", None, (*ENTRY, RZ0, RZ0 + 1.0), (0.0, 0.0, -1.0)), t["sensor header"])),
        Route.of("imu to j6", [
            (IMU_BOX[0], IMU_BOX[1], BAND_Y[0], IMU_BOX[2] - TOUCH, RZ0, under_z1),
            band,
            j6_approach,
        ], ends=(imu_terminal(), t["j6"])),
        Route.of("pass-through to j6", [band, j6_approach],
                 ends=(Terminal("pass-through", None, (*PASS, RZ0, RZ0 + 1.0), (0.0, 0.0, -1.0)), t["j6"])),
        # Only with the cover off: the cable runs out over the deck's edge.
        Route.of("usb-c cable", [
            (S2_X0 - TOUCH - M.USB_C_PLUG[2], S2_X0 - TOUCH, t["usb-c"].region[2] + 4.5 - M.USB_C_PLUG[0] / 2,
             t["usb-c"].region[2] + 4.5 + M.USB_C_PLUG[0] / 2,
             S2_Z1 + 1.6 - M.USB_C_PLUG[1] / 2, S2_Z1 + 1.6 + M.USB_C_PLUG[1] / 2),
        ], ends=(t["usb-c"],), cover_off=True),
        Route.of("battery plug", [
            (XT_FACE_X - M.XT60_PLUG_LENGTH, XT_FACE_X - TOUCH, XT_BODY[2], XT_BODY[3], XT_BODY[4], XT_BODY[5]),
        ], ends=(Terminal("xt60 face", "xt60", (XT_BODY[0], XT_BODY[0] + 1.0, *XT_BODY[2:]), (-1.0, 0.0, 0.0)),)),
    ]


# The end-stop wires' entry, up through the deck into the corridor beside
# the sensor header, over the frame's open side; the pass-through, down from
# the band to the open centre. (x0, x1, y0, y1). Source: chosen.
ENTRY = (COR_X[0] + 0.1, COR_X[1] - 0.1, 16.0, 26.0)
PASS = (-5.0, -1.0, BAND_Y[0] + 0.5, BAND_Y[1] - 0.5)


def openings() -> list[Opening]:
    found = [Opening(f"lead slot {i + 1}", "deck", (x0, x1, *SLOT_Y), DECK_Z0, DECK_Z1, kind="slot")
             for i, (x0, x1) in enumerate(SLOT_X)]
    found.append(Opening("end-stop entry", "deck", ENTRY, DECK_Z0, DECK_Z1, kind="entry", wires=7))
    found.append(Opening("pass-through", "deck", PASS, DECK_Z0, DECK_Z1, kind="pass-through", wires=4))
    for name, pairs in tie_points().items():
        for i, fp in enumerate(pairs):
            found.append(Opening(f"{name} tie {i + 1}", "deck", fp, DECK_Z0, DECK_Z1))
    return found


def tie_points() -> dict[str, list[tuple[float, float, float, float]]]:
    """Cable-tie hole pairs, (x0, x1, y0, y1) each: across the diode's sleeve,
    over the XT60's body, round the fuse holder (twice), and for the lead
    bundles beside the slots. Source: chosen, each over open frame or floor."""
    def across_y(x: float, y0: float, y1: float):  # holes either side of a run along x
        return [(x - TIE_L / 2, x + TIE_L / 2, y0 - TIE_W, y0), (x - TIE_L / 2, x + TIE_L / 2, y1, y1 + TIE_W)]

    def across_x(y: float, x0: float, x1: float):  # either side of a run along y
        return [(x0 - TIE_W, x0, y - TIE_L / 2, y + TIE_L / 2), (x1, x1 + TIE_W, y - TIE_L / 2, y + TIE_L / 2)]

    r = M.SCHOTTKY_SLEEVE_D / 2
    return {
        "diode": across_y(43.7, DIODE_Y - r - 0.1, DIODE_Y + r + 0.1),
        "xt60": across_y(XT_BODY[0] + 9.0, XT_BODY[2] - XT_WALL - FIT - 0.2, XT_BODY[3] + XT_WALL + FIT + 0.2),
        "fuse rear": across_x(FUSE_BOX[2] + 20.0, FUSE_BOX[0] - 0.2, FUSE_BOX[1] + 0.2),
        "fuse front": across_x(FUSE_BOX[3] - 4.0, FUSE_BOX[0] - 0.2, FUSE_BOX[1] + 0.2),
    }


# --------------------------------------------------------------------------- printed parts


def deck() -> Part:
    """The deck: the box's footprint and the tabs to the ring's inserts, with
    the insert holes, slots, entries and tie holes through it, the pockets
    that locate the loose holders (task 5.3), and on it the SZBK07's stops,
    the Servo 2040's standoffs, the XT60's cradle and the cover's posts."""
    z0, z1 = DECK_Z0, DECK_Z1
    shape = octagon(BOX_X, BOX_Y, BOX_DIAG, z0, z1) + box(-TAB_X, TAB_X, -TAB_Y, TAB_Y, z0, z1)
    shape = shape + features()
    for x, y in M.FRAME_INSERTS:
        shape = shape - cyl(M3_D / 2, x, y, z0 - 1, z1 + 1) - cyl(CBORE_D / 2, x, y, z1 - CBORE_DEPTH, z1 + 1)
    for o in openings():
        shape = shape - box(*o.footprint, z0 - 1, z1 + 1)
    for (x, y), d, top in pilots():
        shape = shape - cyl(d / 2, x, y, top - 5.0, top + 1)
    for x, y in COVER_SCREWS:  # a heat-set insert in each post's top, and room below it for the screw
        top = LID_Z0 - TOUCH
        shape = shape - cyl(M3_INSERT_D / 2, x, y, top - M3_INSERT_L, top + 1)
        shape = shape - cyl(M3_D / 2, x, y, top - M3_INSERT_L - SCREW_TAIL, top)
    for _, feet in loose_holders():
        for foot in feet:
            shape = shape - pocket(foot)
    return Part("deck", shape, "deck", UP, "black")


# --------------------------------------------------------------------------- loose holders (task group 5)

# A holder's foot, as the deck's pocket for it is cut: a box (x0, x1, y0, y1)
# in a frame turned `turn` degrees, or a circle (x, y, r).
Foot = tuple


def foot_solid(foot: Foot, grow: float, z0: float, z1: float) -> Solid:
    """A foot's footprint grown by `grow` on each side, from z0 to z1."""
    if len(foot) == 3:
        x, y, r = foot
        return cyl(r + grow, x, y, z0, z1)
    x0, x1, y0, y1, turn = foot
    return turned((x0 - grow, x1 + grow, y0 - grow, y1 + grow, z0, z1), turn)


def pocket(foot: Foot) -> Solid:
    """The deck's pocket for a foot: POCKET_FIT round it, POCKET_DEPTH deep,
    cut to just above the deck's top (not into the features on it), and kept
    inside the cover's walls, whose rim on the deck stays whole."""
    return foot_solid(foot, POCKET_FIT, POCKET_Z, DECK_Z1 + 0.01) & cavity()


def foot_base(foot: Foot, z1: float) -> Solid:
    """The part of a holder in its pocket and up to z1: the foot, standing
    TOUCH above the pocket's floor."""
    return foot_solid(foot, 0.0, POCKET_Z + TOUCH, z1)


def loose_holders() -> list[tuple[Part, list[Foot]]]:
    """The white holders that sit loose in pockets in the deck, each with its
    feet: the LM2596's standoffs, the IMU's pad, the fans' holders and the
    T-Display's holder. Each is kept in its pocket by what lies over it."""
    return [lm_standoffs(), imu_pad(), intake_holder(), *exhaust_holders(), td_holder()]


def lm_standoffs() -> tuple[Part, list[Foot]]:
    """The LM2596's two standoffs, one part copied to the other hole: a
    round column from its pocket up to the module, with a pilot for the
    module's self-tapping screw down its length. The module over them is
    FIT under the T-Display's holder, which keeps it and them in place."""
    (x, y), *rest = [lm(*h) for h in M.LM2596_HOLES]
    top = LM_BOX[4] - TOUCH
    shape = foot_base((x, y, LM_BOSS_D / 2), top) - cyl(PILOT_M25_D / 2, x, y, POCKET_Z + TOUCH + 1.0, top + 1)
    copies = tuple(Location((ox - x, oy - y, 0)) for ox, oy in rest)
    feet = [(hx, hy, LM_BOSS_D / 2) for hx, hy in [(x, y), *rest]]
    return Part("lm2596-standoff", shape, "holder", UP, "white", copies=copies, held_down=POCKET_DEPTH,
                carries=("lm2596",)), feet


# The IMU's pad (task 5.3): a flat plate across the breakout between its two
# bosses, clear of the floor inserts' screw heads (r 2.85, ISO 7380) beside
# it; the Servo 2040 over the breakout keeps it in its pocket. Source: chosen.
IMU_PAD_Y = (11.0, 17.5)
IMU_PAD_TOP = DECK_Z1 + 1.0  # the plate's top, under the breakout's underside parts
IMU_PILOT_DEPTH = 4.0  # an M2 x 5 through the 1.6 mm breakout. Source: chosen.


def imu_pad() -> tuple[Part, list[Foot]]:
    """The pad: a plate, and a boss with a pilot under each of the breakout's
    holes."""
    x0, x1 = IMU_BOX[0], IMU_BOX[1]
    feet: list[Foot] = [(x0, x1, *IMU_PAD_Y, 0.0)] + [(x, y, IMU_BOSS_D / 2) for x, y in IMU_BOSSES]
    shape = foot_base(feet[0], IMU_PAD_TOP)
    top = IMU_BOX[4] - TOUCH
    for f in feet[1:]:
        shape = shape + foot_base(f, top)
    for x, y in IMU_BOSSES:
        shape = shape - cyl(PILOT_M2_D / 2, x, y, top - IMU_PILOT_DEPTH, top + 1)
    return Part("imu-pad", shape, "holder", UP, "white", held_down=POCKET_DEPTH, carries=(IMU,)), feet


def intake_holder() -> tuple[Part, list[Foot]]:
    """The intake fan's holder: a plate under the fan, a low lip outside its
    foot, lips inside its ends and a cheek at its rear end (the XT60
    cradle's rear wall locates its front end). The cheek rises behind the
    fan, clear of the air channel, to just under the lid, which keeps the
    holder in its pocket: the fan does not reach the lid."""
    wall_in = -(BOX_X - WALL) + TOUCH  # the cover's left wall, inside
    diag_in = BOX_DIAG - WALL * SQRT2  # the cover's corner faces, inside, as |x| + |y|
    hz = DECK_Z1 + FAN_HOLD_H
    cheek_y = (INTAKE_Y0 - FIT - FAN_CHEEK, INTAKE_Y0 - FIT)
    cheek_x0 = -(diag_in - 2 * TOUCH + cheek_y[0])
    lips_y = ((INTAKE_Y0, INTAKE_Y0 + INTAKE_LIP_LEN), (INTAKE_Y1 - INTAKE_LIP_LEN, INTAKE_Y1))
    # The plate under the fan and the cheek, and under each inner lip (overlapping the plate).
    feet: list[Foot] = [(wall_in, INTAKE_X1 + FIT, cheek_y[0], INTAKE_Y1, 0.0)]
    feet += [(INTAKE_X1 + FIT - EMBED, INTAKE_LIP_X1, *y, 0.0) for y in lips_y]
    shape = foot_base(feet[0], DECK_Z1)
    for f, y in zip(feet[1:], lips_y):
        shape = shape + foot_base(f, DECK_Z1) + box(INTAKE_X1 + FIT, INTAKE_LIP_X1, *y, DECK_Z1 - EMBED, hz)
    shape = (shape + box(wall_in, INTAKE_X0 - FIT, INTAKE_Y0, INTAKE_Y1, DECK_Z1 - EMBED, DECK_Z1 + INTAKE_LIP_H)
             + box(cheek_x0, INTAKE_X1 + FIT, *cheek_y, DECK_Z1 - EMBED, hz)
             + box(cheek_x0, INTAKE_X1 - INTAKE_COLUMN_GAP, *cheek_y, hz - EMBED, LID_Z0 - INTAKE_LID_GAP))
    return Part("intake-fan-holder", shape & cavity(TOUCH), "holder", UP, "white", held_down=POCKET_DEPTH,
                carries=(FANS[1],)), feet


# The intake holder's cheek rises behind the fan to this far under the lid
# (the exhaust holders' cheeks too), this far inside the fan's inner face
# (clear of the air channel's first segment). Source: chosen, less than
# POCKET_DEPTH; the channel's 0.2.
INTAKE_LID_GAP = 0.5
INTAKE_COLUMN_GAP = 0.3


def exhaust_holders() -> list[tuple[Part, list[Foot]]]:
    """The exhaust fan's holders: two columns under its ends, above the
    board's corner, each with a cheek beyond its end that reaches in over the
    fan's inner face as a lip; in the corner frame (u, v), turned 45 degrees.
    The cheeks rise to just under the cover, which keeps them in their
    pockets with the fan fitted or not."""
    top = EXH_Z0 - TOUCH
    cz = LID_Z0 - INTAKE_LID_GAP
    lip = (EXH_U0 - FIT - EXH_LIP_T, EXH_U0 - FIT)
    inside = cavity(TOUCH)
    # Left end: the column and the cheek, which rises past the servo leads'
    # route and there reaches in over the fan's inner face as a lip.
    lf: Foot = (EXH_HOLD_U0[0], EXH_U1, EXH_V1 - FAN_END, EXH_V1 + FIT + FAN_CHEEK, DIAG)
    left = (foot_base(lf, top)
            + turned((EXH_HOLD_U0[0], EXH_U1, EXH_V1 + FIT, EXH_V1 + FIT + FAN_CHEEK, top - 1.0, cz), DIAG)
            + turned((lip[0], EXH_HOLD_U0[0] + 1.0, EXH_V1 + FIT, EXH_V1 + FIT + FAN_CHEEK, *EXH_LIP_Z[0]), DIAG)
            + turned((*lip, EXH_V1 - EXH_LIP_OVER, EXH_V1 + FIT + 0.5, *EXH_LIP_Z[0]), DIAG))
    # Right end: the column and the cheek, which reaches in over the fan's
    # inner face above the LM2596's output route, below the T-Display.
    rf: Foot = (EXH_HOLD_U0[1], EXH_U1, EXH_V0 - FIT - FAN_CHEEK, EXH_V0 + FAN_END, DIAG)
    right = (foot_base(rf, top)
             + turned((EXH_HOLD_U0[1], EXH_U1, EXH_V0 - FIT - FAN_CHEEK, EXH_V0 - FIT, top - 1.0, cz), DIAG)
             + turned((lip[0], EXH_U1, EXH_V0 - FIT - FAN_CHEEK, EXH_V0 - FIT, *EXH_LIP_Z[1]), DIAG)
             + turned((*lip, EXH_V0 - FIT - 0.5, EXH_V0 + EXH_LIP_OVER, *EXH_LIP_Z[1]), DIAG))
    return [(Part(f"exhaust-fan-holder-{side}", shape & inside, "holder", UP, "white", held_down=POCKET_DEPTH,
                  carries=(FANS[0],)), [foot])
            for side, shape, foot in (("left", left, lf), ("right", right, rf))]


# The T-Display's holder (task 5.2): a white portal over the LM2596, printed
# on its back (up +y), loose in pockets in the deck and held down through the
# board by the lid. Source: chosen (design.md "Assumptions", task group 5).
# - Its right wall stands between the LM2596 and the cover's right wall, its
#   rear wall behind the LM2596's input route; the plate over the module joins
#   them, and the rear block on the plate is its rear lip.
# - Pads rise from the plate into the board's bare spots (TD_BARE), FIT inside
#   each, and bear on its underside; their faces towards the rear (down as
#   printed) slope at 45 degrees, inside the bare spot.
# - The front lip stands beyond the board's front end, its foot sloping under
#   the end's bare strip.
TDH_LIP_T = 1.6  # Source: chosen, four perimeters.
TDH_LIP_H = 2.0  # the lips' top above the board's underside, higher than the lid lets it rise. Source: chosen.
TDH_REAR_Y0 = -34.4  # the rear wall's back, clear of the rear-right cover post. Source: chosen.


def td_holder() -> tuple[Part, list[Foot]]:
    pcb = td_box(*M.TD_PCB, 0, 0)
    face = TD_Z0 - TOUCH  # the pads' tops, under the board's underside
    top = TD_Z0 + TDH_LIP_H
    pz0, pz1 = TDH_PLATE_Z0, TDH_PLATE_Z1
    x0 = td_terminal(1, "x").region[1] + 0.02 + TOUCH  # clear of the T-Display's link route
    x_front = (40.0 + FIT + TOUCH, td_terminal(0, "x").region[1] - 6.0 - TOUCH)  # clear of the air channel and the 5V wire
    xw = (LM_BOX[1] + FIT + TOUCH, INNER_X)  # the right wall
    yr = (TDH_REAR_Y0, LM_BOX[2] - 4.0 - TOUCH)  # the rear wall, behind the LM2596's input route
    zf = POCKET_Z + TOUCH
    feet: list[Foot] = [(*xw, yr[0], LM_BOX[3] - TOUCH, 0.0), (x0, xw[1], *yr, 0.0)]
    shape = foot_base(feet[0], pz0 + EMBED) + foot_base(feet[1], pz0 + EMBED)
    # The right wall stands clear of the LM2596's rear standoff.
    sx, sy = lm(*M.LM2596_HOLES[0])
    r = LM_BOSS_D / 2 + FIT
    shape = shape - box(xw[0] - 1.0, sx + r, sy - r, sy + r, zf - 1.0, LM_BOX[4] + FIT)
    # The plate, short of the 5V wire up to the board's pin.
    shape = shape + box(x0, xw[1], yr[0], pcb[3] + FIT - TOUCH, pz0, pz1)
    shape = shape - box(x_front[1], xw[1] + 1.0, LM_BOX[3] - TOUCH, LM_BOX[3] + 6.1 + TOUCH, pz0 - 1.0, pz1 + 1.0)
    # The rear block, its front face the rear lip.
    shape = shape + box(x0, xw[1], yr[0], pcb[2] - FIT, pz1 - EMBED, top)
    # The pads: their tops FIT inside the bare spots, their rear faces at 45 degrees.
    for b in M.TD_BARE[1:]:
        bx0, bx1, by0, by1, _, _ = td_box(*b, 0, 0)
        y0, y1 = by0 + FIT, by1 - FIT
        shape = shape + yz_prism([(y0, face), (y1, face), (y1, pz1 - EMBED), (y0 + face - pz1 + EMBED, pz1 - EMBED)],
                                 bx0 + FIT, bx1 - FIT)
    # The end pad and the front lip: across the end's bare strip, the lip's
    # inner face FIT beyond the board's end, and its foot sloping under it.
    ex0, ex1, ey0, ey1, _, _ = td_box(*M.TD_BARE[0], 0, 0)
    y0, y1 = ey0 + FIT, pcb[3] - FIT
    lip0, lip1 = pcb[3] + FIT, pcb[3] + FIT + TDH_LIP_T
    notch = lip0 - y1  # down at 45 degrees from the pad's front edge to the lip's inner face
    shape = shape + yz_prism([(y0 + face - pz1 + EMBED, pz1 - EMBED), (y0, face), (y1, face), (lip0, face - notch),
                              (lip0, top), (lip1, top), (lip1, pz1 - EMBED)], x_front[0], x_front[1])
    return Part("t-display-holder", shape & cavity(TOUCH), "holder", (0.0, 1.0, 0.0), "white",
                held_down=POCKET_DEPTH, carries=("t-display",)), feet


def yz_prism(pts, x0: float, x1: float) -> Solid:
    """A prism of a polygon in (y, z), from x0 to x1, whichever way round the polygon runs."""
    plane = Plane(origin=(x0, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    return extrude(plane * Polygon(*pts, align=None), amount=x1 - x0, dir=(1, 0, 0)).solid()


def td_support() -> Support:
    """Where the holder bears on the board: its bare spots."""
    return Support("t-display-holder", "t-display", TD_Z0, tuple(td_box(*b, 0, 0)[:4] for b in M.TD_BARE))


def intake_fan() -> Fan:
    return Fan(FANS[1], (INTAKE_X0, (INTAKE_Y0 + INTAKE_Y1) / 2, INTAKE_Z0), (-1.0, 0.0, 0.0), exhaust=False,
               size=M.FAN_SIZE)


def exhaust_fan() -> Fan:
    vc = (EXH_V0 + EXH_V1) / 2
    return Fan(FANS[0], (*uv(EXH_U1, vc), EXH_Z0), (1 / SQRT2, 1 / SQRT2, 0.0), exhaust=True, size=M.FAN_SIZE)


def uv(u: float, v: float) -> tuple[float, float]:
    """A point in the front-right corner frame, in the robot's frame."""
    return (u - v) / SQRT2, (u + v) / SQRT2


# --------------------------------------------------------------------------- the cover


def cavity(margin: float = 0.0) -> Solid:
    """The space inside the cover, `margin` in from its inside faces, from
    below the deck up to the lid."""
    w = WALL + margin
    inner = octagon(BOX_X - w, BOX_Y - w, BOX_DIAG - w * SQRT2, COVER_Z0 - 10.0, LID_Z0 - margin)
    return inner.chamfer(COVER_INNER_CHAMFER, None, top_edges(inner, LID_Z0 - margin))


def top_edges(solid: Solid, z: float):
    return [e for e in solid.edges() if abs(e.bounding_box().min.Z - z) < 1e-6 and abs(e.bounding_box().max.Z - z) < 1e-6]


def vent_slots(span_c: float, z0: float, z1: float, cut: tuple[float, float], turn: float) -> Solid:
    """VENT_SLOTS vertical slots centred on span_c along the wall, cut from
    cut[0] to cut[1] across it, in a frame turned `turn` (x across the wall)."""
    out = None
    for i in range(VENT_SLOTS):
        a = span_c - VENT_SPAN / 2 + i * VENT_PITCH
        s = turned((cut[0], cut[1], a, a + VENT_SLOT_W, z0, z1), turn)
        out = s if out is None else out + s
    return out


def inlet_span() -> tuple[float, float, float, float]:
    """The inlet grille's (y0, y1, z0, z1) on the left wall, centred on the intake fan."""
    yc, zc = (INTAKE_Y0 + INTAKE_Y1) / 2, INTAKE_Z0 + M.FAN_SIZE[1] / 2
    return yc - VENT_SPAN / 2, yc + VENT_SPAN / 2, zc - VENT_IN_H / 2, zc + VENT_IN_H / 2


def outlet_span() -> tuple[float, float, float, float]:
    """The outlet grille's (v0, v1, z0, z1) on the front-right corner face, centred on the exhaust fan."""
    vc, zc = (EXH_V0 + EXH_V1) / 2, EXH_Z0 + M.FAN_SIZE[1] / 2
    return vc - VENT_SPAN / 2, vc + VENT_SPAN / 2, zc - VENT_OUT_H / 2, zc + VENT_OUT_H / 2


def vents() -> list[Vent]:
    """The grilles, as the wall regions they occupy."""
    iy0, iy1, iz0, iz1 = inlet_span()
    ov0, ov1, oz0, oz1 = outlet_span()
    area_in = VENT_SLOTS * VENT_SLOT_W * VENT_IN_H
    area_out = VENT_SLOTS * VENT_SLOT_W * VENT_OUT_H
    return [Vent("inlet", box(-BOX_X, -(BOX_X - WALL), iy0, iy1, iz0, iz1), area_in),
            Vent("outlet", turned(((BOX_DIAG - WALL * SQRT2) / SQRT2, BOX_DIAG / SQRT2, ov0, ov1, oz0, oz1), DIAG),
                 area_out)]


def letter(ch: str, x: float, y: float) -> Solid:
    out = None
    for x0, y0, x1, y1 in LETTERS[ch]:
        s = box(x + x0 * NAME_SCALE, x + x1 * NAME_SCALE, y + y0 * NAME_SCALE, y + y1 * NAME_SCALE,
                LID_Z1 - INLAY_DEPTH, LID_Z1)
        out = s if out is None else out + s
    return out


def inlays():
    """The name and the chevrons, flush in the lid's top."""
    width = (len(NAME) - 1) * NAME_ADVANCE + 6 * NAME_SCALE
    x = NAME_CENTRE[0] - width / 2
    y = NAME_CENTRE[1] - 5 * NAME_SCALE
    out = None
    for i, ch in enumerate(NAME):
        s = letter(ch, x + i * NAME_ADVANCE, y)
        out = s if out is None else out + s
    k = INLAY_STROKE * SQRT2  # a 45-degree stroke's height
    a = CHEVRON_ARM
    for cx, cy in CHEVRONS:
        pts = [(cx - a, cy), (cx, cy + a), (cx + a, cy), (cx + a, cy - k), (cx, cy + a - k), (cx - a, cy - k)]
        face = Plane.XY.offset(LID_Z1 - INLAY_DEPTH) * Polygon(*pts, align=None)
        out = out + extrude(face, amount=INLAY_DEPTH, dir=(0, 0, 1)).solid()
    return out


def window_cutter() -> Solid:
    """The window: its opening at the lid's underside (window_rects), widening
    at 45 degrees to the top, except at the notch's outer end, which stays
    straight so the bevel does not run into the top chamfer."""
    rects = window_rects()
    (x0, x1, y0, y1), notches = rects[0], rects[1:]
    pts = [(x0, y0), (x1, y0)]
    edge = x1
    for nx0, nx1, ny0, ny1 in sorted(notches, key=lambda r: r[2]):
        if nx0 >= x1 - 1e-9:
            pts += [(x1, ny0), (nx1, ny0), (nx1, ny1), (x1, ny1)]
            edge = max(edge, nx1)
    pts += [(x1, y1), (x0, y1)]
    outline = Polygon(*pts, align=None)
    taper = extrude(Plane.XY.offset(LID_Z0) * outline, amount=LID_T + 0.01, taper=-45)
    below = extrude(Plane.XY.offset(LID_Z0 - 1.0) * outline, amount=1.0)
    return (taper + below) & box(-200, edge, -200, 200, 0, 200)


def cover() -> Part:
    """The cover: a faceted shell standing on the deck's rim, its top edge
    chamfered, a groove round its walls, the bumpers' rails on its corner faces, the window over the T-Display, the
    vents, the XT60's notch, the screw holes, hold-down blocks over the
    SZBK07's corners, and the flush inlays. It prints lid-down."""
    outer = octagon(BOX_X, BOX_Y, BOX_DIAG, COVER_Z0, LID_Z1)
    outer = outer.chamfer(COVER_CHAMFER, None, top_edges(outer, LID_Z1))
    shape = outer - cavity()
    d = GROOVE_DEPTH
    shape = shape - (octagon(BOX_X + 1, BOX_Y + 1, BOX_DIAG + SQRT2, *GROOVE_Z)
                     - octagon(BOX_X - d, BOX_Y - d, BOX_DIAG - d * SQRT2, GROOVE_Z[0] - 1, GROOVE_Z[1] + 1))
    shape = shape + rails() - rail_pilots()
    for xa, xb in ((SZ_BOX[0], SZ_BOX[0] + SZ_HOLD[0]), (SZ_BOX[1] - SZ_HOLD[0], SZ_BOX[1])):
        for ya, yb in ((SZ_BOX[2], SZ_BOX[2] + SZ_HOLD[1]), (SZ_BOX[3] - SZ_HOLD[1], SZ_BOX[3])):
            shape = shape + box(xa, xb, ya, yb, SZ_BOX[5] + SZ_HOLD_GAP, LID_Z0 + EMBED)
    shape = shape - window_cutter()
    for x, y in COVER_SCREWS:
        shape = shape - cyl(M3_D / 2, x, y, LID_Z0 - 1, LID_Z1 + 1)
    iy0, iy1, iz0, iz1 = inlet_span()
    shape = shape - vent_slots(-(iy0 + iy1) / 2, iz0, iz1, (BOX_X - WALL - 1, BOX_X + 1), 180)  # turned to face -x
    ov0, ov1, oz0, oz1 = outlet_span()
    shape = shape - vent_slots((ov0 + ov1) / 2, oz0, oz1, (EXH_U1 + 0.1, BOX_DIAG / SQRT2 + 1), DIAG)
    ny0, ny1, nz1 = XT_NOTCH
    shape = shape - box(-BOX_X - 1, -(BOX_X - WALL) + 0.5, ny0, ny1, COVER_Z0 - 1, nz1)
    marks = inlays()
    return Part("cover", shape - marks, "cover", DOWN, "black", inlays=marks)


# --------------------------------------------------------------------------- bumpers (task 5.1)

# Each corner face carries two vertical rails, a dovetail near each end of
# the face and clear of the outlet's grille in the front-right one; a yellow
# PETG bumper slides down onto them. The bumper is a hook round each rail,
# joined by a curved leaf that stands off the face and flexes in a knock;
# the hooks stand past the side walls and the bumper's top above the lid.
# A knock that flattens the leaf pushes its ends apart, so only the -s hook
# is fixed (screwed to its rail); the +s hook's slot is FLOAT_SLOT longer
# towards -s, so that hook slides outwards along its rail's dovetail, which
# still keeps it on. The rails' tops are 45-degree wedges flush with the top
# chamfer, and the bumper's slots end in the same slope, which is its stop.
# Every corner's bumper is the same part, turned: the corner faces are
# alike. Source: chosen (design.md "Bumpers", "Assumptions", task group 5).
#
# In a corner's frame: s along the face, n out of it, z up; the face's centre
# at the origin (corner_location()).
CORNER_HALF = (BOX_X + BOX_Y - BOX_DIAG) * SQRT2 / 2  # half the corner face's length
CORNER_CENTRE = ((BOX_X + BOX_DIAG - BOX_Y) / 2, (BOX_DIAG - BOX_X + BOX_Y) / 2)  # the front-right face's centre
RAIL_S = 21.0  # each rail's centre along the face, its neck on the face. Source: chosen.
RAIL_NECK = 3.0  # the dovetail's width at the face. Source: chosen.
RAIL_HEAD = 6.4  # its width at its outer face. Source: chosen, so the free hook's longer slot still hooks on.
RAIL_D = 3.0  # how far it stands off the face. Source: chosen.
RAIL_Z0 = 40.0  # its foot, above the groove. Source: chosen.
RAIL_TOP = LID_Z1 - COVER_CHAMFER  # where the top chamfer begins: the wedge ends flush with it
SLIDE = 0.2  # sliding fit round the rail, PETG on PETG, normal to each face. Source: chosen, above the skill's 0.15 for TPU.
HOOK_WALL = 1.2  # the hook's wall beside the rail's head. Source: chosen, three perimeters.
HOOK_BACK = 1.6  # its wall outside the rail. Source: chosen.
BUMPER_Z0 = 44.0  # the hooks' foot. Source: chosen, 25 mm along the rails.
BUMPER_RISE = 3.5  # how far the bumpers' tops stand above the lid's. Source: chosen, above the cover's screw heads.
BUMPER_Z1 = LID_Z1 + BUMPER_RISE
LEAF_T = 1.5  # Source: design.md "Bumpers" (1.2 to 1.6 mm).
LEAF_H = 12.0  # the leaf's height, down from the bumper's top, above most of the outlet's grille. Source: chosen.
LEAF_APEX = 8.0  # the leaf's outer face off the corner face at its middle. Source: chosen, inside the keep-clear radius.
LEAF_POINTS = 41
FLOAT_SLOT = 1.0  # how much longer the free hook's slot is. Source: chosen, above leaf_travel() (about 0.65).
# An M2 x 6 self-tapping screw through the fixed hook into its rail keeps the
# bumper on its rails when the robot is turned over or shaken; it comes out
# from outside to change the bumper. At this height, under the leaf; the
# rail's pilot reaches into the cover's wall, leaving this much of it.
# Source: chosen; the pilot as the IMU's (PILOT_M2_D), the clearance the
# skill's M2.5 clearance (2.9) less 0.5.
RETAIN_Z = 52.0
RETAIN_SCREW = (2.0, 6.0)  # its shank's diameter and length
RETAIN_CLEAR_D = 2.4
RETAIN_WALL_LEFT = 0.6


def corner_location(k: int) -> Location:
    """Corner k's frame (0 front-right, 1 front-left, 2 rear-left, 3 rear-right)."""
    cx, cy = CORNER_CENTRE
    sx, sy = ((1, 1), (-1, 1), (-1, -1), (1, -1))[k]
    return Location((sx * cx, sy * cy, 0), (0, 0, (-45.0, 45.0, 135.0, -135.0)[k]))


def rail_profile(s0: float, grow: float = 0.0, n0: float = -EMBED, longer: float = 0.0) -> list[tuple[float, float]]:
    """The dovetail's outline in (s, n), from n0 to its outer face, grown by
    `grow` normal to each side, and swept `longer` towards -s (a free
    hook's slot)."""
    k = (RAIL_HEAD - RAIL_NECK) / 2 / RAIL_D  # its half-width grows k per mm out
    shift = grow * math.sqrt(1 + k * k)  # a flank moved `grow` normal to itself, along s

    def half(n: float) -> float:
        return RAIL_NECK / 2 + k * n + shift

    n1 = RAIL_D + grow
    return [(s0 - longer - half(n0), n0), (s0 + half(n0), n0), (s0 + half(n1), n1), (s0 - longer - half(n1), n1)]


def xy_prism(pts, z0: float, z1: float) -> Solid:
    """A prism of a polygon in (x, y), from z0 up to z1, whichever way round the polygon runs."""
    return extrude(Plane.XY.offset(z0) * Polygon(*pts, align=None), amount=z1 - z0, dir=(0, 0, 1)).solid()


def below_wedge(offset: float = 0.0) -> Solid:
    """Everything in a corner's frame with n + z at most RAIL_TOP + offset:
    under the rails' 45-degree tops (offset 0), or the bumper's slots' (its
    SLIDE normal to them)."""
    t = RAIL_TOP + offset
    return yz_prism([(-10.0, 0.0), (20.0, 0.0), (20.0, t - 20.0), (-10.0, t + 10.0)], -40.0, 40.0)


def at_corners(one: Solid) -> Solid:
    """A solid given in a corner's frame, at all four corners."""
    out = None
    for k in range(4):
        r = one.moved(corner_location(k))
        out = r if out is None else out + r
    return out


def rails() -> Solid:
    """The four corner faces' rails, on the cover."""
    one = None
    for s0 in (-RAIL_S, RAIL_S):
        r = xy_prism(rail_profile(s0), RAIL_Z0, RAIL_TOP + 1.0) & below_wedge()
        one = r if one is None else one + r
    return at_corners(one)


def rail_pilots() -> Solid:
    """The retaining screws' pilots, through each fixed hook's rail into the cover's
    wall: horizontal as the cover prints, so teardrops pointing down (up as
    printed)."""
    n0, n1 = -(WALL - RETAIN_WALL_LEFT), RAIL_D + 0.5
    return at_corners(teardrop(PILOT_M2_D / 2, n1 - n0, -RAIL_S, (n0 + n1) / 2, RETAIN_Z, "y", DOWN))


def bumper_screws() -> list[Screw]:
    """Each bumper's retaining screw: the head on the fixed hook's outer
    face, driven in along -n from outside."""
    found = []
    n = RAIL_D + SLIDE + HOOK_BACK
    for k in range(4):
        a = math.radians((-45.0, 45.0, 135.0, -135.0)[k])
        cx, cy = (CORNER_CENTRE[0] * (1, -1, -1, 1)[k], CORNER_CENTRE[1] * (1, 1, -1, -1)[k])
        out = (-math.sin(a), math.cos(a), 0.0)  # n in the robot's frame
        x = cx - RAIL_S * math.cos(a) + n * out[0]
        y = cy - RAIL_S * math.sin(a) + n * out[1]
        found.append(Screw(f"bumper {k + 1} screw", (x, y, RETAIN_Z), out, shank=RETAIN_SCREW))
    return found


def leaf_arc() -> tuple[float, float, float]:
    """The leaf's outer face: its radius, its centre's n, and its half-angle."""
    sag = LEAF_APEX - (RAIL_D + SLIDE + HOOK_BACK)
    radius = (RAIL_S ** 2 + sag ** 2) / (2 * sag)
    return radius, LEAF_APEX - radius, math.asin(RAIL_S / radius)


def leaf_travel() -> float:
    """How far the free hook moves when the leaf is pressed flat: its centre
    line's arc less its chord."""
    radius, _, a = leaf_arc()
    r = radius - LEAF_T / 2
    return 2 * a * r - 2 * r * math.sin(a)


def leaf_profile() -> list[tuple[float, float]]:
    """The leaf: a circular band between the hooks, its outer face through
    the hooks' outer faces at the rails and LEAF_APEX out at its middle."""
    radius, cn, a = leaf_arc()
    angles = [-a + 2 * a * i / (LEAF_POINTS - 1) for i in range(LEAF_POINTS)]
    outer = [(radius * math.sin(t), cn + radius * math.cos(t)) for t in angles]
    inner = [((radius - LEAF_T) * math.sin(t), cn + (radius - LEAF_T) * math.cos(t)) for t in reversed(angles)]
    return outer + inner


def bumper() -> Part:
    """The front-right corner's bumper, copied to the other three corners. It
    prints on its top (up -z): the leaf hangs from the top, and the slots'
    sloped ends face up as printed."""
    half = RAIL_HEAD / 2 + SLIDE + HOOK_WALL
    ne = RAIL_D + SLIDE + HOOK_BACK
    shape = None
    for s0, longer in ((-RAIL_S, 0.0), (RAIL_S, FLOAT_SLOT)):
        hook = box(s0 - longer - half, s0 + half, SLIDE, ne, BUMPER_Z0, BUMPER_Z1)
        slot = xy_prism(rail_profile(s0, SLIDE, -1.0, longer), BUMPER_Z0 - 1.0, BUMPER_Z1 + 1.0) \
            & below_wedge(SLIDE * SQRT2)
        hook = hook - slot
        if not longer:
            hook = hook - teardrop(RETAIN_CLEAR_D / 2, HOOK_BACK + 2.0, s0, ne - HOOK_BACK / 2, RETAIN_Z, "y", DOWN)
        shape = hook if shape is None else shape + hook
    shape = shape + xy_prism(leaf_profile(), BUMPER_Z1 - LEAF_H, BUMPER_Z1)
    first = corner_location(0)
    copies = tuple(corner_location(k) * first.inverse() for k in (1, 2, 3))
    s1 = RAIL_S - FLOAT_SLOT - half - 1.0  # the free end: its hook and the leaf's last millimetre
    region = box(s1, s1 + 40.0, -5.0, LEAF_APEX + 5.0, BUMPER_Z0 - 5.0, BUMPER_Z1 + 5.0).moved(first)
    a = math.radians(-45.0)
    floating = Floating(region, (math.cos(a), math.sin(a), 0.0), (-math.sin(a), math.cos(a), 0.0), leaf_travel())
    return Part("bumper", shape.moved(first), "bumper", DOWN, "yellow", copies=copies, floating=floating)


def channel() -> AirChannel:
    """The passive air path: from the inlet (the space inside its grille, out
    to the intake fan's inner face) over the fuse holder, up and across over
    the SZBK07, forward over the board, right in front of the T-Display, and
    into the front-right corner to the outlet (the exhaust fan's space behind
    its grille). Every box keeps FIT + TOUCH from what it passes."""
    c = FIT + TOUCH
    iy0, iy1, iz0, iz1 = inlet_span()
    ov0, ov1, oz0, oz1 = outlet_span()
    top = LID_Z0 - FIT
    over_sz = SZ_BOX[5] + c
    over_board = S2_TOP + c
    x_hold = SZ_BOX[1] - SZ_HOLD[0] - 0.5  # clear of the hold-down blocks at the SZBK07's right corners
    y_hold = (SZ_BOX[2] + SZ_HOLD[1] + FIT, SZ_BOX[3] - SZ_HOLD[1] - FIT)
    td_front = td_box(M.TD_PCB[0], M.TD_PCB[1], M.TD_PCB[2], M.TD_PCB[3], 0, 0)[3]
    return AirChannel(
        segments=(
            ((INTAKE_X1 - 0.2, SZ_BOX[0] - 0.5, -38.0, INTAKE_Y1 - TOUCH, FUSE_BOX[5] + c, top), "x"),  # over the fuse holder
            ((SZ_BOX[0] - 3.0, x_hold, *y_hold, over_sz, top), "x"),                # over the SZBK07
            ((-5.0, x_hold, y_hold[0], 0.0, over_sz, top), "y"),                    # turning forward
            ((-5.0, x_hold, SZ_BOX[3] + 0.5, 47.0, over_board, top), "y"),          # over the board
            ((-5.0, 40.0, td_front + c, 47.0, over_board, top), "x"),               # in front of the T-Display
            ((30.0, EXH_U0 + 0.13, -5.0, EXH_V1 - EXH_LIP_OVER - FIT, over_board, top), "x", DIAG),  # into the corner
        ),
        inlet=(-(BOX_X - WALL) + TOUCH, INTAKE_X1 + FIT - TOUCH, iy0, iy1, iz0, iz1),
        outlet=(EXH_U0, EXH_U1 + 0.25, ov0, ov1, oz0, oz1),
        inlet_open_area=VENT_SLOTS * VENT_SLOT_W * VENT_IN_H,
        past=SZBK07,
        outlet_turn=DIAG,
    )


def pilots():
    """((x, y), pilot diameter, standoff top) for every self-tapping screw
    into the deck: the board's standoffs. (The LM2596's and the IMU's screws
    go into their white holders, lm_standoffs() and imu_pad().)"""
    return [(s2(*h), PILOT_M25_D, S2_Z0 - TOUCH) for h in M.S2040_HOLES]


def features() -> Solid:
    """Everything on the deck's top, sunk EMBED into it."""
    z = DECK_Z1 - EMBED
    parts = []
    for (x, y), _, top in pilots():
        parts.append(cyl(S2_STANDOFF_D / 2, x, y, z, top))
    # SZBK07: L-stops at the rear corners, ribs in front of the front corners.
    sx0, sx1, sy0, sy1 = SZ_BOX[:4]
    zt = DECK_Z1 + STOP_H
    # Blocks rather than L-shapes: the corner itself would reach the cover's
    # corner face.
    for xa, xb, rear in ((sx0 - FIT - STOP_T, sx0 - FIT, (sx0 + 1.0, sx0 + 9.0)),
                         (sx1 + FIT, sx1 + FIT + STOP_T, (sx1 - 9.0, sx1 - 1.0))):
        parts.append(box(xa, xb, sy0 + 2.5, sy0 + 9.5, z, zt))
        parts.append(box(*rear, sy0 - FIT - STOP_T, sy0 - FIT, z, zt))
    for xa, xb in ((sx0 + 1.0, sx0 + 7.0), (sx1 - 7.0, sx1 - 1.0)):
        parts.append(box(xa, xb, sy1 + FIT, sy1 + FIT + RIB_T, z, DECK_Z1 + RIB_H))
    # XT60: side walls out to the inner wall as a hood, back stops beside the cups.
    bx0, bx1, by0, by1 = XT_BODY[:4]
    wz = XT_Z0 + XT_WALL_H
    for ya, yb in ((by0 - FIT - XT_WALL, by0 - FIT), (by1 + FIT, by1 + FIT + XT_WALL)):
        parts.append(box(-INNER_X, bx1 + TOUCH + XT_STOP, ya, yb, z, wz))
    for ya, yb in ((by0 - FIT, XT_CUPS[2] - FIT), (XT_CUPS[3] + FIT, by1 + FIT)):
        parts.append(box(bx1 + TOUCH, bx1 + TOUCH + XT_STOP, ya, yb, z, wz))
    # The cover's posts, up to just under the lid, kept inside the cover.
    inside = cavity(TOUCH)
    for x, y in COVER_SCREWS:
        parts.append(cyl(POST_D / 2, x, y, z, LID_Z0 - TOUCH) & inside)
    out = parts[0]
    for p in parts[1:]:
        out = out + p
    return out


def spacer() -> Part:
    """One spacer under the deck at a floor insert, copied to the other three."""
    (x, y), *rest = M.FRAME_FLOOR_INSERTS
    shape = cyl(SPACER_OD / 2, x, y, SPACER_Z0, SPACER_Z1) - cyl(SPACER_ID / 2, x, y, SPACER_Z0 - 1, SPACER_Z1 + 1)
    copies = tuple(Location((ox - x, oy - y, 0)) for ox, oy in rest)
    return Part("spacer", shape, "holder", UP, "white", copies=copies)


# --------------------------------------------------------------------------- screws


def screws() -> list[Screw]:
    """Every screw's head and its screwdriver's way in. The deck's screws on
    the floor inserts lie under the SZBK07 and the board, so they go in first."""
    found = []
    for x, y in M.FRAME_RING_INSERTS:
        found.append(Screw(f"deck screw ({x}, {y})", (x, y, DECK_Z1 - CBORE_DEPTH + BUTTON_HEAD_H), UP, after=("cover",)))
    for x, y in M.FRAME_FLOOR_INSERTS:
        found.append(Screw(f"deck screw ({x}, {y})", (x, y, DECK_Z1 - CBORE_DEPTH + BUTTON_HEAD_H), UP,
                           after=(SZBK07, SERVO_2040, "cover")))
    for i, h in enumerate(M.S2040_HOLES):
        found.append(Screw(f"servo 2040 screw {i + 1}", (*s2(*h), S2_Z1 + 1.8), UP, after=("cover",)))
    for i, h in enumerate(M.LM2596_HOLES):
        found.append(Screw(f"lm2596 screw {i + 1}", (*lm(*h), LM_BOX[4] + MODULE_PCB_T + 1.8), UP,
                           after=("t-display", "t-display-holder", "cover")))
    for i, (x, y) in enumerate(IMU_BOSSES):
        found.append(Screw(f"lsm6dso screw {i + 1}", (x, y, IMU_BOX[4] + MODULE_PCB_T + 1.2), UP,
                           after=(SERVO_2040, "cover")))
    for i, (x, y) in enumerate(COVER_SCREWS):
        found.append(Screw(f"cover screw {i + 1}", (x, y, LID_Z1 + BUTTON_HEAD_H), UP))
    return found + bumper_screws()


# --------------------------------------------------------------------------- the assembly


def assembly() -> Assembly:
    t = s2_terminals()
    fans = [exhaust_fan(), intake_fan()]
    held = [szbk07(), servo2040(), lm2596(), t_display(), imu(), xt60(), fuse(), diode()]
    held += [Held(f.name, f.shape()) for f in fans]
    holders = [part for part, _ in loose_holders()]
    return Assembly(
        parts=[deck(), spacer(), cover(), bumper(), *holders],
        held=held,
        routes=routes(t),
        openings=openings(),
        screws=screws(),
        channel=channel(),
        imu_pad=ImuPad("imu-pad", IMU_BOSSES, IMU_BOX[4] - TOUCH, IMU_BOSS_D / 2, PILOT_M2_D / 2),
        vents=vents(),
        fans=fans,
        supports=[td_support()],
    )
