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
  it stays inside the legs' keep-clear region.

Sizes come from measurements.py; the numbers here are layout choices, each
with its reason.
"""

from __future__ import annotations

import math

from build123d import Location, Solid

import measurements as M
from model import (IMU, J6, LM2596_OUTPUT_ROUTE, SCHOTTKY, SENSOR_HEADER, SERVO_2040, SIDES, SZBK07, Assembly, Box6, Exposure, Held, ImuPad,
                   Opening, Part, Route, Screw, Terminal, box, box6)

UP = (0.0, 0.0, 1.0)

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


def s2(bx: float, by: float) -> tuple[float, float]:
    """A point on the Servo 2040's drawing, in the robot's frame."""
    return S2_X0 + bx, S2_Y0 + by


# LM2596 on the right behind the corridor, its length along y: input pads at
# the rear end, output pads at the front. On bosses so its underside leads
# clear the deck. Source: chosen.
LM_X0 = 39.5
LM_Y0 = -27.0
LM_BOSS_H = 3.0  # Source: chosen.
LM_BOSS_D = 6.0  # Source: chosen.
LM_BOX = (LM_X0, LM_X0 + M.LM2596_SIZE[1], LM_Y0, LM_Y0 + M.LM2596_SIZE[0],
          DECK_Z1 + LM_BOSS_H + TOUCH, DECK_Z1 + LM_BOSS_H + TOUCH + M.LM2596_SIZE[2])
LM_PAD_DEPTH = 2 * M.LM2596_PAD_INSET  # the pad region at each end


def lm(mx: float, my: float) -> tuple[float, float]:
    """A point on the LM2596 (length along x, width along y, from its corner
    by the input pads), in the robot's frame: length along +y, width along -x."""
    return LM_X0 + M.LM2596_SIZE[1] - my, LM_Y0 + mx


# T-Display above the LM2596, screen up, its long side along y. Its PCB's
# underside clears a holder plate over the LM2596 (task 5.2) by its tallest
# underside part. Source: chosen.
TD_X0 = 36.7
TD_Y0 = -(M.TD_PCB[3] - M.TD_PCB[2]) / 2
TD_HOLDER_PLATE = 2.0  # Source: chosen, the holder's plate (task 5.2).
TD_Z0 = LM_BOX[5] + FIT + TD_HOLDER_PLATE + M.TD_UNDERSIDE_TALLEST + 0.1  # PCB underside


def td(x: float, y: float) -> tuple[float, float]:
    """A point in the T-Display model's frame, turned screen up (a half turn
    about its y axis: x to -x), in the robot's frame."""
    return TD_X0 - x, TD_Y0 + (y - M.TD_PCB[2])


def td_box(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> Box6:
    """A box in the model's frame (z up from the PCB's screen face, the
    model's -z), in the robot's frame."""
    (ax, ay), (bx, by) = td(x0, y0), td(x1, y1)
    return (min(ax, bx), max(ax, bx), min(ay, by), max(ay, by), TD_Z0 + z0, TD_Z0 + z1)


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
    return Held(SZBK07, box6(SZ_BOX))


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
    return Terminal(f"lm2596 {end}", "lm2596", (LM_BOX[0], LM_BOX[1], y0, y1, LM_BOX[4], LM_BOX[5]), side)


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
    return Held("t-display", shape, exposed=tuple(exposed))


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
    lm_in = (COR_X[0], LM_BOX[1], LM_BOX[2] - 4.0, LM_BOX[2] - TOUCH, RZ0, LM_BOX[5])
    # Up to the inner header row, clear of the LM2596's boss beside it.
    td_link = (COR_X[0], td_terminal(1, "x").region[1] + 0.02, *td_terminal(1, "x").region[2:4], RZ0, cz1)
    out_front = (COR_X[0], LM_BOX[1], LM_BOX[3] + TOUCH, 28.6, RZ0, LM_BOX[5])
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
    the insert holes, slots, entries and tie holes through it, and on it the
    stops, standoffs, bosses, the IMU pad and the XT60's cradle."""
    z0, z1 = DECK_Z0, DECK_Z1
    shape = octagon(BOX_X, BOX_Y, BOX_DIAG, z0, z1) + box(-TAB_X, TAB_X, -TAB_Y, TAB_Y, z0, z1)
    shape = shape + features()
    for x, y in M.FRAME_INSERTS:
        shape = shape - cyl(M3_D / 2, x, y, z0 - 1, z1 + 1) - cyl(CBORE_D / 2, x, y, z1 - CBORE_DEPTH, z1 + 1)
    for o in openings():
        shape = shape - box(*o.footprint, z0 - 1, z1 + 1)
    for (x, y), d, top in pilots():
        shape = shape - cyl(d / 2, x, y, top - 5.0, top + 1)
    return Part("deck", shape, "deck", UP, "black")


def pilots():
    """((x, y), pilot diameter, boss top) for every self-tapping screw into
    the deck: the board's standoffs, the LM2596's and the IMU's bosses."""
    found = [(s2(*h), PILOT_M25_D, S2_Z0 - TOUCH) for h in M.S2040_HOLES]
    found += [(lm(*h), PILOT_M25_D, LM_BOX[4] - TOUCH) for h in M.LM2596_HOLES]
    found += [(b, PILOT_M2_D, IMU_BOX[4] - TOUCH) for b in IMU_BOSSES]
    return found


def features() -> Solid:
    """Everything on the deck's top, sunk EMBED into it."""
    z = DECK_Z1 - EMBED
    parts = []
    for (x, y), _, top in pilots():
        d = S2_STANDOFF_D if top == S2_Z0 - TOUCH else LM_BOSS_D if top == LM_BOX[4] - TOUCH else IMU_BOSS_D
        parts.append(cyl(d / 2, x, y, z, top))
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
                           after=("t-display", "t-display holder", "cover")))
    for i, (x, y) in enumerate(IMU_BOSSES):
        found.append(Screw(f"lsm6dso screw {i + 1}", (x, y, IMU_BOX[4] + MODULE_PCB_T + 1.2), UP,
                           after=(SERVO_2040, "cover")))
    return found


# --------------------------------------------------------------------------- the assembly


def assembly() -> Assembly:
    t = s2_terminals()
    held = [szbk07(), servo2040(), lm2596(), t_display(), imu(), xt60(), fuse(), diode()]
    return Assembly(
        parts=[deck(), spacer()],
        held=held,
        routes=routes(t),
        openings=openings(),
        screws=screws(),
        imu_pad=ImuPad("deck", IMU_BOSSES, IMU_BOX[4] - TOUCH, IMU_BOSS_D / 2, PILOT_M2_D / 2),
    )
