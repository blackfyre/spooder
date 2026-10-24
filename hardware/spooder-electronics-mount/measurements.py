"""Named measurements for the electronics mount, each with its source.

Every size the mount is built from lives here, as one named constant with its
source in a comment: a reference file in the repository, a datasheet or
reseller figure, or a measurement on the part. Lengths are millimetres.

Two kinds of value:

- Nominal values (task 1.0 of `add-electronics-mount`): sizes and wire sides
  not yet measured or checked on the parts. Each is named in `MEASURE_BEFORE_PRINTING` and listed in the
  README's "Measure before printing" table, with the value to replace once
  the part is measured (task 1.1).
- Reference data (task 1.2): values read from the repository's reference
  files. `reference_check.py` re-derives the ones from machine-readable files
  (the frame top STL, the T-Display STEP model and the Servo 2040 drawing's
  pixels) and fails when a constant no longer matches its file.

This module uses the standard library only, so the README test runs without
the CAD toolchain.
"""

# ---------------------------------------------------------------------------
# Nominal held-part sizes (task 1.0): replace with measured values (task 1.1).
# ---------------------------------------------------------------------------

# SZBK07 buck converter (servo supply), outline length x width x height.
# Source: nominal, reseller listings (about 60 x 53-54 x 27-28 mm by batch);
# the larger figures are taken so the mount errs towards room.
SZBK07_SIZE = (60.0, 54.0, 28.0)
# Its screw terminals sit on the two short ends (the 54 mm ends): input at
# one, output at the other. Source: nominal, reseller photos; assumed until
# measured.
SZBK07_TERMINALS_ON_SHORT_ENDS = True
# Top of its heatsinks above its board's underside. Source: nominal, taken as
# the full outline height until measured.
SZBK07_HEATSINK_TOP = 28.0

# LM2596 buck module (logic supply), outline length x width x height.
# Source: nominal, reseller listings of the common blue module.
LM2596_SIZE = (45.0, 20.0, 14.0)
# Its two mounting holes, on the diagonal, from the module's lower-left corner
# (length along x, width along y). Source: nominal, reseller photos; assumed.
LM2596_HOLES = ((7.0, 2.5), (38.0, 17.5))
# Mounting-hole diameter. Source: nominal, M3 clearance on the common module.
LM2596_HOLE_D = 3.0
# Input pads (IN+, IN-) at the x = 0 end and output pads (OUT+, OUT-) at the
# x = length end, each this far in from its end. Source: nominal, reseller
# photos; assumed.
LM2596_PAD_INSET = 2.5

# 30 x 30 x 10 mm fan (3010), outline. Source: nominal, the 3010 size.
FAN_SIZE = (30.0, 30.0, 10.0)
# Its mounting-hole spacing, square. Source: nominal, the usual 3010 spacing.
FAN_HOLE_SPACING = 24.0
# Its mounting-hole diameter. Source: nominal, the usual 3010 hole.
FAN_HOLE_D = 3.0
# Its leads leave at one corner of the frame. Source: nominal, assumed.
FAN_WIRE_EXIT_AT_CORNER = True

# LSM6DSO IMU breakout, outline length x width. Source: nominal; the breakout
# has not been chosen yet, so this stays nominal until one is found (task 1.1).
LSM6DSO_SIZE = (20.0, 15.0)
# Its two mounting holes' spacing, along its length, on the centreline.
# Source: nominal, assumed for a two-hole breakout.
LSM6DSO_HOLE_SPACING = 15.0
# Its mounting-hole diameter (M2). Source: nominal, assumed.
LSM6DSO_HOLE_D = 2.2

# Schottky diode (such as a 1N5819), a DO-41 body: length and diameter.
# Source: nominal, the DO-41 package outline (5.2 mm long, 2.7 mm across).
SCHOTTKY_BODY = (5.2, 2.7)
# The heat-shrink sleeve it is carried in, inline on the LM2596's output lead:
# its size before shrinking (6 mm) as the conservative envelope, and its length
# over the body and both solder joints. Source: nominal, assumed.
SCHOTTKY_SLEEVE_D = 6.0
SCHOTTKY_SLEEVE_LENGTH = 20.0  # Source: nominal, assumed (body plus 7.4 mm each side).

# A servo plug seated on a Servo 2040 header pin with its lead bent back: the
# height of plug and bend above the board's top face. Source: nominal, a
# 2.54 mm JR/Dupont housing about 14 mm tall on the header plus the lead's
# bend; assumed until measured.
SERVO_PLUG_HEIGHT = 20.0

# Added with the layout (task 3.1), nominal like the ones above.
# The SZBK07's screw terminals span this much of the middle of each short end,
# so a route reserves that span and the stops hold its corners. Source:
# nominal, reseller photos (a 4-way block about 25 mm long); assumed.
SZBK07_TERMINAL_SPAN = 30.0

# XT60 socket housing: length along the mating axis, width, height. Source:
# nominal, the Amass XT60 outline (about 16 x 15.5 x 8.2 mm); assumed until
# measured.
XT60_SIZE = (16.0, 15.5, 8.2)
# Its solder cups behind the housing, and the cups' width and height.
# Source: nominal, assumed.
XT60_CUP_LENGTH = 7.0
XT60_CUP_SECTION = (10.0, 6.0)  # Source: nominal, assumed (two cups side by side).
# The battery's plug mated in the socket, with its solder joints, beyond the
# socket's mating face. Source: nominal, assumed (an XT60 plug's housing less
# its overlap with the socket, plus its joints).
XT60_PLUG_LENGTH = 20.0

# The inline fuse holder (a blade-fuse holder on its leads): length between
# its lead exits, width, height. Source: nominal, assumed for an inline ATO
# holder.
FUSE_HOLDER_SIZE = (35.0, 14.0, 10.0)

# Servo 2040 board thickness, and how far its through-hole leads stand below
# its underside. Source: nominal, a standard 1.6 mm board; the leads assumed.
S2040_PCB_THICKNESS = 1.6
S2040_UNDERSIDE_LEADS = 2.0  # Source: nominal, assumed.

# A USB-C cable's plug, its overmould: width, height and length out of the
# receptacle. Source: nominal, a common cable (about 12 x 6.5 x 30 mm); assumed.
USB_C_PLUG = (12.5, 7.0, 30.0)

# The LSM6DSO breakout's height on its bosses: its board and parts. Source:
# nominal, a 1.6 mm board with the chip and passives; assumed.
LSM6DSO_HEIGHT = 3.0

# A servo plug's section, which must pass through a lead slot when the leads
# are threaded up from below. Source: nominal, a 3-way 2.54 mm servo plug
# (about 8 x 4.5 mm with its key); assumed.
SERVO_PLUG_SECTION = (8.0, 4.5)

# An end-stop or sensor wire's outside diameter. Source: nominal, 26 AWG
# hook-up wire with its insulation; assumed.
SENSOR_WIRE_D = 1.3

# How far above the LM2596's board's underside its wires stand where they are
# soldered to its pads: the 1.6 mm board and a wire lying on its pad. The
# routes to its pads rise no higher, under the exhaust fan holder's right lip
# 3.87 mm above the board's underside. Source: nominal, 22 AWG hook-up wire;
# assumed.
LM2596_WIRE_HEIGHT = 3.8

# The constants in this file that are nominal (the ones above, and the two
# Servo 2040 wire sides below), each listed in the README's "Measure before
# printing" table. Source: this file.
# Of those, the ones the build records but does not use: the fans are held
# by their frames, not screwed through their holes, and their leads are routed
# by hand (the holders' test print, task 7.2, checks both). Every other one
# sizes the layout, is read by a check, or is checked by
# layout.measurement_failures(). Source: this file, layout.py and checks.py.
REFERENCE_ONLY = ("FAN_HOLE_SPACING", "FAN_HOLE_D", "FAN_WIRE_EXIT_AT_CORNER")
MEASURE_BEFORE_PRINTING = (
    "SZBK07_SIZE",
    "SZBK07_TERMINALS_ON_SHORT_ENDS",
    "SZBK07_HEATSINK_TOP",
    "LM2596_SIZE",
    "LM2596_HOLES",
    "LM2596_HOLE_D",
    "LM2596_PAD_INSET",
    "FAN_SIZE",
    "FAN_HOLE_SPACING",
    "FAN_HOLE_D",
    "FAN_WIRE_EXIT_AT_CORNER",
    "LSM6DSO_SIZE",
    "LSM6DSO_HOLE_SPACING",
    "LSM6DSO_HOLE_D",
    "SCHOTTKY_BODY",
    "SCHOTTKY_SLEEVE_D",
    "SCHOTTKY_SLEEVE_LENGTH",
    "SERVO_PLUG_HEIGHT",
    "SZBK07_TERMINAL_SPAN",
    "XT60_SIZE",
    "XT60_CUP_LENGTH",
    "XT60_CUP_SECTION",
    "XT60_PLUG_LENGTH",
    "FUSE_HOLDER_SIZE",
    "S2040_PCB_THICKNESS",
    "S2040_UNDERSIDE_LEADS",
    "USB_C_PLUG",
    "LSM6DSO_HEIGHT",
    "SERVO_PLUG_SECTION",
    "SENSOR_WIRE_D",
    "LM2596_WIRE_HEIGHT",
    "S2040_EXT_WIRE_SIDE",
    "S2040_SENSOR_WIRE_SIDE",
)

# ---------------------------------------------------------------------------
# Frame top (task 1.2). Source: hardware/aecert-hexapod-v1/stl/Frame Top.stl,
# which is already in the robot's frame (mm, +y forward, centred on the body).
# ---------------------------------------------------------------------------

# Top of the raised ring. Source: Frame Top.stl, its upward faces' highest z.
FRAME_RING_Z = 25.75
# The recessed floor. Source: Frame Top.stl, its second level of upward faces.
FRAME_FLOOR_Z = 20.0
# The frame top's underside. Source: Frame Top.stl, its downward faces' z.
FRAME_BOTTOM_Z = 16.0

# The eight M3 heat-set insert holes, (x, y): four on the ring, four on the
# floor. Source: Frame Top.stl, centroids of the holes in its height map.
FRAME_RING_INSERTS = ((20.0, 73.0), (-20.0, 73.0), (20.0, -73.0), (-20.0, -73.0))
FRAME_FLOOR_INSERTS = ((12.86, 19.06), (-12.86, 19.06), (12.86, -19.06), (-12.86, -19.06))
FRAME_INSERTS = FRAME_RING_INSERTS + FRAME_FLOOR_INSERTS  # Source: the two above.
# Insert hole diameter. Source: Frame Top.stl (equivalent diameters 4.44 on the
# ring, 4.35 on the floor), the M3 insert's Ø4.4 hole.
FRAME_INSERT_HOLE_D = 4.4
# The ring's insert holes are blind, down to this z; the floor's run through.
# Source: Frame Top.stl, the hole floors in its height map.
FRAME_RING_INSERT_FLOOR_Z = 19.75

# Open space: rectangles (x0, x1, y0, y1) where the frame top has no material
# at all, so a tie or a slot over them has the battery bay's free space below.
# Each is the largest open rectangle found there on a 0.25 mm grid, inside the
# leg keep-clear region; each group is mirrored about both axes.
# Source: Frame Top.stl, its height map (reference_check.py re-checks them).
FRAME_OPEN_CENTRE = ((-6.0, 6.0, -6.0, 6.0),)
FRAME_OPEN_NECK = ((-7.25, 7.25, 21.25, 32.75), (-7.25, 7.25, -32.75, -21.25))  # Source: as above.
FRAME_OPEN_FRONT_REAR = ((-15.0, 15.0, 32.75, 56.0), (-15.0, 15.0, -56.0, -32.75))  # Source: as above.
FRAME_OPEN_SIDES = (  # Source: as above.
    (26.5, 47.75, 6.5, 28.75),
    (-47.75, -26.5, 6.5, 28.75),
    (26.5, 47.75, -28.75, -6.5),
    (-47.75, -26.5, -28.75, -6.5),
)
FRAME_OPEN = FRAME_OPEN_CENTRE + FRAME_OPEN_NECK + FRAME_OPEN_FRONT_REAR + FRAME_OPEN_SIDES  # Source: the four above.

# ---------------------------------------------------------------------------
# Servo 2040 (task 1.2), in the board's own frame: the drawing's top view,
# origin at the board's lower-left corner, +x along the 62 mm edge, +y up the
# drawing, components on top. Source: hardware/pimoroni-servo2040/
# servo-2040-drawing.png (dimensioned values), with the undimensioned ones
# scaled from the drawing at about 17.7 px/mm (reference_check.py re-checks those).
# ---------------------------------------------------------------------------

# Board outline. Source: drawing, 62.00 x 42.00.
S2040_BOARD = (62.0, 42.0)
# Height with its connectors. Source: Pimoroni's product page, 62 x 42 x 12 mm.
S2040_HEIGHT = 12.0
# Header pitch on every header. Source: drawing, the 0.1 inch grid.
S2040_PITCH = 2.54
# M2.5 mounting holes, 2.70 mm in from each edge. Source: drawing.
S2040_HOLES = ((2.7, 2.7), (59.3, 2.7), (2.7, 39.3), (59.3, 39.3))

# Servo header: 18 columns in three groups of six, three rows. The drawing's
# 13.60 and 17.40 dimensions run to the centres of the groups (the "+" marks
# between columns 3 and 4, 9 and 10, 15 and 16), not to a column.
S2040_SERVO_GROUP_CENTRES_X = (13.60, 31.00, 48.40)  # Source: drawing, 13.60 then 17.40 twice.
S2040_SERVO_COL1_X = 7.25  # Source: drawing, 13.60 - 2.5 x 2.54.
S2040_SERVO_GROUP_STEP = 17.40  # Source: drawing, column n to column n + 6.
# Row centres as y: the dimensioned row (3.90 below the top edge) is the "+"
# row; "-" is the row above it and "S" (signal) the row below, by the silk
# labels between columns 6 and 7. Source: drawing.
S2040_SERVO_ROW_GND_Y = 40.64  # Source: drawing, 42.00 - 3.90 + 2.54.
S2040_SERVO_ROW_POWER_Y = 38.10  # Source: drawing, 42.00 - 3.90.
S2040_SERVO_ROW_SIGNAL_Y = 35.56  # Source: drawing, 42.00 - 3.90 - 2.54.

# EXT terminal (servo power in), a two-way screw terminal near the right edge.
# Its centre is 14.00 below the top edge (drawing) and in line with the sensor
# header's middle column, 3.90 in from the right edge (scaled).
S2040_EXT_CENTRE = (58.10, 28.0)  # Source: drawing, (62.00 - 3.90, 42.00 - 14.00).
# Its two poles' spacing along y, "+" above "-" by the silk. Source: drawing,
# scaled (3.48 mm), the 3.5 mm terminal pitch.
S2040_EXT_POLE_PITCH = 3.5
# The side its wires enter from: the board's right edge (+x). Source: nominal,
# the spooder-printed-parts skill ("Wiring"); the drawing does not show the
# terminal's opening, so check it on the board.
S2040_EXT_WIRE_SIDE = "+x"

# Sensor header: three columns (from left: IN signal, 3V3, GND, by the silk)
# and six rows, sensor 1 at the bottom. Centre 3.90 in from the right edge and
# 16.30 up from the bottom edge.
S2040_SENSOR_CENTRE = (58.10, 16.30)  # Source: drawing, (62.00 - 3.90, 16.30).
S2040_SENSOR_COLS = 3  # Source: drawing.
S2040_SENSOR_ROWS = 6  # Source: drawing.
# The side its wires leave from: its pins stand vertical, and the right edge
# (+x), 1.36 mm from the outer column, is the nearest side clear of other
# headers. Source: nominal, assumed from the drawing (design.md); check it on
# the board.
S2040_SENSOR_WIRE_SIDE = "+x"

# J6: the 17-pin row along the bottom edge (5V, GND, 3V3, SDA, SCL, INT, GND,
# A0-A2, SWC, SWD, GND, RST, GND, BOOT, GND by the silk), pin 17 (5V) at the
# left and pin 1 (GND) at the right, matching the J6 pin list in the
# spooder-hardware-check skill. A1 (pin 9) is 34.81 from the left edge.
S2040_J6_PIN9_X = 34.81  # Source: drawing.
# Row centre above the bottom edge: the pads alternate 1.36 and 1.64 (a
# staggered press-fit footprint), 1.50 on average. Source: drawing, scaled
# (not dimensioned).
S2040_J6_ROW_Y = 1.5
# Pin n's x is S2040_J6_PIN9_X + (9 - n) x S2040_PITCH. Source: this file.
S2040_J6_PINS = 17
# USB-C on the left edge, its centre 21.00 up. Source: drawing.
S2040_USB_Y = 21.0

# ---------------------------------------------------------------------------
# T-Display (task 1.2), in its STEP model's frame: PCB x -25.16..0,
# y 0..51.48, its top face at z 0; the screen and buttons on the -z face, the
# ESP32 and the other parts on the +z face (the board's underside as fitted,
# screen up). Source: hardware/lilygo-t-display/3d_file/T-Display.step, its
# solids' bounding boxes (reference_check.py re-derives them).
# ---------------------------------------------------------------------------

# PCB outline (x0, x1, y0, y1) and thickness. Source: T-Display.step, the
# three PCB solids.
TD_PCB = (-25.16, 0.0, 0.0, 51.48)
TD_PCB_THICKNESS = 1.01  # Source: T-Display.step, PCB z -1.01..0.
# The screen module on the -z face, (x0, x1, y0, y1) and its depth below the
# PCB. Source: T-Display.step, its largest solid.
TD_SCREEN = (-21.44, -3.84, 10.65, 42.65)
TD_SCREEN_DEPTH = 1.56  # Source: T-Display.step, z -2.58..-1.02.
# The buttons on the -z face, (x0, x1, y0, y1), 2.03 deep. Source: T-Display.step.
TD_BUTTONS = ((-22.05, -19.28, 1.45, 5.44), (-5.56, -2.79, 1.30, 5.28), (-24.69, -22.35, 9.58, 14.17))
TD_BUTTON_DEPTH = 2.03  # Source: T-Display.step, z -3.05..-1.02.
# The tallest part on the +z face: a 3.4 mm part at x -15.46..-11.21,
# y 6.35..10.55, near the y 0 end. Source: T-Display.step.
TD_UNDERSIDE_TALLEST = 3.4
# The two header rows' through-holes, (x0, x1, y0, y1), along the long edges.
# The model's pins and spacers stand out of the -z face to z -8.5; the mount
# keeps both faces clear along these strips. Source: T-Display.step.
TD_HEADER_ROWS = ((-25.17, -22.63, 14.83, 45.31), (-2.26, 0.28, 14.83, 45.31))
# How far the pins and spacers stand out on the -z face, from the PCB's +z
# face (the model's z 0). Source: T-Display.step, the pins' lowest z.
TD_HEADER_PIN_DEPTH = 8.5
# Bare spots on the +z face, (x0, x1, y0, y1): no part and no header row in
# them, on a 0.5 mm grid rounded outwards round every part, so a holder can
# bear on them. Source: T-Display.step (reference_check.py re-checks them).
TD_BARE_END = (-25.16, 0.0, 46.0, 51.48)  # the full width at the y 51.48 end
TD_BARE_CENTRE = (-17.0, -7.5, 22.0, 29.5)  # Source: as above.
TD_BARE_SIDE = (-22.5, -18.0, 31.5, 40.0)  # Source: as above.
TD_BARE = (TD_BARE_END, TD_BARE_CENTRE, TD_BARE_SIDE)  # Source: the three above.
