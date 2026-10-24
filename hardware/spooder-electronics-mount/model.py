"""The electronics mount's data model and geometry helpers.

Everything is in the frame top's coordinates (mm, +y forward, the frame top's
ring at z 25.75), so parts are checked and exported in their assembled
position. An `Assembly` is what `build.py` generates and what the checks in
`checks.py` and the export in `export.py` read:

- `Part`: a printed part, with its print orientation (`print_up`), its colour
  and any further copies (the bumpers) and second-colour inlays (the cover);
- `Held`: a bought part's outline (boards, converters, fans, the diode),
  reduced to simple solids for the checks;
- `Route`: a reserved cable route, which no part may enter, and the
  `Terminal`s it ends at, each reached from the side it takes its wires from;
- `Opening`: a tie hole, slot, wire entry or pass-through in the deck, which
  must open onto free space on the frame top below it;
- `Screw`: a screw's head and the outward direction a screwdriver comes from;
- `AirChannel`: the passive air path from the inlet past the SZBK07 to the
  outlet.

Boxes are given as (x0, x1, y0, y1, z0, z1).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from build123d import Circle, Location, Plane, Polygon, Shape, Solid, Vector, extrude

Vec3 = tuple[float, float, float]

# The one held part allowed inside a reserved route, and that route: the
# Schottky diode, carried inline on the LM2596's output lead to J6. Source:
# the spec's "Cable routes"; design.md "Everything electrical on the deck".
SCHOTTKY = "schottky diode"
LM2596_OUTPUT_ROUTE = "lm2596 output"
# The fans, held parts that the passive air channel is checked without.
# Source: the spec's "Cooling" (scenario "No fans fitted").
FANS = ("exhaust fan", "intake fan")
# Held parts the sensor provisions are checked against: the IMU breakout must
# lie near the centre and clear of the SZBK07. Source: the spec's "Sensor
# provisions".
SZBK07 = "szbk07"
IMU = "lsm6dso"
SERVO_2040 = "servo 2040"
# The Servo 2040's terminals that the sensor provisions' routes must reach.
# Source: the spec's "Sensor provisions".
J6 = "j6"
SENSOR_HEADER = "sensor header"
# Wire sides as measurements.py writes them, as unit axis directions.
SIDES = {"+x": (1.0, 0.0, 0.0), "-x": (-1.0, 0.0, 0.0), "+y": (0.0, 1.0, 0.0), "-y": (0.0, -1.0, 0.0),
         "+z": (0.0, 0.0, 1.0), "-z": (0.0, 0.0, -1.0)}
Box6 = tuple[float, float, float, float, float, float]


def box(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> Solid:
    """A box from its two corners (build123d's `Box` is centred)."""
    return Solid.make_box(x1 - x0, y1 - y0, z1 - z0).moved(Location((x0, y0, z0)))


def box6(b: Box6) -> Solid:
    return box(*b)


AXES = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0), "z": (0.0, 0.0, 1.0)}


def teardrop(r: float, length: float, x: float, y: float, z: float, axis: str, up: Vec3) -> Solid:
    """A hole cutter of radius r and the given length, centred on (x, y, z)
    along `axis` ("x", "y" or "z"), with a 45-degree point towards `up` (the
    direction that is up as printed, perpendicular to the axis). Subtract it
    from a part for a horizontal hole that prints without support."""
    a = Vector(AXES[axis])
    u = Vector(up).normalized()
    if abs(a.dot(u)) > 1e-9:
        raise ValueError(f"teardrop: up {up} is not perpendicular to axis {axis}")
    c = r * math.cos(math.radians(45))
    profile = Circle(r) + Polygon((0, 0), (c, c), (r * math.sqrt(2), 0), (c, -c), align=None)
    plane = Plane(origin=(x, y, z), x_dir=u, z_dir=a)
    return extrude(plane * profile, amount=length / 2, both=True).solid()


@dataclass
class Part:
    """A printed part. `shape` is its first copy in its assembled position;
    `copies` move it to the others (each distinct part is exported once).
    `print_up` is the robot-frame direction that is up as printed."""

    name: str
    shape: Shape
    role: str  # "deck", "cover", "bumper" or "holder"
    print_up: Vec3
    colour: str
    copies: tuple[Location, ...] = ()
    inlays: Shape | None = None  # the cover's second-colour inlays, exported in its 3MF

    def instances(self) -> list[Shape]:
        return [self.shape] + [self.shape.moved(loc) for loc in self.copies]

    def solids(self) -> list[Shape]:
        """Every copy, with the inlays as part of the first."""
        found = self.instances()
        if self.inlays is not None:
            found.append(self.inlays)
        return found


@dataclass
class Exposure:
    """A region of a held part that the cover leaves open in one axis
    direction: the T-Display's screen and buttons upwards, the XT60's mating
    face outwards. With `must_show`, the check also requires it to be open;
    without (a fan behind its grille), it is only exempt from enclosure."""

    direction: Vec3
    region: Box6
    must_show: bool = True


@dataclass
class Held:
    """A bought part's outline. `route` names the one reserved route it may
    lie in, wholly: only the Schottky diode (`SCHOTTKY`) may have one, and
    only the LM2596's output route (`LM2596_OUTPUT_ROUTE`)."""

    name: str
    shape: Shape
    exposed: tuple[Exposure, ...] = ()
    route: str | None = None


@dataclass
class Terminal:
    """Where a route ends: a region (a box) of the held part `held`, and the
    unit axis direction `side` its wires leave it along. With `held` None the
    region is free space the route must contain itself (the space just above
    a wire entry in the deck)."""

    name: str
    held: str | None
    region: Box6
    side: Vec3


@dataclass
class Route:
    """A reserved cable route: one or more solids no part may enter. Routes
    may overlap each other. `boxes` are the boxes the shape was made from,
    when it was made from boxes (`Route.of`); `ends` are the terminals it
    must reach; with `cover_off` it is kept clear of every part but the
    cover."""

    name: str
    shape: Shape
    ends: tuple[Terminal, ...] = ()
    boxes: tuple[Box6, ...] = ()
    cover_off: bool = False  # reserved only with the cover off (a USB cable): the cover is not checked against it

    @classmethod
    def of(cls, name: str, boxes, ends=(), cover_off: bool = False) -> "Route":
        boxes = tuple(boxes)
        shape = box6(boxes[0])
        for b in boxes[1:]:
            shape = shape + box6(b)
        return cls(name, shape, tuple(ends), boxes, cover_off)


@dataclass
class Opening:
    """A tie hole, slot, wire entry or pass-through through a part (the
    deck), with its footprint (x0, x1, y0, y1) and the part's underside z0
    and top z1 there. `kind` is "tie", "slot", "entry" (wires up from below)
    or "pass-through" (wires down to below the frame top, which needs the
    frame open under it); `wires` is how many wires an entry or pass-through
    must take."""

    name: str
    part: str
    footprint: tuple[float, float, float, float]
    z0: float
    z1: float
    kind: str = "tie"
    wires: int = 0


@dataclass
class Screw:
    """A screw: the centre of its head's top, the unit direction, out of
    the mount, along which a screwdriver reaches it, and the parts fitted
    after it is driven (not in its path then: the cover over the deck's
    screws)."""

    name: str
    head: Vec3
    axis: Vec3
    after: tuple[str, ...] = ()


@dataclass
class AirChannel:
    """The passive air path: boxes in order from the inlet to the outlet,
    each with the axis air flows along in it; the free space just inside the
    inlet's and the outlet's grilles, as boxes; the inlet's net open area
    (its grille's openings, not its outline); the held part the channel must
    pass (the SZBK07). The fans (`FANS`) are left out of the check, which is
    of the passive case."""

    segments: tuple[tuple[Box6, str], ...]
    inlet: Box6
    outlet: Box6
    inlet_open_area: float
    past: str


@dataclass
class ImuPad:
    """The IMU's pad on the deck: the part it is on and its screw bosses'
    centres (x, y), their top z and radius, and the pilot hole's radius."""

    part: str
    bosses: tuple[tuple[float, float], ...]
    top: float
    radius: float
    pilot_r: float


@dataclass
class Assembly:
    parts: list[Part] = field(default_factory=list)
    held: list[Held] = field(default_factory=list)
    routes: list[Route] = field(default_factory=list)
    openings: list[Opening] = field(default_factory=list)
    screws: list[Screw] = field(default_factory=list)
    channel: AirChannel | None = None
    imu_pad: ImuPad | None = None
