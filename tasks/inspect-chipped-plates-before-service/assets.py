"""Restaurant dish-inspection station with a low slotted drainer and serving hatch."""

from __future__ import annotations

import math

from partkit import part
from yam_arena.task_file import TABLE_TOP

TOP = TABLE_TOP["station"]
PLATE_X = 0.140
PLATE_Y = {"plate_a": 0.125, "plate_b": -0.125}
BIN_XY = (0.410, -0.320)
SERVICE_XY = (0.410, 0.0)
PLATE_RADIUS = 0.090
BIN_INNER_HALF = 0.130
BIN_HEIGHT = 0.135
SERVICE_INNER_HALF = 0.141
PLATE_TILT = math.radians(15)
PLATE_YAW = 0.0
# The dishes settle onto the shallow runners near TOP + .0415. Reset starts
# 13 mm above contact, as in the physically successful early contact runs.
PLATE_CENTER_Z = TOP + .055

GLAZE = (0.88, 0.87, 0.82)
UNDERSIDE = (0.76, 0.75, 0.70)
FRACTURE = (0.67, 0.57, 0.43)
CHARCOAL = (0.12, 0.14, 0.15)
STEEL = (0.67, 0.72, 0.73)
GREEN = (0.18, 0.42, 0.33)
RED = (0.61, 0.13, 0.11)


def box(x, y, z, sx, sy, sz, color, yaw=0.0):
    return ((x, y, z), (sx, sy, sz), color, yaw)


def plate(name: str, *, chipped: bool):
    # Ordinary round plates lean slightly within the low draining rack. The
    # chipped plate has missing outer-lip geometry, not a painted defect.
    cylinders = [((0, 0, 0), 0.075, 0.006, GLAZE, "Z", PLATE_TILT),
                 ((0, -0.0015, 0.0025), 0.059, 0.0015,
                   UNDERSIDE, "Z", PLATE_TILT)]
    # Standard round service plate: continuous raised rim, no handling ear.
    boxes = []
    # A broad, uninterrupted restaurant-plate flange gives the parallel jaws
    # a realistic radial pinch surface without a protruding grip tab.
    for i in range(48):
        angle = 2 * math.pi * (i + .5) / 48
        py = .069 * math.sin(angle)
        cylinders.append(((.069 * math.cos(angle),
                           py * math.cos(PLATE_TILT),
                           py * math.sin(PLATE_TILT)),
                          .009, .008, GLAZE, "Z", PLATE_TILT))
    for i in range(48):
        if chipped and i in {34, 35, 36}:
            continue
        angle = 2 * math.pi * (i + .5) / 48
        py = .080 * math.sin(angle)
        cylinders.append(((.080 * math.cos(angle),
                           py * math.cos(PLATE_TILT),
                           py * math.sin(PLATE_TILT)),
                          .0072, .006, GLAZE, "Z", PLATE_TILT))
    # A continuous rolled edge forms a shallow undercut above the pinch band.
    # The narrow outer neck connects the original lip to its overhanging top;
    # no local ear or hidden grasp tab is introduced. The chipped arc is
    # omitted from every layer of the roll.
    for i in range(96):
        if chipped and i in {68, 69, 70, 71, 72, 73}:
            continue
        angle = 2 * math.pi * (i + .5) / 96
        py = .084 * math.sin(angle)
        cylinders.append(((.084 * math.cos(angle),
                           py * math.cos(PLATE_TILT) - .002 * math.sin(PLATE_TILT),
                           py * math.sin(PLATE_TILT) + .002 * math.cos(PLATE_TILT)),
                          .003, .008, GLAZE, "Z", PLATE_TILT))
    for i in range(48):
        if chipped and i in {34, 35, 36}:
            continue
        angle = 2 * math.pi * (i + .5) / 48
        py = .080 * math.sin(angle)
        cylinders.append(((.080 * math.cos(angle),
                           py * math.cos(PLATE_TILT) - .006 * math.sin(PLATE_TILT),
                           py * math.sin(PLATE_TILT) + .006 * math.cos(PLATE_TILT)),
                          .010, .003, GLAZE, "Z", PLATE_TILT))
    if chipped:
        for i in (33, 37):
            angle = 2 * math.pi * (i + .5) / 48
            py = .080 * math.sin(angle)
            cylinders.append(((.080 * math.cos(angle),
                               py * math.cos(PLATE_TILT),
                               py * math.sin(PLATE_TILT)),
                              .0072, .0062, FRACTURE, "Z", PLATE_TILT))
    return part(name, (PLATE_X, PLATE_Y[name], PLATE_CENTER_Z), boxes,
                cylinders=cylinders, mass=0.285, friction=5.0,
                yaw=PLATE_YAW)


def dish_drainer():
    """Two separated wire-rack bays, with shallow supports under each plate."""
    color = (0.36, 0.42, 0.43)
    # A domestic rack has thin perimeter wires and upright separator tines.
    # The tines sit outside the swept disk; they keep the two bays visibly
    # distinct without trapping either rim or blocking a central pinch.
    shapes = []
    for y in (-.250, .250):
        shapes.append(box(0, y, .014, .226, .006, .006, color))
    for x in (-.109, .109):
        shapes.append(box(x, 0, .014, .006, .506, .006, color))
        for y in (-.250, .250):
            shapes.append(box(x, y, .008, .006, .006, .016, color))
    for y in (-.245, 0.0, .245):
        for x in (-.075, -.025, .025, .075):
            shapes.append(box(x, y, .057, .005, .005, .082, STEEL))
            shapes.append(box(x, y, .099, .009, .009, .004, STEEL))
    rail_z = TOP + .0395
    for slot_y in PLATE_Y.values():
        for dx in (-.035, 0.0, .035):
            shapes.append(((dx, slot_y, rail_z - TOP),
                           (.028, .100 / math.cos(PLATE_TILT), .006),
                           STEEL, 0.0, PLATE_TILT))
    return part("dish_drainer", (PLATE_X, 0, TOP), shapes, mass=.75,
                friction=1.5, fixed_to_world=True)


def waste_bin():
    """Open countertop reject bin with a hinged-up lid and front pedal."""
    half = BIN_INNER_HALF
    wall = 0.006
    outer = half + wall
    shapes = [box(0, 0, 0.004, 2 * outer, 2 * outer, 0.008, CHARCOAL)]
    for x in (-outer + wall / 2, outer - wall / 2):
        shapes.append(box(x, 0, BIN_HEIGHT / 2, wall, 2 * outer,
                          BIN_HEIGHT, CHARCOAL))
    for y in (-outer + wall / 2, outer - wall / 2):
        shapes.append(box(0, y, BIN_HEIGHT / 2, 2 * outer - 2 * wall,
                          wall, BIN_HEIGHT, CHARCOAL))
    # Red molded lip and open, hinged-up lid make this a waste bin rather than
    # another receiving tray. The lid is fixed in its open reset pose so the
    # plate enters by ordinary gravity/contact, without a scripted lid motion.
    for x in (-outer, outer):
        shapes.append(box(x, 0, BIN_HEIGHT - 0.005,
                          0.010, 2 * outer, 0.010, RED))
    for y in (-outer, outer):
        shapes.append(box(0, y, BIN_HEIGHT - 0.005,
                          2 * outer, 0.010, 0.010, RED))
    shapes.append(box(-outer - .016, 0, .013, .030, .186, .004, RED))
    shapes.append(box(-outer - .016, 0, .016, .014, .100, .001, (0.96, .94, .88)))
    # Small silver pedal and hinge barrels are familiar domestic-bin cues.
    shapes.append(box(-outer - .031, 0, .006, .063, .085, .008, STEEL))
    shapes.append(box(-outer - .031, 0, .011, .048, .067, .002, (0.83, .86, .83)))
    for y in (-.090, .090):
        shapes.append(box(outer + .003, y, BIN_HEIGHT, .010, .025, .012, CHARCOAL))
    # The pedal lid has flipped back behind the bin opening. This keeps its
    # broad face and waste pictogram visible without covering the aperture.
    lid_x = outer + .082
    lid_z = BIN_HEIGHT + .008
    shapes.append(box(lid_x, 0, lid_z, .160, .266, .009, RED))
    # Raised white wastebasket pictogram on the upward face of the open lid.
    def lid_mark(dx, dy, sx, sy):
        centre = (lid_x + dx, dy, lid_z + .006)
        return (centre, (sx, sy, .0015), (0.95, .94, .88), 0.0, 0.0)

    shapes.extend((lid_mark(0, .008, .067, .009),
                   lid_mark(0, -.013, .052, .041),
                   lid_mark(-.015, -.013, .004, .028),
                   lid_mark(.015, -.013, .004, .028)))
    # A legible label distinguishes the reject bin from the service tray.
    font = {"W": ("101", "101", "101", "111", "101"),
            "A": ("010", "101", "111", "101", "101"),
            "S": ("111", "100", "111", "001", "111"),
            "T": ("111", "010", "010", "010", "010"),
            "E": ("111", "100", "110", "100", "111")}
    pixel = .0075
    for i, char in enumerate("WASTE"):
        for row, bits in enumerate(font[char]):
            for col, bit in enumerate(bits):
                if bit == "1":
                    shapes.append(box(lid_x + .064 - row * pixel,
                                      (9.5 - i * 4 - col) * pixel,
                                      lid_z + .006, pixel * .82,
                                      pixel * .82, .0015,
                                      (0.98, .98, .94)))
    return part("waste_bin", (*BIN_XY, TOP), shapes, mass=0.65,
                friction=0.85, fixed_to_world=True)


def service_tray():
    # A low washable stainless tray directly in front of the pass opening.
    sx, sy = 0.300, 0.300
    shapes = [box(0, 0, 0.004, sx, sy, 0.008, STEEL)]
    for x in (-sx / 2, sx / 2):
        shapes.append(box(x, 0, 0.009, 0.006, sy, 0.010, STEEL))
    for y in (-sy / 2, sy / 2):
        shapes.append(box(0, y, 0.009, sx, 0.006, 0.010, STEEL))
    shapes.append(box(0, 0, .009, .280, .280, .002, (0.83, .87, .84)))
    return part("service_tray", (*SERVICE_XY, TOP), shapes, mass=0.50,
                friction=0.85, fixed_to_world=True)


def serving_window():
    """Open restaurant pass-through with sill, two jambs, header and sign."""
    shapes = [box(0, 0, .013, .027, .290, .026, STEEL)]
    for y in (-.145, .145):
        shapes.append(box(0, y, .109, .027, .018, .218, STEEL))
    shapes.append(box(0, 0, .211, .030, .300, .023, STEEL))
    shapes.append(box(0, 0, .224, .040, .250, .004, GREEN))
    # Six block letters on the horizontal header, legible to the top camera.
    font = {"P": ("111", "101", "111", "100", "100"),
            "I": ("111", "010", "010", "010", "111"),
            "C": ("111", "100", "100", "100", "111"),
            "K": ("101", "101", "110", "101", "101"),
            "U": ("101", "101", "101", "101", "111")}
    pixel = .0057
    for i, char in enumerate("PICKUP"):
        for row, bits in enumerate(font[char]):
            for col, bit in enumerate(bits):
                if bit == "1":
                    shapes.append(box((2 - row) * pixel,
                                      (11.5 - i * 4 - col) * pixel, .227,
                                      pixel * .82, pixel * .82, .0008,
                                      (0.98, .98, .94)))
    # Shallow rear shelf reinforces that this is a transfer window.
    shapes.append(box(.039, 0, .010, .050, .230, .020, STEEL))
    return part("serving_window", (.580, SERVICE_XY[1], TOP), shapes,
                mass=1.2, fixed_to_world=True)


def make_objects(chipped_slot: str = "b"):
    if chipped_slot not in ("a", "b"):
        raise ValueError("chipped_slot must be 'a' or 'b'")
    return [dish_drainer(), waste_bin(), service_tray(), serving_window(),
            plate("plate_a", chipped=chipped_slot == "a"),
            plate("plate_b", chipped=chipped_slot == "b")]
