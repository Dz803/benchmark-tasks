"""Two hinged household bins and three everyday waste items.

Geometry is authored from rigid primitives. A local VOMP bin mesh was not
copied because its underlying redistribution terms remain unverified.
"""

from __future__ import annotations

import math

from partkit import part
from yam_arena.task_file import Asset, TABLE_TOP

TOP = TABLE_TOP["station"]
BIN_X = .200
BIN_Y = {"green": -.13, "black": .13}
BIN_RADIUS = .086
BIN_INNER_RADIUS = .082
BIN_HEIGHT = .190
LID_RADIUS = .090
LID_Z = BIN_HEIGHT + .006
HINGE_X = .081
BOTTLE_XY = (.055, -.015)
CAN_XY = (0.000, .135)
NAPKIN_XY = (-.005, .055)
BOTTLE_RADIUS = .025
BOTTLE_HEIGHT = .140
GREEN = (.05, .42, .16)
BLACK = (.055, .06, .07)
LIGHT_PET = (.58, .80, .81)
WHITE = (.97, .98, .96)
DARK = (.08, .19, .25)


def box(x, y, z, sx, sy, sz, color, yaw=0.0):
    return ((x, y, z), (sx, sy, sz), color, yaw)


FONT = {
    "A": ("010", "101", "111", "101", "101"),
    "C": ("111", "100", "100", "100", "111"),
    "E": ("111", "100", "110", "100", "111"),
    "G": ("111", "100", "101", "101", "111"),
    "I": ("111", "010", "010", "010", "111"),
    "L": ("100", "100", "100", "100", "111"),
    "M": ("101", "111", "111", "101", "101"),
    "N": ("101", "111", "111", "111", "101"),
    "O": ("111", "101", "101", "101", "111"),
    "P": ("111", "101", "111", "100", "100"),
    "R": ("111", "101", "111", "110", "101"),
    "S": ("111", "100", "111", "001", "111"),
    "T": ("111", "010", "010", "010", "010"),
    "W": ("101", "101", "111", "111", "101"),
    "X": ("101", "101", "010", "101", "101"),
    "Y": ("101", "101", "010", "010", "010"),
    "D": ("110", "101", "101", "101", "110"),
}


def horizontal_text(word, pixel, y, z):
    width = (len(word) * 4 - 1) * pixel
    return [box(-width / 2 + (i * 4 + j) * pixel, y + (2 - row) * pixel, z,
                pixel * .88, pixel * .88, .0008, WHITE)
            for i, char in enumerate(word)
            for row, bits in enumerate(FONT[char])
            for j, bit in enumerate(bits) if bit == "1"]


def front_text(word, pixel, z, x=-.090):
    width = (len(word) * 4 - 1) * pixel
    return [box(x, -width / 2 + (i * 4 + j) * pixel, z + (2 - row) * pixel,
                .0008, pixel * .88, pixel * .88, WHITE)
            for i, char in enumerate(word)
            for row, bits in enumerate(FONT[char])
            for j, bit in enumerate(bits) if bit == "1"]


def bin_wall(kind):
    color = GREEN if kind == "green" else BLACK
    shapes = []
    for i in range(24):
        angle = (i + .5) * 2 * math.pi / 24
        r = (BIN_RADIUS + BIN_INNER_RADIUS) / 2
        shapes.append(box(r * math.cos(angle), r * math.sin(angle), BIN_HEIGHT / 2,
                          BIN_RADIUS - BIN_INNER_RADIUS,
                          r * (2 * math.pi / 24) * 1.02,
                          BIN_HEIGHT, color, angle))
        shapes.append(box(BIN_RADIUS * math.cos(angle), BIN_RADIUS * math.sin(angle),
                          BIN_HEIGHT - .005, .007,
                          BIN_RADIUS * (2 * math.pi / 24) * 1.02,
                          .010, color, angle))
    shapes.append(box(-.088, 0, BIN_HEIGHT * .59, .004, .155, .060, color))
    if kind == "green":
        shapes += front_text("MIXED", .0045, BIN_HEIGHT * .64)
        shapes += front_text("RECYCLING", .0026, BIN_HEIGHT * .49)
    else:
        shapes += front_text("GENERAL", .0036, BIN_HEIGHT * .64)
        shapes += front_text("WASTE", .0045, BIN_HEIGHT * .49)
    floor = (((0, 0, .004), BIN_INNER_RADIUS, .008, color, "Z"),)
    return part(f"{kind}_bin", (BIN_X, BIN_Y[kind], TOP),
                shapes, mass=.32, cylinders=floor, friction=1.0,
                fixed_to_world=True)


def lid(kind):
    color = GREEN if kind == "green" else BLACK
    shapes = [box(-.039, 0, .011, .030, .055, .018, color),
              # Raised molded pinch post keeps the gripper clear of the lid
              # surface while it follows the rear hinge.
              box(-.050, -.055 if kind == "green" else 0,
                  .026, .025, .030, .052, color)]
    shapes += horizontal_text("MIXED" if kind == "green" else "WASTE",
                              .008 if kind == "green" else .0065,
                              .040 if kind == "green" else -.040, .0062)
    cap = (((0, 0, 0), LID_RADIUS, .011, color, "Z"),)
    return part(f"{kind}_lid", (BIN_X, BIN_Y[kind], TOP + LID_Z),
                shapes, mass=.046, cylinders=cap, friction=.9,
                hinge_attachment=(f"{kind}_bin",
                                  (HINGE_X, 0, LID_Z),
                                  (HINGE_X, 0, 0), 110.0))


def bottle():
    # A molded grip waist lets the 47 mm YAM gripper grasp a real 50 mm
    # outer-diameter bottle without pretending its fingers pass through PET.
    shell = (((0, 0, .023), BOTTLE_RADIUS, .046, LIGHT_PET, "Z"),
             ((0, 0, .064), .018, .036, LIGHT_PET, "Z"),
             ((0, 0, .098), BOTTLE_RADIUS, .032, LIGHT_PET, "Z"),
             ((0, 0, .116), .021, .010, LIGHT_PET, "Z"),
             ((0, 0, .126), .014, .012, LIGHT_PET, "Z"),
             ((0, 0, .136), .015, .008, DARK, "Z"))
    trim = tuple(((0, 0, z), .0258, .002, LIGHT_PET, "Z")
                 for z in (.014, .025, .095, .105))
    trim += (((0, 0, .096), .0259, .020, WHITE, "Z"),)
    return part("pet_bottle", (*BOTTLE_XY, TOP + .002),
                boxes=(), cylinders=shell + trim, mass=.034, friction=.9)


def soup_can():
    # Reuse Isaac Lab Arena's registered YCB model, including its printed
    # label and metal-can silhouette. Scale to a grippable small household can.
    return Asset(name="soup_can", asset="tomato_soup_can",
                 xy=CAN_XY, extent=.060, mass=.040, convex=True)


def used_napkin():
    # A folded, food-stained paper napkin. This is one lightweight rigid body;
    # deformable paper is outside the current YAM/PhysX task contract.
    paper = (.88, .85, .77)
    grease = (.43, .25, .12)
    folds = [box(0, 0, .004, .060, .046, .006, paper),
             box(-.009, .006, .008, .038, .030, .003, (.94, .91, .84), .12),
             box(.012, -.009, .008, .029, .025, .003, paper, -.18),
             box(-.010, -.006, .010, .017, .012, .001, grease)]
    return part("used_napkin", (*NAPKIN_XY, TOP + .003),
                folds, mass=.012, friction=.95)


def make_objects():
    return [bin_wall("green"), lid("green"), bin_wall("black"),
            lid("black"), bottle(), soup_can(), used_napkin()]
