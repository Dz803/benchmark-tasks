"""Procedural, dimensioned assets for the four-D-cell emergency lantern scene.

All lengths are metres. Parts are compound USD rigid bodies created by partkit
when yam_arena loads task.py. This module defines geometry and a pressable
spring-return switch.
"""

from __future__ import annotations

import math

from partkit import Slider, part

LANTERN_XY = (0.35, 0.0)
LOOSE_X = 0.17
LOOSE_Y = (-0.067, 0.067)
COVER_XY = (0.15, 0.265)
COVER_SEATED_Z = 0.057

# Energizer E95 / IEC LR20 D-cell envelope: 34.2 mm diameter, 61.5 mm length.
D_RADIUS = 0.0171
D_LENGTH = 0.0615
D_MASS = 0.139
CELL_CENTRE_Z = 0.023 + D_RADIUS
SLOT_X = -0.025
SLOT_Y = (-0.063, -0.021, 0.021, 0.063)
SLOT_PLUS_X = (1, -1, 1, -1)
PREINSTALLED_SLOTS = (0, 3)
EMPTY_SLOTS = (1, 2)

BODY = (0.17, 0.20, 0.23)
TRIM = (0.07, 0.085, 0.10)
WELL = (0.035, 0.045, 0.055)
RAIL = (0.28, 0.31, 0.33)
LENS_RING = (0.18, 0.23, 0.25)
LENS = (0.43, 0.48, 0.49)  # frosted but visibly unlit
LENS_ON = (1.0, 0.94, 0.60)
CELL_BLUE = (0.12, 0.39, 0.77)
SILVER = (0.72, 0.76, 0.79)
PLUS = (0.90, 0.18, 0.11)
MINUS = (0.87, 0.87, 0.83)
SWITCH = (0.96, 0.58, 0.13)


def box(x, y, z, sx, sy, sz, color):
    return ((x, y, z), (sx, sy, sz), color)


def glyph(x: float, y: float, z: float, positive: bool, color,
          span: float = 0.012, stroke: float = 0.0018):
    """Two shallow strokes make a visible + or - without a texture dependency."""
    bars = [box(x, y, z, span, stroke, 0.0008, color)]
    if positive:
        bars.append(box(x, y, z, stroke, span, 0.0008, color))
    return bars


def make_lantern(table_top: float):
    """Open 4-slot holder, broad diffuser, carry handle and pressable switch."""
    b = [
        box(0, 0, 0.011, 0.270, 0.190, 0.022, BODY),
        box(SLOT_X, 0, 0.021, 0.086, 0.176, 0.004, WELL),
        # The battery carrier has one physical floor and five longitudinal
        # rails. A D cell rests on the floor rather than a score-only target.
    ]
    for y in (-0.084, -0.042, 0.0, 0.042, 0.084):
        # Low ribs guide the cell without providing a high edge on which a
        # slightly tilted rigid cylinder can perch during insertion.
        b.append(box(SLOT_X, y, 0.0305, 0.077, 0.0015, 0.016, RAIL))
    for x in (-0.061, 0.011):
        b.append(box(x, 0, 0.032, 0.003, 0.168, 0.020, RAIL))
    # Small metal contact pads face the ends of each slot. The negative-side
    # spring is reduced to its contact pad in this scene; no electrical force
    # or current is claimed.
    for y in SLOT_Y:
        b.append(box(-0.0585, y, 0.040, 0.002, 0.013, 0.017, SILVER))
        b.append(box(0.0085, y, 0.040, 0.002, 0.013, 0.017, SILVER))
    # Long lips outside the carrier accept the removable cover's underside
    # skirt. The cover is a separate rigid body, not painted onto the bay.
    for y in (-0.092, 0.092):
        b.append(box(SLOT_X, y, 0.045, 0.101, 0.004, 0.040, TRIM))
    # The close-fitting cover rim seats vertically inside the two side lips.
    for slot, y in enumerate(SLOT_Y):
        plus_x = SLOT_X + (0.052 if SLOT_PLUS_X[slot] > 0 else -0.052)
        minus_x = SLOT_X - (0.052 if SLOT_PLUS_X[slot] > 0 else -0.052)
        b.extend(glyph(plus_x, y, 0.023, True, PLUS, span=0.017, stroke=0.0022))
        b.extend(glyph(minus_x, y, 0.023, False, MINUS, span=0.017, stroke=0.0022))
    # Broad work-light diffuser and a guarded push-on switch sit beyond the
    # battery cover. The switch is a spring-return joint, not a painted button.
    b.extend([
        box(0.092, 0, 0.043, 0.088, 0.145, 0.043, TRIM),
        box(0.092, 0, 0.067, 0.087, 0.138, 0.006, LENS_RING),
        box(0.094, 0.004, 0.073, 0.074, 0.112, 0.006, LENS),
        box(0.092, -0.068, 0.073, 0.033, 0.023, 0.005, RAIL),
        box(0.131, 0, 0.018, 0.008, 0.157, 0.027, BODY),
        # Low carry handle: two side bars and a cross-piece, all attached to
        # the housing. It stays outside the battery and cover workspace.
        box(0.135, -0.103, 0.018, 0.088, 0.009, 0.019, TRIM),
        box(0.135, 0.103, 0.018, 0.088, 0.009, 0.019, TRIM),
        box(0.177, 0, 0.018, 0.009, 0.215, 0.019, TRIM),
    ])
    return part(
        "lantern", pos=(*LANTERN_XY, table_top), mass=0.427, boxes=b,
        sliders=(Slider("power_button", rest=(0.092, -0.068, 0.080),
                        boxes=(box(0.092, -0.068, 0.080, 0.020, 0.015, 0.008, SWITCH),),
                        travel=0.004, rate=180.0, damping=2.0, mass=0.008),),
        friction=0.85,
    )


def make_cell(name: str, x: float, y: float, z: float, yaw: float = 0.0):
    """Full-size D cell with a raised positive nub and printed top + mark."""
    boxes = [box(-0.016, 0, 0, 0.016, 0.028, 0.027, CELL_BLUE)]
    boxes.extend(glyph(0.019, 0, D_RADIUS + 0.0005, True, PLUS))
    return part(
        name, pos=(x, y, z), yaw=yaw, mass=D_MASS, boxes=boxes,
        cylinders=[((0, 0, 0), D_RADIUS, 0.0585, CELL_BLUE, "X"),
                   ((0.0298, 0, 0), 0.0065, 0.0034, SILVER, "X"),
                   ((-0.0295, 0, 0), 0.013, 0.0007, SILVER, "X")],
        friction=1.10,  # vinyl jacket against compliant gripper pads
    )


def make_cover(table_top: float):
    """Detached panel with a close-fitting inner rim and outer support runners."""
    # The panel sits on the side lips while its inner rim enters the narrow
    # channel vertically, like a removable friction-fit battery door.
    b = [box(0, 0, 0.019, 0.105, 0.187, 0.006, BODY),
         box(0, 0, 0.024, 0.024, 0.023, 0.008, TRIM),
         box(-0.0525, 0, 0.013, 0.008, 0.026, 0.024, TRIM)]
    for y in (-0.083, 0.083):
        b.append(box(0, y, 0.009, 0.097, 0.004, 0.018, TRIM))
    for y in (-0.092, 0.092):
        b.append(box(0, y, 0.0125, 0.090, 0.002, 0.009, TRIM))
    # A continuous inner rim descends into the narrow channel between the
    # carrier's outer ribs and its side lips. At the seated pose it has about
    # 1 mm clearance inside and 2.75 mm outside, so it is registered rather
    # than simply resting as a flat panel on top.
    for y in (-0.0865, 0.0865):
        b.append(box(0, y, -0.004, 0.090, 0.0015, 0.035, TRIM))
    for x in (-0.044, 0.044):
        b.append(box(x, 0, -0.004, 0.003, 0.173, 0.035, TRIM))
    # Shallow molded lines on the cover face keep it legible from the top view.
    for y in (-0.052, 0.052):
        b.append(box(0, y, 0.0224, 0.072, 0.004, 0.001, RAIL))
    return part("cover", pos=(*COVER_XY, table_top + 0.0215), mass=0.09, boxes=b,
                friction=0.75)


def make_scene(table_top: float):
    lantern = make_lantern(table_top)
    installed = []
    for slot in PREINSTALLED_SLOTS:
        yaw = 0.0 if SLOT_PLUS_X[slot] > 0 else math.pi
        installed.append(make_cell(f"installed_{slot}", LANTERN_XY[0] + SLOT_X,
                                   LANTERN_XY[1] + SLOT_Y[slot],
                                   table_top + CELL_CENTRE_Z, yaw=yaw))
    # Loose cells can naturally lie with opposite terminal directions; the
    # agent must match each visible nub to the marked well before insertion.
    loose = [make_cell(f"loose_{i}", LOOSE_X, y, table_top + D_RADIUS + 0.0005,
                       yaw=math.pi if i == 0 else 0.0)
             for i, y in enumerate(LOOSE_Y)]
    cover = make_cover(table_top)
    return [lantern, *installed, *loose, cover]
