"""YAM servo lost-motion fixture with measured unilateral elastic links."""
from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from partkit import part, on_reset, on_step
from readout_texture import spawn as spawn_readout
from runtime import RUNTIME
from yam_arena.task_file import TABLE_TOP

SCENE = "station"
TOP = TABLE_TOP[SCENE]
CX = .345
YAW = math.pi / 2  # pull lines run across the overhead image, world Y
INSTRUCTION = (
    "Characterize the mounted robot servo's lost motion using the two connected "
    "spring gauges. Allow both gauges to remain unloaded for two seconds. Grip a "
    "gauge body and pull along its loading line until its reading is 1.2–1.8 N, "
    "with the opposite gauge at or below 0.10 N. Hold the force and angle steady "
    "for two seconds, then unload both gauges for two seconds. Repeat for the "
    "opposite direction. Leave both gauges released and unloaded for two seconds. "
    "The recorder automatically saves the force and angle samples. Do not push "
    "the servo, lever or exposed gauge eyelets, change the servo setting, disturb the "
    "fixture, or reach 2.0 N for 0.2 seconds."
)

STEEL = (.48, .53, .58)
BLACK = (.065, .075, .09)
BLUE = (.30, .46, .58)
WHITE = (.87, .88, .82)
INK = (.09, .15, .20)
YELLOW = (.82, .64, .21)
ORANGE = (.84, .30, .09)


def b(x, y, z, sx, sy, sz, color):
    return ((x, y, z), (sx, sy, sz), color)


def c(x, y, z, radius, height, color, axis="Z"):
    return ((x, y, z), radius, height, color, axis)


_FONT = {
    ' ': '000 000 000 000 000',
    'A': '010 101 111 101 101', 'C': '111 100 100 100 111',
    'D': '110 101 101 101 110', 'E': '111 100 110 100 111',
    'F': '111 100 110 100 100', 'G': '111 100 101 101 111',
    'H': '101 101 111 101 101', 'I': '111 010 010 010 111',
    'J': '001 001 001 101 111', 'L': '100 100 100 100 111',
    'N': '101 111 111 111 101', 'O': '111 101 101 101 111',
    'R': '110 101 110 101 101', 'S': '111 100 111 001 111',
    'T': '111 010 010 010 010', 'U': '101 101 101 101 111',
    '0': '111 101 101 101 111', '2': '111 001 111 100 111',
}


def label(word, x, y, z, pitch, color):
    width = (4 * len(word) - 1) * pitch
    return [b(x - width / 2 + (4 * i + col + .5) * pitch,
              y + (2 - row) * pitch, z, pitch * .8, pitch * .8, .0007, color)
            for i, letter in enumerate(word)
            for row, bits in enumerate(_FONT[letter].split())
            for col, value in enumerate(bits) if value == '1']


def _source_mesh(stage, path, name, mapper):
    import trimesh
    from pxr import Gf, UsdGeom

    source = trimesh.load(Path(__file__).parent / 'source_assets' / name)
    verts = mapper(np.asarray(source.vertices, np.float64))
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in verts])
    mesh.CreateFaceVertexCountsAttr([3] * len(source.faces))
    mesh.CreateFaceVertexIndicesAttr(source.faces.reshape(-1).tolist())
    mesh.CreateSubdivisionSchemeAttr('none')
    mesh.CreateDisplayColorAttr([Gf.Vec3f(*BLUE)])


def _holder_cad(stage, prim_path, _spec):
    _source_mesh(stage, f"{prim_path}/body/source_holder",
                 'servo_holder_single.stl',
                 lambda v: v * .001 + np.array([.022, -.0175, .040]))


def _lever_cad(stage, prim_path, _spec):
    _source_mesh(stage, f"{prim_path}/body/source_lever",
                 'test_lever_100mm_single.stl',
                 lambda v: np.column_stack(((v[:, 0] - 4.) * .001,
                                            -(v[:, 2] - 12.) * .001,
                                            .110 + (v[:, 1] + 30.) * .001)))


def _band_visuals(stage, prim_path, spec):
    """Visual-only links; runtime moves endpoints from measured body states."""
    from pxr import Gf, UsdGeom
    spec.meta['prim_path'] = prim_path
    for side in ('left', 'right'):
        line = UsdGeom.Cylinder.Define(stage, f'{prim_path}/body/band_{side}')
        line.CreateAxisAttr('X')
        line.CreateRadiusAttr(.00125)
        line.CreateHeightAttr(.180)
        line.AddTranslateOp().Set(Gf.Vec3d(0, 0, .210))
        line.AddOrientOp().Set(Gf.Quatf(1, 0, 0, 0))
        line.CreateDisplayColorAttr([Gf.Vec3f(*ORANGE)])


def _gauge_pointer(stage, prim_path, spec):
    from pxr import Gf, UsdGeom
    spec.meta['prim_path'] = prim_path
    pointer = UsdGeom.Cube.Define(stage, f'{prim_path}/body/force_pointer')
    pointer.CreateSizeAttr(1.)
    pointer.AddTranslateOp().Set(Gf.Vec3d(-.052, 0, .0104))
    pointer.AddScaleOp().Set(Gf.Vec3f(.003, .014, .001))
    pointer.CreateDisplayColorAttr([Gf.Vec3f(*ORANGE)])


_SEGMENT_GEOMETRY = (
    (0, .014, .018, .0022), ( .009, .007, .0022, .014),
    ( .009,-.007, .0022, .014), (0,-.014, .018, .0022),
    (-.009,-.007, .0022, .014), (-.009, .007, .0022, .014),
    (0, 0, .018, .0022),
)


def _recorder_digits(stage, prim_path, spec):
    from pxr import Gf, UsdGeom
    spec.meta['prim_path'] = prim_path
    UsdGeom.Xform.Define(stage, f'{prim_path}/readout_visual')
    for row, y in enumerate((.044, 0., -.044)):
        for digit, x in enumerate((.055, .085, .115)):
            for segment, (dx, dy, sx, sy) in enumerate(_SEGMENT_GEOMETRY):
                cube = UsdGeom.Cube.Define(stage, f'{prim_path}/readout_visual/digit_r{row}d{digit}s{segment}')
                cube.CreateSizeAttr(1.)
                # Start dark, inside the opaque screen body. Runtime reveals
                # only segments selected by the measured values.
                cube.AddTranslateOp().Set(Gf.Vec3d(x+dx, y+dy, 0.))
                cube.AddScaleOp().Set(Gf.Vec3f(sx,sy,.001))
                cube.CreateDisplayColorAttr([Gf.Vec3f(.70,.95,.70)])
        dot = UsdGeom.Cube.Define(stage, f'{prim_path}/readout_visual/decimal_r{row}')
        dot.CreateSizeAttr(1.)
        dot.AddTranslateOp().Set(Gf.Vec3d(.071 if row < 2 else .101, y-.015, .0514))
        dot.AddScaleOp().Set(Gf.Vec3f(.0025,.0025,.001))
        dot.CreateDisplayColorAttr([Gf.Vec3f(.70,.95,.70)])


base = part('base_plate', (CX, 0, TOP), (
    b(0, 0, .008, .450, .300, .016, STEEL),
    b(0, 0, .028, .050, .064, .024, BLACK),
), 2.0, yaw=YAW, fixed_to_world=True, cylinders=tuple(
    c(x, y, .0175, .0045, .003, BLACK) for x in (-.205, .205) for y in (-.125, .125)))
base.meta['visual_boxes'] = [
    # Printed alignment witnesses are externally inspectable mount markers.
    b(x, y, .0166, .014, .003, .001, WHITE)
    for x in (-.205, .205) for y in (-.125, .125)
] + [
    # Steel shaft reference markers.
    b(x, 0, .018, .006, .0022, .001, ORANGE) for x in (-.090, .090)
]
base.meta['spawn_hook'] = _band_visuals

holder = part('printed_servo_holder', (CX, 0, TOP), (
    b(0, 0, .043, .044, .070, .006, BLUE),
    b(-.023, 0, .054, .003, .065, .022, BLUE),
    b(.023, 0, .054, .003, .065, .022, BLUE),
), .080, yaw=YAW, fixed_to_world=True)
holder.meta['spawn_hook'] = _holder_cad

servo = part('held_uart_servo', (CX, 0, TOP), (
    b(0, 0, .075, .042, .040, .050, BLACK),
    b(0, -.0205, .076, .034, .001, .026, WHITE),
), .070, yaw=YAW, fixed_to_world=True)

# The motor-side member is driven toward a fixed zero setpoint. The separate
# output member has finite angular travel against opposite PhysX joint limits.
motor = part('motor_rotor', (CX, 0, TOP), (
    b(0, 0, .110, .012, .016, .012, STEEL),
), .030, yaw=YAW, hinge_attachment=(
    'printed_servo_holder', (0, 0, .110), (0, 0, .110),
    12.0, -12.0, 0.0, 'Y', 25.0, .15, .35,
))

lever = part('output_lever', (CX, 0, TOP), (
    b(0, 0, .165, .009, .014, .090, BLUE),
    b(0, 0, .210, .025, .014, .012, ORANGE),
), .030, yaw=YAW, hinge_attachment=(
    'motor_rotor', (0, 0, .110), (0, 0, .110),
    .7, -.7, 0.0, 'Y', 0.0, .01, .35,
))
lever.meta['spawn_hook'] = _lever_cad

supports = []
gauges = []
for sign, side in ((-1, 'left'), (1, 'right')):
    x = sign * .250
    support = part(f'{side}_gauge_cradle', (CX, x, TOP), (
        b(0, 0, .100, .018, .025, .200, BLACK),
        b(0, 0, .200, .160, .026, .008, STEEL),
        # Low lips at the two ends keep the gauge supported yet leave its
        # centre unobstructed for an ordinary parallel-jaw body grasp.
        *(b(ex, ey, .2055, .022, .003, .003, STEEL)
          for ex in (-.065, .065) for ey in (-.014, .014)),
    ), .35, yaw=YAW, fixed_to_world=True)
    # Provisional 140 × 20 × 18 mm envelope: the PASCO page specifies a
    # 100 mm graduated scale, not an overall body dimension.
    gauge = part(f'{side}_spring_gauge', (CX, x, TOP + .213), (
        b(0, 0, 0, .140, .020, .018, WHITE),
        b(sign * -.073, 0, 0, .006, .010, .008, STEEL),
    ), .080, yaw=YAW, friction=.7)
    gauge.meta['visual_boxes'] = [
        b(-.052 + i * .0104, 0, .0095, .0013, .010 if i % 2 == 0 else .006,
          .0007, INK) for i in range(11)
    ] + [
        b(0, 0, .0097, .105, .0012, .0007, INK),
    ] + label('0', -.051, -.005, .0098, .00125, INK) + label('2', .050, -.005, .0098, .00125, INK)
    gauge.meta['spawn_hook'] = _gauge_pointer
    supports.append(support)
    gauges.append(gauge)

recorder = part('measurement_recorder', (CX - .245, 0, TOP), (
    b(0, 0, .024, .320, .170, .048, BLACK),
    b(0, 0, .049, .300, .146, .002, INK),
), .40, yaw=YAW + math.pi, fixed_to_world=True)
recorder.meta['spawn_hook'] = spawn_readout

OBJECTS = [base, holder, servo, motor, lever, *supports, *gauges, recorder]


@on_reset
def _reset(_env):
    RUNTIME.reset()


@on_step
def _step(environment):
    RUNTIME.step(environment)


def score(_objects):
    return RUNTIME.acquisition.score


def success(_objects):
    return RUNTIME.acquisition.score == 1.0
