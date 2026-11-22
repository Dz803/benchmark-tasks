"""Self-service bag-drop scene with a printer-held tag and a side handle."""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from partkit import Slider, part
from lettering import label
from yam_arena.task_file import TABLE_TOP

SCENE = "station"
TOP = TABLE_TOP[SCENE]
# The 450 mm case sits centred between the arms, clear of both READY grippers
# (fingertips at |y| >= .234). The printer lane at y = -.265 keeps the drawn
# strip outside the case footprint.
BAG_XY = (.330, .000)
BAG_YAW = math.pi / 2
KIOSK_XY = (.675, -.265)
INSTRUCTION = (
    "Attach the printed baggage tag securely to the suitcase's long-side "
    "handle, with the barcode facing out. Its back is self-adhesive: loop it "
    "through the handle, press one end's back onto the other end, "
    "release the tag and withdraw both arms."
)

NAVY = (.17, .26, .40)
SEAM = (.035, .055, .090)
RIB = (.13, .22, .35)
HANDLE = (.14, .16, .18)
HANDLE_X = 0.0
# The source suitcase's long-side mounting region is centred at y≈-.02;
# the hard shell's bounding-box centre is not its visible mounting centre.
HANDLE_Y = -.020
SILVER = (.58, .64, .67)
SCALE = (.20, .27, .30)
SCALE_DARK = (.08, .12, .15)
WHITE = (.94, .95, .90)
BLACK = (.035, .045, .055)
YELLOW = (.90, .67, .12)
BLUE = (.13, .39, .65)
ADHESIVE_BACK = (.86, .84, .74)   # faint beige: the self-adhesive back


def box(x, y, z, sx, sy, sz, color, yaw=None, tilt=None):
    result = ((x, y, z), (sx, sy, sz), color)
    return result if yaw is None else result + (yaw, tilt or 0.0)


# At rest the fabric side grip lies close to the case: its centre rises only
# 30 mm above the shell, leaving a fingertip gap. The passive slider lets a
# grip move if contacted; routing uses the existing gap without a required lift.
HANDLE_ARCH = (
    (-.115, .319, .011), (-.103, .322, .011), (-.087, .331, .012), (-.065, .342, .013),
    (-.045, .347, .015), (-.020, .349, .015), (.005, .347, .015), (.025, .342, .013),
    (.047, .331, .012), (.063, .322, .011), (.075, .319, .011),
)
HANDLE_WORLD_X = BAG_XY[0] + math.cos(BAG_YAW) * HANDLE_X - math.sin(BAG_YAW) * HANDLE_Y
HANDLE_WORLD_Y = BAG_XY[1] + math.sin(BAG_YAW) * HANDLE_X + math.cos(BAG_YAW) * HANDLE_Y


def handle_arch_boxes(thickness=.006):
    """Tilted box colliders along the arch, so legs and grip both block the strip."""
    out = []
    for (y0, z0, w0), (y1, z1, w1) in zip(HANDLE_ARCH, HANDLE_ARCH[1:]):
        length = math.hypot(y1 - y0, z1 - z0) + .002
        out.append(box(HANDLE_X, (y0 + y1) / 2, (z0 + z1) / 2, w0 + w1, length, thickness, HANDLE,
                       0.0, math.atan2(z1 - z0, y1 - y0)))
    return tuple(out)


# A moulded hard-shell carry-on provides the ribs, zipper, corners and wheels;
# its simple rectangular collision shell keeps the physical dimensions clear.
scale = part(
    "bag_drop_scale", (*BAG_XY, TOP), [
        box(0, 0, .011, .240, .460, .022, SCALE),
        box(-.115, 0, .024, .010, .460, .007, SILVER),
        box(.115, 0, .024, .010, .460, .007, SILVER),
        box(0, -.223, .024, .240, .010, .007, SILVER),
        box(0, .223, .024, .240, .010, .007, SILVER),
    ], mass=10.0, fixed_to_world=True, yaw=BAG_YAW,
)

bag = part(
    "checked_suitcase", (*BAG_XY, TOP), [
        box(0, .010, .166, .171, .380, .306, NAVY),
        box(HANDLE_X, HANDLE_Y - .085, .310, .032, .027, .016, HANDLE),
        box(HANDLE_X, HANDLE_Y + .085, .310, .032, .027, .016, HANDLE),
    ], mass=8.0, fixed_to_world=True, friction=float(os.environ.get("BAGTAG_CASE_FRICTION", ".4")), yaw=BAG_YAW,
    sliders=(Slider("carry_handle", (0.0, 0.0, 0.0),
                    boxes=handle_arch_boxes(),
                    travel=.040, rate=90.0, damping=5.0, mass=.10,
                    axis="Z", sign=1),),
)
bag.meta["visual_usd"] = str(pathlib.Path(__file__).with_name("assets") /
                             "rolling-suitcase-side-up.usda")
# The handle is still a simple rigid collider on a spring slider; cloth
# deformation is not simulated.
bag.meta["visual_boxes"] = (
    box(HANDLE_X, HANDLE_Y - .085, .319, .041, .032, .004, SEAM),
    box(HANDLE_X, HANDLE_Y + .085, .319, .041, .032, .004, SEAM),
)
bag.meta["slider_visual_usd"] = {
    "carry_handle": str(pathlib.Path(__file__).with_name("assets") /
                        "rolling-suitcase-soft-side-handle.usda")
}
bag.meta["invisible_colliders"] = True
bag.meta["friction_combine_mode"] = "min"

# Reference stock: a 2 x 21 in (50.8 x 533 mm) thermal self-tag baggage tag with a
# self-adhesive back (POS Supply AIR-SLFTAG21RL-SS, 175 tags per roll; see README).
# The printed thermal face is the front (+z); the whole back (-z) is self-adhesive
# and bonds only to the tag itself, never to the case, handle or grippers. The
# simulated stock is 1 mm collision sheets so PhysX can resolve the gripper
# contacts. No liner, staple or pre-joined ends are present. The strip leaves the
# printer printed-side up.
from tag_dims import SEGMENTS, SEGMENT_LENGTH, TAG_WIDTH, BARCODE_PANELS, MARK_PANEL
TAG_COLLISION_THICKNESS = .001
# Real-scale stock. Mass: 177 g/m2 bag-tag stock (Sihl PRESSURETAC BT 28/3 datasheet, 170 um)
# over 50.8 x 533 mm = 4.8 g, i.e. 0.48 g per panel. Bending stiffness: baggage-tag laminates
# have a Clark stiffness (JIS P 8143) of 50-200, preferably 60-150 (Oji Paper bag-tag patent
# WO2003027950A1). We use the preferred lower value, 60 (BAGTAG_CLARK; was 150 in rev3 drafts): Clark
# stiffness S = L^3/100 (L in cm), so a 18.2 cm critical length, i.e. a
# self-weight buckling bending stiffness EI = w g L^3 / 7.837 ~ 6.8e-5 N m2 for 177 g/m2 and
# 50.8 mm width (our conversion, not a value stated in the patent; EI is proportional to S). A
# discretised beam hinge between panels of length L has k = EI / L: for 14 panels of 38.1 mm,
# 1.8e-3 N m/rad = 3.1e-5 N m/deg, the same at every hinge; EI does not depend on the panel count.
# Damping keeps the earlier damping-to-stiffness ratio. Values are USD units per degree.
PANEL_MASS = float(os.environ.get("BAGTAG_PANEL_MASS", repr(4.8e-3 / SEGMENTS)))
# Air drag. PhysX has no aerodynamics, so a hanging strip would swing undamped. Paper has a large
# area per mass: flat-plate drag F = 1/2 rho Cd A v^2 with rho = 1.204 kg/m3 (air, 20 C), Cd = 1.17
# (flat plate normal to the flow; Hoerner, Fluid-Dynamic Drag, 1965), A/m = 1/(177 g/m2), linearised
# at v = 0.5 m/s, gives c = F/(m v) = 1/2 rho Cd v (A/m) ~ 2.0 1/s. A/m and hence c do not depend on
# the panel count. Applied isotropically as PhysX linear damping (edge-on drag is smaller; this is
# an approximation).
# Paper surface friction coefficient. Paper-on-paper static friction is typically 0.3-0.5 (TAPPI T 815 /
# T 549 inclined-plane and horizontal-plane tests); 0.5 is the upper end, used for paper against the
# fingers, lid and itself. Earlier drafts used 1.25, which made released paper follow the open fingers.
PAPER_FRICTION = float(os.environ.get("BAGTAG_PAPER_FRICTION", "1.25"))
AIR_LINEAR_DAMPING = float(os.environ.get("BAGTAG_AIR_DAMPING", repr(round(.5 * 1.204 * 1.17 * .5 / .177, 2))))
CLARK_STIFFNESS = float(os.environ.get("BAGTAG_CLARK", "60"))
# 5.56e-5 N m/deg is k for Clark 150 with ten 53.3 mm panels; k scales with S and with 1/L.
HINGE_STIFFNESS = float(os.environ.get("BAGTAG_HINGE_K", repr(5.56e-5 * SEGMENTS / 10 * CLARK_STIFFNESS / 150)))
HINGE_DAMPING = float(os.environ.get("BAGTAG_HINGE_C", repr(4.85e-6 * SEGMENTS / 10 * CLARK_STIFFNESS / 150)))
# A straight internal feed channel (lengthened by 110 mm for the 533 mm stock)
# retains the trailing stock while about 90 mm of the leading end emerges from
# the printer slot. There is no
# exposed support rod; the free end can droop under its own weight.
# All panels exist at reset; none are spawned or teleported on a pull.
TAG_X0 = KIOSK_XY[0] + .165 + .103
TAG_Y = KIOSK_XY[1]
TAG_Z = TOP + .240


def barcode_boxes():
    bars = []
    for i, width in enumerate((.0020, .0011, .0027, .0014, .0030, .0010, .0020)):
        for side in (1,):   # thermal print on the front (+z) only
            bars.append(box(-.014 + i * .0042, 0, side * .00065,
                            width, .033, .0004, BLACK))
    return bars


tag = []
for i in range(SEGMENTS):
    x = TAG_X0 - (i + .5) * SEGMENT_LENGTH
    name = f"tag_panel_{i:02d}"
    hinge = None if i == 0 else (
        f"tag_panel_{i-1:02d}",
        (-SEGMENT_LENGTH / 2, 0, 0),
        (SEGMENT_LENGTH / 2, 0, 0),
        155.0, -155.0, 0.0, "Y",
        HINGE_STIFFNESS, HINGE_DAMPING, .6,
    )
    panel = part(
        name, (x, TAG_Y, TAG_Z),
        [box(0, 0, 0, SEGMENT_LENGTH, TAG_WIDTH,
             TAG_COLLISION_THICKNESS, WHITE)] +
        [],
        mass=PANEL_MASS, friction=PAPER_FRICTION,
        hinge_attachment=hinge,
        # Presenter rollers release the cut tag under a light ~2 N pull; a
        # 1 mm sheet between closed YAM pads holds only about 4 N by friction.
        breakable_attachment=("self_bag_drop_kiosk", (*KIOSK_XY, TOP),
                              2.0, .4) if i == 0 else None,
    )
    # The self-adhesive back is drawn as a thin tinted layer (visual only).
    back = (box(0, 0, -.00055, SEGMENT_LENGTH - .001, TAG_WIDTH - .001, .0001, ADHESIVE_BACK),)
    panel.meta["linear_damping"] = AIR_LINEAR_DAMPING
    # Numerical, not material: more solver iterations keep the 0.34 g hinged panels together when
    # the grippers push on them (see VALIDATION).
    panel.meta["solver_iterations"] = (int(os.environ.get("BAGTAG_POS_ITERS", "64")), int(os.environ.get("BAGTAG_VEL_ITERS", "8")))
    panel.meta["visual_boxes"] = (tuple(barcode_boxes()) if i in BARCODE_PANELS else ()) + back
    if i == MARK_PANEL:
        panel.meta["visual_boxes"] = panel.meta["visual_boxes"] + (
            box(0, 0, .00065, .022, .030, .0004, YELLOW),)
    tag.append(panel)

# Counter-height self-service bag-drop terminal next to the scale. A front
# display, check-in panel and one tag output slot identify the appliance.
kiosk = part(
    "self_bag_drop_kiosk", (*KIOSK_XY, TOP), [
        # The exterior is solid below and above a 21 mm internal feed gap.
        # The gap and front mouth are hollow collision geometry, not a visual
        # mask over a solid box; the full-size tag can slide out by contact.
        box(.055, 0, .105, .460, .145, .210, SILVER),
        box(.055, 0, .226, .460, .145, .012, SILVER),
        box(.055, 0, .294, .460, .145, .082, WHITE),
        box(.055, 0, .347, .460, .145, .080, WHITE),
        box(.055, -.069, .242, .460, .007, .030, SILVER),
        box(.055, .069, .242, .460, .007, .030, SILVER),
        box(.282, 0, .242, .006, .145, .030, SILVER),
        # Front-facing check-in display and card/receipt scanner.
        box(-.178, 0, .345, .006, .132, .070, SCALE_DARK),
        box(-.182, 0, .345, .002, .118, .058, BLUE),
        box(-.178, -.043, .302, .006, .042, .018, BLACK),
        box(-.182, -.043, .302, .002, .032, .010, YELLOW),
        # One framed outlet with an integral short lip, not projecting rods.
        box(-.178, 0, .257, .007, .090, .008, BLACK),
        box(-.178, 0, .223, .007, .090, .008, BLACK),
        box(-.178, -.050, .240, .007, .010, .034, BLACK),
        box(-.178, .050, .240, .007, .010, .034, BLACK),
        box(-.184, 0, .222, .012, .082, .004, SILVER),
        # Status lamp is visual context, not a task control.
        box(-.178, .060, .296, .003, .015, .010, YELLOW),
    ], mass=8.0, fixed_to_world=True,
)
kiosk.meta["visual_boxes"] = tuple(label("BAG DROP", .010, .015, .389,
                                         .0033, SCALE_DARK))

OBJECTS = [scale, bag, kiosk, *tag]


# The adhesive back bonds only paper to paper; never a gripper, the handle or the case.
# No position/orientation is changed when their contact bond is created.
from partkit import on_reset, on_step
import tag_closure
on_reset(tag_closure.reset)
on_step(tag_closure.step)
SCORE_OVER = "end"

def score(objects):
    return tag_closure.score(objects)

def success(objects):
    return score(objects) >= 1.0

def closure_diagnostics():
    return tag_closure.diagnostics()
