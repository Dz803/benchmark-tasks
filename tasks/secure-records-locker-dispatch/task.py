"""Retrieve an identified archive case from a numbered, PIN-locked locker."""

from __future__ import annotations

import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from lettering import pixels
from partkit import Slider, on_reset, on_step, part, set_look, set_slider_lock
from yam_arena.task_file import TABLE_TOP

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
CABINET_SHIFT = .200
INSTRUCTION = (
    "Records-room inventory drill: these synthetic cases contain blank forms "
    "only. Enter issued PIN 2468 at locker 27. Match the printed case code "
    "F317 and place only that sealed case flat in the green receiving tray. "
    "Close the door until its latch re-engages. Leave locker 12 and the other "
    "cases undisturbed."
)

STEEL = (0.44, 0.49, 0.52)
EDGE = (0.15, 0.19, 0.22)
FACE = (0.24, 0.30, 0.34)
BLACK = (0.025, 0.037, 0.046)
WHITE = (0.91, 0.93, 0.90)
CASE = (0.57, 0.67, 0.69)
CASE_EDGE = (0.32, 0.42, 0.45)
RED = (0.72, 0.19, 0.16)
GREEN = (0.24, 0.60, 0.43)
LOCKED_LIGHT = (0.82, 0.16, 0.10)
UNLOCKED_LIGHT = (0.12, 0.82, 0.25)


def box(x, y, z, sx, sy, sz, color):
    return ((x, y, z), (sx, sy, sz), color)


def front_label(word, x, y, z, pitch, color):
    """Raised graphics on a vertical face seen from negative world X."""
    width = (4 * len(word) - 1) * pitch
    return [box(x, y + width / 2 - (c + .5) * pitch,
                z + (2 - row) * pitch, .0005, pitch * .82, pitch * .82, color)
            for c, row in pixels(word)]


def ticket_label(word, x, y, z, pitch, color):
    """Horizontal text in the overhead camera's screen orientation."""
    width = (4 * len(word) - 1) * pitch
    return [box(x + (2 - row) * pitch,
                y + width / 2 - (c + .5) * pitch, z,
                pitch * .82, pitch * .82, .0005, color)
            for c, row in pixels(word)]


# Two independently numbered records lockers share a 220 mm deep, 420 mm wide,
# 380 mm high enclosure. Each 180 mm-high bay holds two rigid archive cases and has its
# own hinged door and physical keypad on the right-hand fascia.
frame = [
    box(0, 0, .008, .22, .42, .016, STEEL),
    box(.102, 0, .190, .016, .42, .380, STEEL),
    box(0, .202, .190, .22, .016, .380, STEEL),
    box(0, -.202, .190, .22, .016, .380, STEEL),
    box(0, 0, .372, .22, .42, .016, STEEL),
    box(0, 0, .190, .22, .42, .012, STEEL),
    box(-.109, -.155, .190, .026, .110, .380, FACE),
    box(-.006, .051, .096, .192, .296, .008, STEEL),
    box(-.006, .051, .275, .192, .296, .008, STEEL),
    box(-.123, -.155, .100, .003, .106, .165, EDGE),
    box(-.123, -.155, .280, .003, .106, .165, EDGE),
]
# Each cap has 7 mm travel and 30 mm horizontal spacing. Independent key names
# preserve which locker was addressed even when identical digits are pressed.
sliders = []
button_rows = (("1", "2", "3"), ("4", "5", "6"),
               ("7", "8", "9"), ("C", "0", "E"))
for locker, base_z in ((12, .149), (27, .329)):
    for row, chars in enumerate(button_rows):
        for col, char in enumerate(chars):
            y = -.125 - .030 * col
            z = base_z - .034 * row
            rest = (-.127, y, z)
            glyph = tuple(front_label(char, -.131, y, z, .0020, WHITE))
            sliders.append(Slider(
                name=f"locker{locker}_key_{char.lower()}", rest=rest,
                boxes=(box(-.127, y, z, .006, .023, .027, BLACK),),
                visual_boxes=glyph, travel=.007, rate=320., damping=4.,
                mass=.006, axis="X", sign=1,
            ))

# A narrow spring-driven steel bolt lies just behind each closed door panel
# and in front of its rear-facing strike. It retracts 20 mm into the fascia
# when a valid code is entered. The cabinet articulation drives the bolt
# joint; no body pose is written by the controller.
for locker, center_z in ((12, .100), (27, .280)):
    sliders.append(Slider(
        name=f"bolt_{locker}", rest=(-.113, -.093, center_z),
        boxes=(box(-.113, -.093, center_z, .003, .018, .026, STEEL),),
        travel=.020, rate=450., damping=12., mass=.018,
        axis="Y", sign=-1,
    ))

CABINET = part("cabinet", pos=(.40 + CABINET_SHIFT, 0, TOP), mass=8.0,
               boxes=frame, sliders=sliders, fixed_to_world=True,
               friction=.75)
CABINET.meta["visual_boxes"] = (
    box(-.124, -.155, .366, .001, .022, .009, LOCKED_LIGHT),
)

DOORS = []
for locker, center_z in ((12, .100), (27, .280)):
    door_boxes = [
        box(0, 0, 0, .012, .300, .165, FACE),
        box(-.010, 0, .078, .006, .298, .007, EDGE),
        box(-.010, 0, -.078, .006, .298, .007, EDGE),
        box(-.017, -.118, -.034, .025, .012, .014, STEEL),
        box(-.017, -.118, .034, .025, .012, .014, STEEL),
        box(-.040, -.118, 0, .014, .012, .082, STEEL),
        # Rear strike: its front face stops against an extended bolt as the
        # door is pulled outward, but clears a retracted bolt.
        box(.018, -.138, 0, .014, .020, .030, STEEL),
    ]
    door = part(f"door_{locker}", pos=(.278 + CABINET_SHIFT, .051, TOP + center_z), mass=.95,
                boxes=door_boxes, friction=.7,
                hinge_attachment=("cabinet", (-.122, .201, center_z),
                                  (0, .150, 0), 0.0, -105.0))
    door.meta["visual_boxes"] = tuple(
        front_label(f"BOX {locker}", -.007, .023, .032, .0052, WHITE)
        + front_label("RECORDS", -.007, .023, .002, .0028, WHITE)
    )
    DOORS.append(door)


# The C5-footprint cases have a continuous, raised front rim which permits a
# vertical pinch without putting a jaw beneath the shelf. No narrow pull tab
# or hidden grasp proxy is attached. Two initial shelf assignments test that
# the label, rather than a fixed shelf position, determines the selected case.
LAYOUT = os.environ.get("ROBOCURVE_RECORDS_LAYOUT", "A").upper()
if LAYOUT not in ("A", "B"):
    raise ValueError("ROBOCURVE_RECORDS_LAYOUT must be A or B")
target_level, distractor_level = ((.198, .281) if LAYOUT == "A" else (.281, .198))
DOCUMENTS = ((12, "F104", .018), (12, "F263", .101),
             (27, "F317", target_level), (27, "F892", distractor_level))
DOCUMENT_IDS = tuple(identifier for _, identifier, _ in DOCUMENTS)
DOCUMENT_HOMES = {identifier: np.array([.407 + CABINET_SHIFT, .051, TOP + level])
                  for _, identifier, level in DOCUMENTS}
OBJECTS = [CABINET, *DOORS]
for locker, identifier, level in DOCUMENTS:
    document_boxes = [
        box(0, 0, .011, .162, .229, .022, CASE),
        box(0, 0, .0225, .158, .225, .003, CASE_EDGE),
        box(-.080, 0, .0235, .008, .229, .019, CASE_EDGE),
        box(-.085, 0, .0235, .004, .222, .016, CASE),
        box(-.070, 0, .025, .004, .205, .002, RED),
    ]
    document = part(f"document_{identifier}", pos=(.407 + CABINET_SHIFT, .051, TOP + level),
                    mass=.22, boxes=document_boxes, friction=1.0)
    document.meta["visual_boxes"] = tuple(
        front_label(identifier, -.088, .070, .025, .0040, BLACK)
        + front_label("SEALED CASE", -.088, -.040, .025, .0021, RED)
    )
    OBJECTS.append(document)


# A shallow receiving tray sits beside the cabinet. Its low rim keeps the
# extracted case clear of the tray during the lateral transfer.
tray = part("transfer_tray", pos=(.365, -.420, TOP), mass=1.4,
            boxes=(
                box(0, 0, .012, .350, .390, .010, GREEN),
                box(-.171, 0, .035, .008, .390, .036, GREEN),
                box(.171, 0, .035, .008, .390, .036, GREEN),
                box(0, -.191, .035, .350, .008, .036, GREEN),
                box(0, .191, .035, .350, .008, .036, GREEN),
            ), fixed_to_world=True)
ticket = part("dispatch_ticket", pos=(.118, .145, TOP + .001),
              mass=.03, boxes=(box(0, 0, .002, .155, .112, .004, WHITE),),
              fixed_to_world=True)
ticket.meta["visual_boxes"] = tuple(
    ticket_label("DISPATCH", .042, 0, .0043, .0026, BLACK)
    + ticket_label("BOX 27", .006, 0, .0043, .0038, BLACK)
    + ticket_label("F317", -.017, 0, .0043, .0038, BLACK)
    + ticket_label("PIN 2468", -.048, 0, .0043, .0027, BLACK)
)
OBJECTS += [tray, ticket]


STATE = {}


@on_reset
def reset(environment=None):
    STATE.clear()
    STATE.update(buffer="", key_armed=True, key_events=[],
                 authorized=False, access=False, opened=False,
                 ever_open=False, relock_requested=False,
                 invalid=False, placed_ticks=0,
                 final_ticks=0, door_deg=0.0, bolt_m=0.0,
                 score=0.0)
    if environment is not None:
        for locker in (12, 27):
            set_slider_lock(environment, "cabinet", f"bolt_{locker}",
                            locked=True, target=0.0)
        set_look("cabinet", LOCKED_LIGHT, LOCKED_LIGHT, emissive=True)


def _hands_clear(environment, points):
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS

    for side in ("left", "right"):
        robot = environment.unwrapped.scene[f"{side}_robot"]
        q = robot.data.joint_pos.torch[0].detach().cpu().numpy()
        indices = [robot.joint_names.index(n) for n in ARM_JOINTS]
        tcp, _ = ik.tcp_pose(q[indices], station.BASES[side])
        if min(float(np.linalg.norm(tcp - p)) for p in points) < .10:
            return False
    return True


def _in_tray(case):
    # Test all footprint corners, not just the case centre; the tray walls
    # are 8 mm thick and the placement has a small practical margin.
    corners = [case.to_world((x, y, 0.0))
               for x in (-.081, .081) for y in (-.1145, .1145)]
    inside = all(.205 <= p[0] <= .525 and -.600 <= p[1] <= -.240
                 for p in corners)
    flat = case.rotation()[2, 2] >= .90
    settled = (float(np.linalg.norm(case.lin_vel)) < .02
               and float(np.linalg.norm(case.ang_vel)) < .2)
    on_floor = TOP + .015 <= min(p[2] for p in corners) <= TOP + .042
    return inside and flat and settled and on_floor


@on_step
def monitor(environment):
    from yam_arena.scene import object_states

    names = ("cabinet", "door_12", "door_27", *(
        f"document_{identifier}" for identifier in DOCUMENT_IDS))
    states = object_states(environment, names)
    cabinet, target_door = states["cabinet"], states["door_27"]
    target = states["document_F317"]
    angle12 = -float(np.arctan2(states["door_12"].rotation()[1, 0],
                                states["door_12"].rotation()[0, 0]))
    angle27 = -float(np.arctan2(target_door.rotation()[1, 0],
                                target_door.rotation()[0, 0]))
    STATE["door_deg"] = float(np.degrees(angle27))
    STATE["bolt_m"] = cabinet.sliders["bolt_27"]

    down = [char for row in button_rows for char in row
            if cabinet.sliders[f"locker27_key_{char.lower()}"] >= .004]
    fully_released = all(
        cabinet.sliders[f"locker27_key_{char.lower()}"] <= .002
        for row in button_rows for char in row)
    if fully_released:
        STATE["key_armed"] = True
    elif STATE["key_armed"]:
        STATE["key_armed"] = False
        if len(down) == 1:
            char = down[0]
            STATE["key_events"].append(char)
            if char == "C":
                STATE["buffer"] = ""
            elif char == "E":
                valid = STATE["buffer"] == "2468"
                STATE["buffer"] = ""
                if valid and not STATE["invalid"]:
                    STATE["authorized"] = True
                    set_slider_lock(environment, "cabinet", "bolt_27",
                                    locked=True, target=-.020)
                    set_look("cabinet", LOCKED_LIGHT, UNLOCKED_LIGHT,
                             emissive=True)
            else:
                STATE["buffer"] = (STATE["buffer"] + char)[:8]
        else:
            STATE["buffer"] = ""

    if angle12 > np.deg2rad(10):
        STATE["invalid"] = True
    if angle27 > np.deg2rad(10) and not STATE["access"]:
        STATE["invalid"] = True
    for locker, identifier, level in DOCUMENTS:
        if identifier == "F317":
            continue
        if np.linalg.norm(states[f"document_{identifier}"].pos - DOCUMENT_HOMES[identifier]) > .040:
            STATE["invalid"] = True
    if np.linalg.norm(target.pos - DOCUMENT_HOMES["F317"]) > .040 \
            and not STATE["opened"]:
        STATE["invalid"] = True

    if STATE["authorized"] and cabinet.sliders["bolt_27"] <= -.015:
        STATE["access"] = True
    if angle27 > np.deg2rad(15):
        STATE["ever_open"] = True
    if STATE["access"] and angle27 >= np.deg2rad(50):
        STATE["opened"] = True
    if STATE["ever_open"] and not STATE["relock_requested"] \
            and angle27 <= np.deg2rad(5) \
            and cabinet.sliders["bolt_27"] <= -.015:
        STATE["relock_requested"] = True
        set_slider_lock(environment, "cabinet", "bolt_27",
                        locked=True, target=0.0)
        set_look("cabinet", LOCKED_LIGHT, LOCKED_LIGHT, emissive=True)

    tray_point = np.array([.365, -.420, TOP + .04])
    case_points = (target.pos,
                   target.to_world((-.085, 0, .024)),
                   *(target.to_world((x, y, .006))
                     for x in (-.081, .081) for y in (-.1145, .1145)))
    # Holding the open door with one arm is allowed during transfer. Full
    # completion, however, requires release of the actual pull bar, not just
    # clearance from the door body's origin (which is 13 cm from the bar).
    transfer_clear = _hands_clear(environment, (*case_points, tray_point))
    final_clear = _hands_clear(environment, (
        *case_points, tray_point,
        target_door.to_world((-.040, -.118, 0.0))))
    placed_now = STATE["opened"] and _in_tray(target) and transfer_clear
    STATE["placed_ticks"] = STATE["placed_ticks"] + 1 if placed_now else 0
    final_now = (STATE["placed_ticks"] >= 20
                 and abs(angle27) <= np.deg2rad(5)
                 and cabinet.sliders["bolt_27"] >= -.003 and final_clear)
    STATE["final_ticks"] = STATE["final_ticks"] + 1 if final_now else 0
    if STATE["invalid"]:
        STATE["score"] = 0.0
    else:
        STATE["score"] = (0.15 * STATE["access"]
                          + 0.20 * STATE["opened"]
                          + 0.45 * (STATE["placed_ticks"] >= 20)
                          + 0.20 * (STATE["final_ticks"] >= 20))


reset()


def score(_objects):
    return float(STATE["score"])


def success(_objects):
    return STATE["score"] >= 1.0
