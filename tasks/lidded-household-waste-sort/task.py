"""Sort three household waste items through physically hinged bin lids."""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from partkit import on_reset, on_step
from assets import (BIN_HEIGHT, BIN_INNER_RADIUS, BIN_X, BIN_Y,
                    BOTTLE_HEIGHT, BOTTLE_RADIUS, TOP, make_objects)

SCENE = "station"
SCORE_OVER = "end"
INSTRUCTION = ("Sort the empty plastic drinks bottle, empty metal soup can, "
               "and food-stained paper napkin. Put the bottle and can in the "
               "green MIXED RECYCLING bin and the used napkin in the black "
               "GENERAL WASTE bin. Open each lid to put items in, then leave "
               "both lids closed. Do not put anything in the wrong bin.")
OBJECTS = make_objects()
NAMES = [p.name for p in OBJECTS]
STATE = {}


def _objects(environment):
    from yam_arena.scene import object_states
    return object_states(environment, NAMES)


def _grippers(environment):
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, FINGER_OPEN
    base = environment.unwrapped
    out = []
    for side in ("left", "right"):
        asset = base.scene[f"{side}_robot"]
        q = asset.data.joint_pos.torch[0]
        fingers = [asset.joint_names.index(n) for n in FINGER_JOINTS]
        opening = float(np.clip(np.mean(q[fingers].detach().cpu().numpy()) / FINGER_OPEN, 0, 1))
        joints = [asset.joint_names.index(n) for n in ARM_JOINTS]
        tcp, _ = ik.tcp_pose(q[joints].detach().cpu().numpy(),
                             np.asarray(station.BASES[side], float))
        out.append((opening, np.asarray(tcp, float)))
    return out


def _near_robot(environment, point, radius=.13, max_opening=.8):
    return any(op < max_opening and np.linalg.norm(tcp - point) < radius
               for op, tcp in _grippers(environment))


def _released(environment, point):
    return all(op > .7 or np.linalg.norm(tcp - point) > .11
               for op, tcp in _grippers(environment))


def _lid_angle(lid, bin_body):
    relative = bin_body.rotation().T @ lid.rotation()
    return max(0.0, math.atan2(float(relative[0, 2]), float(relative[0, 0])))


def _radius(point, kind):
    return float(np.linalg.norm(np.asarray(point[:2]) - (BIN_X, BIN_Y[kind])))


DESTINATION = {"pet_bottle": "green", "soup_can": "green",
               "used_napkin": "black"}


def _bottle_surface(item):
    # Physical shell is circular, not the diagonal of its square AABB. Sample
    # the wide body and smaller cap at their real axial extents.
    circles = ((0.0, BOTTLE_RADIUS), (.114, BOTTLE_RADIUS),
               (BOTTLE_HEIGHT, .015))
    return np.asarray([item.to_world((radius * math.cos(theta),
                                      radius * math.sin(theta), z))
                       for z, radius in circles
                       for theta in np.linspace(0, 2 * math.pi, 32,
                                                endpoint=False)], float)


def _bounds(name, item):
    # Procedural bottle geometry is authored from its base, whereas the
    # generic library AABB assumes a centre-origin object. Use its actual
    # oriented extrema to avoid a false underside below the table.
    if name == "pet_bottle":
        surface = _bottle_surface(item)
        return surface.min(axis=0), surface.max(axis=0)
    return np.asarray(item.low, float), np.asarray(item.high, float)


def _bin_of(name, item):
    # The entire projected AABB must be inside one cavity. This avoids
    # credit for an item balanced across a rim or partly outside the bin.
    low, high = _bounds(name, item)
    for kind in ("green", "black"):
        if name == "pet_bottle":
            radial = max(_radius(point, kind) for point in _bottle_surface(item))
            limit = BIN_INNER_RADIUS + .001  # 1 mm contact tolerance
        else:
            radial = max(_radius((x, y), kind)
                         for x in (low[0], high[0])
                         for y in (low[1], high[1]))
            limit = BIN_INNER_RADIUS - .003
        if (radial < limit
                and low[2] < TOP + BIN_HEIGHT - .010):
            return kind
    return None


def _settled(name, item, environment):
    low, high = _bounds(name, item)
    point = np.asarray(item.pos, float)
    # One discarded item may rest on another; requiring every underside to
    # touch the bin floor would reject a valid filled household bin.
    # The Arena YCB can's visual AABB extends slightly below its contact hull;
    # its measured settled underside is 4.4 mm above the tabletop in the bin.
    min_under = TOP + (.003 if name == "soup_can" else .006)
    return (low[2] >= min_under and high[2] < TOP + BIN_HEIGHT - .002
            and np.linalg.norm(item.lin_vel) < .030
            and np.linalg.norm(item.ang_vel) < .35
            and _released(environment, point))


def _bottle_bounds(bottle):
    surface = _bottle_surface(bottle)
    return surface.min(axis=0), surface.max(axis=0)


@on_reset
def reset(environment=None):
    STATE.clear()
    STATE.update(opened={"green": False, "black": False},
                 last_angle={"green": 0.0, "black": 0.0},
                 stable={name: 0 for name in DESTINATION},
                 settled={name: False for name in DESTINATION},
                 wrong_bin=False, score=0.0)


reset()


@on_step
def monitor(environment):
    objects = _objects(environment)
    for kind in ("green", "black"):
        angle = _lid_angle(objects[f"{kind}_lid"], objects[f"{kind}_bin"])
        STATE[f"{kind}_angle_deg"] = math.degrees(angle)
        delta = angle - STATE["last_angle"][kind]
        if (angle >= math.radians(60) and delta > .002
                and _near_robot(environment, objects[f"{kind}_lid"].pos, .15)):
            STATE["opened"][kind] = True
        STATE["last_angle"][kind] = angle

    for name, destination in DESTINATION.items():
        item = objects[name]
        actual = _bin_of(name, item)
        if actual is not None and actual != destination:
            STATE["wrong_bin"] = True
        if actual == destination and _settled(name, item, environment):
            STATE["stable"][name] += 1
        else:
            STATE["stable"][name] = 0
        STATE["settled"][name] = (STATE["stable"][name] >= 20
                                  and STATE["opened"][destination])

    if STATE["wrong_bin"]:
        STATE["score"] = 0.0
        return
    settled = STATE["settled"]
    score = .25 * sum(settled.values())
    for kind in ("green", "black"):
        if STATE["opened"][kind] and any(
                settled[name] for name, dest in DESTINATION.items()
                if dest == kind):
            score += .10
    if (all(settled.values())
            and STATE["green_angle_deg"] <= 20
            and STATE["black_angle_deg"] <= 20):
        score += .05
    STATE["score"] = round(score, 3)


def score(_objects):
    return float(STATE["score"])


def success(_objects):
    return STATE["score"] == 1.0
