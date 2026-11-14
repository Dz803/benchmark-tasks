"""Inspect washed plates before they pass through a restaurant serving hatch."""

from __future__ import annotations

import math
import os
import pathlib
import sys
from collections import deque

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from partkit import on_reset, on_step
from assets import (BIN_HEIGHT, BIN_INNER_HALF, BIN_XY, PLATE_RADIUS,
                    PLATE_TILT, SERVICE_INNER_HALF, SERVICE_XY, TOP,
                    make_objects)

SCENE = "station"
SCORE_OVER = "end"
INSTRUCTION = ("Inspect the face and rim of each washed plate for chips or cracks "
               "before sorting. Place each intact plate in the tray at the "
               "serving hatch, and discard each damaged plate in the reject bin.")

# Two reproducible layouts: PLATE_SEED=0 matches the setup image; seed 1 swaps
# the chipped plate. Testing both seeds exposes a fixed-slot guessing policy.
PLATE_SEED = int(os.environ.get("PLATE_SEED", "0"))
CHIPPED_SLOT = "a" if PLATE_SEED % 2 else "b"
DAMAGED = f"plate_{CHIPPED_SLOT}"
SOUND = "plate_b" if CHIPPED_SLOT == "a" else "plate_a"
PLATES = ("plate_a", "plate_b")
OBJECTS = make_objects(CHIPPED_SLOT)
NAMES = [p.name for p in OBJECTS]

# The dish face is local +Z tilted about X; current plate yaw is zero, so this
# vector is also its reset world normal. Runtime body rotation tracks motion.
_FACE_LOCAL = np.array([0.0, -math.sin(PLATE_TILT), math.cos(PLATE_TILT)])
_INSPECT_DOT = math.cos(math.radians(42))
_INSPECT_TICKS = 12        # 0.6 s at 20 Hz: enough for a useful camera frame
_PLACE_TICKS = 20          # 1.0 s: reject pass-through poses
_BIN_CONTACT_TOL = .005    # inside the 6 mm rim: tolerate wall contact jitter
_CAMERA_POS = np.array([-0.1664948785, -0.009, 1.54182055])  # YAM demo top cam
STATE = {}


def _face_normal(body):
    return body.rotation() @ _FACE_LOCAL


def _plate_bounds_z(body, normal):
    # Bounds of a 180 mm thin disk, not the coarse AABB of the compound body.
    nz = float(np.clip(normal[2], -1.0, 1.0))
    half = PLATE_RADIUS * math.sqrt(max(0.0, 1.0 - nz * nz)) + .004 * abs(nz)
    return float(body.pos[2] - half), float(body.pos[2] + half)


def _inside_square(body, xy, half_width):
    # Exact support bounds of the tilted circular plate.
    # A constant XY radius falsely rejects a disk safely resting at a wall.
    r = body.rotation()
    normal = r @ _FACE_LOCAL
    for i in (0, 1):
        disk_half = (PLATE_RADIUS * math.sqrt(max(0.0, 1.0 - normal[i] ** 2))
                     + .004 * abs(normal[i]))
        low = float(body.pos[i] - disk_half)
        high = float(body.pos[i] + disk_half)
        if low < xy[i] - half_width or high > xy[i] + half_width:
            return False
    return True


def _gripper_near(environment, body, *, closed):
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, FINGER_OPEN

    for side in ("left", "right"):
        arm = environment.unwrapped.scene[f"{side}_robot"]
        q = arm.data.joint_pos.torch[0]
        finger_ids = [arm.joint_names.index(n) for n in FINGER_JOINTS]
        opening = float(np.clip(np.mean(q[finger_ids].detach().cpu().numpy())
                                / FINGER_OPEN, 0, 1))
        joint_ids = [arm.joint_names.index(n) for n in ARM_JOINTS]
        tcp, _ = ik.tcp_pose(q[joint_ids].detach().cpu().numpy(),
                             np.asarray(station.BASES[side], float))
        if np.linalg.norm(tcp - body.pos) < .145:
            if closed and opening < .55:
                return True
            if not closed and opening < .70:
                return True
    return False


def _released(environment, body):
    return not _gripper_near(environment, body, closed=False)


def _inspection_pose(body, environment, normal):
    if not _gripper_near(environment, body, closed=True):
        return False
    x, y, z = map(float, body.pos)
    if not (-.10 <= x <= .25 and abs(y) <= .22 and z >= TOP + .17):
        return False
    if _plate_bounds_z(body, normal)[0] < TOP + .075:
        return False
    to_cam = _CAMERA_POS - body.pos
    to_cam /= np.linalg.norm(to_cam)
    return float(np.dot(normal, to_cam)) >= _INSPECT_DOT


def _in_service(body, normal):
    low, high = _plate_bounds_z(body, normal)
    return (_inside_square(body, SERVICE_XY, SERVICE_INNER_HALF)
            and normal[2] >= .94          # usable food-contact face upward
            and TOP + .006 <= low <= TOP + .035
            and high <= TOP + .065)


def _on_service_any_face(body, normal):
    # Safety lock applies even when the chipped plate is upside-down or tilted.
    low, _ = _plate_bounds_z(body, normal)
    return (_inside_square(body, SERVICE_XY, SERVICE_INNER_HALF)
            and TOP + .003 <= low <= TOP + .040)


def _in_bin(body, normal):
    low, high = _plate_bounds_z(body, normal)
    return (_inside_square(body, BIN_XY, BIN_INNER_HALF + _BIN_CONTACT_TOL)
            and low >= TOP + .006
            and high <= TOP + BIN_HEIGHT - .006)


def _inside_bin_any_face(body, normal):
    low, high = _plate_bounds_z(body, normal)
    return (_inside_square(body, BIN_XY, BIN_INNER_HALF + _BIN_CONTACT_TOL)
            and low >= TOP + .003
            and high <= TOP + BIN_HEIGHT + .010)


def _settled(history):
    if len(history) < _PLACE_TICKS:
        return False
    p0, n0 = history[0]
    return (max(np.linalg.norm(p - p0) for p, _ in history) <= .008
            and min(float(np.dot(n, n0)) for _, n in history) >= math.cos(.12))


@on_reset
def reset(environment=None):
    STATE.clear()
    STATE.update(inspect_ticks={n: 0 for n in PLATES},
                 inspected={n: False for n in PLATES},
                 service_history=deque(maxlen=_PLACE_TICKS),
                 bin_history=deque(maxlen=_PLACE_TICKS),
                 wrong_service=False, wrong_discard=False,
                 dropped=False, score=0.0)


reset()


@on_step
def monitor(environment):
    from yam_arena.scene import object_states

    bodies = object_states(environment, PLATES)
    normals = {n: _face_normal(bodies[n]) for n in PLATES}
    sound, damaged = bodies[SOUND], bodies[DAMAGED]

    for name in PLATES:
        body = bodies[name]
        if _inspection_pose(body, environment, normals[name]):
            STATE["inspect_ticks"][name] += 1
        else:
            STATE["inspect_ticks"][name] = 0
        if STATE["inspect_ticks"][name] >= _INSPECT_TICKS:
            STATE["inspected"][name] = True
        if _plate_bounds_z(body, normals[name])[0] < TOP - .030:
            STATE["dropped"] = True

    # Wrong sorting is a historical safety failure; moving a plate later
    # cannot erase the fact that it was released at the wrong destination.
    if (_on_service_any_face(damaged, normals[DAMAGED])
            and _released(environment, damaged)):
        STATE["wrong_service"] = True
    if (_inside_bin_any_face(sound, normals[SOUND])
            and _released(environment, sound)):
        STATE["wrong_discard"] = True

    # Inspection pose is tracked for demo diagnostics, but earns no points and
    # is not a gate: the measured outcome is safe classification.
    targets = (("service_history", SOUND, _in_service),
               ("bin_history", DAMAGED, _in_bin))
    for key, name, predicate in targets:
        body, normal = bodies[name], normals[name]
        history = STATE[key]
        if predicate(body, normal) and _released(environment, body):
            history.append((body.pos.copy(), normal.copy()))
        else:
            history.clear()

    if any(STATE[k] for k in ("wrong_service", "wrong_discard", "dropped")):
        STATE["score"] = 0.0
        return

    placement_points = (.50 * int(_settled(STATE["service_history"]))
                        + .50 * int(_settled(STATE["bin_history"])))
    STATE["score"] = round(placement_points, 2)


def score(_objects):
    return float(STATE["score"])


def success(_objects):
    return STATE["score"] >= 1.0
