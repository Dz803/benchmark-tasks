"""Finish a four-D-cell emergency lantern, close its cover, and switch it on.

The cells, housing, cover, and spring-return button are physical rigid bodies.
Electrical continuity and the visible lamp response are a discrete proxy.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from partkit import on_reset, on_step, set_look  # noqa: E402
from assets import (CELL_CENTRE_Z, COVER_SEATED_Z, COVER_XY, D_RADIUS,
                    LANTERN_XY, LENS, LENS_ON, LOOSE_X, LOOSE_Y,
                    SLOT_PLUS_X, SLOT_X, SLOT_Y, make_scene)  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = (
    "Finish installing the emergency lantern's two loose D batteries. "
    "Inspect the + and - marks in the empty wells to orient each battery, "
    "close the battery cover, and switch the light on."
)

OBJECTS = make_scene(TOP)
CELL_NAMES = ("installed_0", "installed_3", "loose_0", "loose_1")
ALL_NAMES = ("lantern", *CELL_NAMES, "cover")
STABLE_FRAMES = 10  # half a second at 20 Hz after button release
_STATE = {"on": False, "button_down": False, "switched_with_cover": False,
          "stable_frames": 0}


def _cell_fit(lantern, cell, slot: int) -> tuple[bool, bool]:
    """Measure actual well position and nub direction in the housing frame."""
    local = lantern.to_local(cell.pos)
    positive = lantern.rotation().T @ (cell.rotation() @ np.array([1.0, 0.0, 0.0]))
    in_well = (abs(local[0] - SLOT_X) < 0.005
               and abs(local[1] - SLOT_Y[slot]) < 0.006
               and abs(local[2] - CELL_CENTRE_Z) < 0.005)
    aligned = abs(positive[0]) > math.cos(math.radians(20)) and abs(positive[2]) < 0.30
    correct = bool(in_well and aligned and positive[0] * SLOT_PLUS_X[slot] > 0)
    reversed_in_well = bool(in_well and aligned and positive[0] * SLOT_PLUS_X[slot] < 0)
    return correct, reversed_in_well


def evaluate(objects) -> dict:
    """Expose measured physical conditions; no command or demo phase is trusted."""
    lantern, cover = objects["lantern"], objects["cover"]
    fits = {slot: {name: _cell_fit(lantern, objects[name], slot)
                   for name in CELL_NAMES} for slot in range(4)}
    installed = bool(fits[0]["installed_0"][0] and fits[3]["installed_3"][0])
    loose_1 = any(fits[1][name][0] for name in ("loose_0", "loose_1"))
    loose_2 = any(fits[2][name][0] for name in ("loose_0", "loose_1"))
    # One cell cannot be double-counted in two wells; original cells must
    # remain in their original wells rather than merely lying nearby.
    distinct_loose = any(fits[1][a][0] and fits[2][b][0]
                         for a in ("loose_0", "loose_1")
                         for b in ("loose_0", "loose_1") if a != b)
    all_correct = bool(installed and distinct_loose)
    wrong = any(reversed_ for row in fits.values() for _, reversed_ in row.values())

    local_cover = lantern.to_local(cover.pos)
    lid_up = cover.rotation()[:, 2]
    housing_up = lantern.rotation()[:, 2]
    seated_cover = (abs(local_cover[0] - SLOT_X) < 0.010
                    and abs(local_cover[1]) < 0.010
                    and abs(local_cover[2] - COVER_SEATED_Z) < 0.007
                    and float(lid_up @ housing_up) > math.cos(math.radians(12)))
    button_down = abs(float(lantern.sliders.get("power_button", 0.0))) >= 0.0025
    return {"installed": installed, "loose_slots": (bool(loose_1), bool(loose_2)),
            "all_correct": all_correct, "wrong": bool(wrong),
            "cover": bool(seated_cover), "button_down": bool(button_down),
            "light_on": bool(_STATE["on"]),
            "switched_with_cover": bool(_STATE["switched_with_cover"]),
            "stable_frames": int(_STATE["stable_frames"])}


@on_reset
def _reset(environment=None):
    _STATE.update(on=False, button_down=False, switched_with_cover=False,
                  stable_frames=0)
    if environment is not None:
        set_look("lantern", LENS, LENS)


@on_step
def _monitor(environment):
    from yam_arena.scene import object_states

    state = evaluate(object_states(environment, ALL_NAMES))
    ready = state["all_correct"] and not state["wrong"]
    previous_light = _STATE["on"]
    if not ready:
        _STATE["on"] = False
        _STATE["switched_with_cover"] = False
    elif state["button_down"] and not _STATE["button_down"]:
        _STATE["on"] = not _STATE["on"]
        if _STATE["on"] and state["cover"]:
            _STATE["switched_with_cover"] = True
    _STATE["button_down"] = state["button_down"]
    if previous_light != _STATE["on"]:
        set_look("lantern", LENS, LENS_ON if _STATE["on"] else LENS,
                 emissive=_STATE["on"])

    final_pose = (ready and state["cover"] and _STATE["on"]
                  and _STATE["switched_with_cover"] and not state["button_down"])
    _STATE["stable_frames"] = _STATE["stable_frames"] + 1 if final_pose else 0


def success(objects) -> bool:
    state = evaluate(objects)
    return bool(state["all_correct"] and not state["wrong"] and state["cover"]
                and not state["button_down"] and _STATE["on"]
                and _STATE["switched_with_cover"]
                and _STATE["stable_frames"] >= STABLE_FRAMES)


def score(objects) -> float:
    state = evaluate(objects)
    # A battery seated backwards is a hard failure. The preinstalled cells
    # must also remain correct for any progress to count.
    if state["wrong"] or not state["installed"]:
        return 0.0
    if success(objects):
        return 1.0
    loose_count = sum(state["loose_slots"])
    return float(0.20 * loose_count
                 + 0.20 * (loose_count == 2 and state["cover"])
                 + 0.20 * (loose_count == 2 and state["cover"]
                           and _STATE["on"] and _STATE["switched_with_cover"]))
