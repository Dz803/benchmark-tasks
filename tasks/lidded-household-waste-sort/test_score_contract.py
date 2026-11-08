"""CPU score checks for the three-item contract; PhysX trials remain required."""

import math
import pathlib
import sys
import unittest

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import task
from assets import BIN_X, BIN_Y, BOTTLE_XY, CAN_XY, LID_Z, NAPKIN_XY, TOP
from partkit import PartState


def body(x, y, z, size=(.01, .01, .01), angle=0.0):
    half = np.asarray(size, float) / 2
    return PartState(pos=np.array((x, y, z), float),
                     quat_xyzw=np.array((0, math.sin(angle / 2), 0,
                                         math.cos(angle / 2)), float),
                     lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                     low=np.array((x, y, z)) - half,
                     high=np.array((x, y, z)) + half)


def scene(green_angle=0.0, black_angle=0.0, bottle=None,
          can=None, napkin=None):
    bottle = bottle or (*BOTTLE_XY, TOP + .002)
    can = can or (*CAN_XY, TOP + .030)
    napkin = napkin or (*NAPKIN_XY, TOP + .013)
    return {"green_bin": body(BIN_X, BIN_Y["green"], TOP),
            "black_bin": body(BIN_X, BIN_Y["black"], TOP),
            "green_lid": body(BIN_X, BIN_Y["green"], TOP + LID_Z,
                              angle=green_angle),
            "black_lid": body(BIN_X, BIN_Y["black"], TOP + LID_Z,
                              angle=black_angle),
            "pet_bottle": body(*bottle, size=(.050, .050, .140)),
            "soup_can": body(*can, size=(.044, .060, .045)),
            "used_napkin": body(*napkin, size=(.060, .046, .010))}


class Contract(unittest.TestCase):
    def run_states(self, states):
        task.reset()
        task._near_robot = lambda *_args, **_kwargs: True
        task._released = lambda *_args, **_kwargs: True
        for objects in states:
            task._objects = lambda _env, o=objects: o
            task.monitor(None)
        return task.score(None)

    def test_idle_and_lid_only_zero(self):
        initial = scene()
        open_green = scene(green_angle=math.radians(70))
        self.assertEqual(self.run_states([initial] * 25), 0.0)
        self.assertEqual(self.run_states([initial, open_green] * 25), 0.0)

    def test_all_items_correct_and_lids_closed(self):
        open_both = scene(green_angle=math.radians(70),
                          black_angle=math.radians(70))
        placed_open = scene(green_angle=math.radians(70),
                            black_angle=math.radians(70),
                            bottle=(BIN_X, BIN_Y["green"], TOP + .008),
                            can=(BIN_X - .050, BIN_Y["green"], TOP + .031),
                            napkin=(BIN_X, BIN_Y["black"], TOP + .013))
        placed_closed = scene(bottle=(BIN_X, BIN_Y["green"], TOP + .008),
                              can=(BIN_X - .050, BIN_Y["green"], TOP + .031),
                              napkin=(BIN_X, BIN_Y["black"], TOP + .013))
        self.assertEqual(self.run_states([scene(), open_both] + [placed_open] * 21), .95)
        task._objects = lambda _env: placed_closed
        task.monitor(None)
        self.assertEqual(task.score(None), 1.0)

    def test_wrong_bin_permanently_zero(self):
        wrong = scene(bottle=(BIN_X, BIN_Y["black"], TOP + .008))
        self.assertEqual(self.run_states([scene(), wrong]), 0.0)
        self.assertTrue(task.STATE["wrong_bin"])
        task._objects = lambda _env: scene()
        task.monitor(None)
        self.assertEqual(task.score(None), 0.0)

    def test_arena_can_visual_bounds_resting_on_bin_floor(self):
        # The YCB mesh bounds sit slightly below its contact hull in PhysX;
        # a stable can with its underside 3.5 mm above the table is valid.
        open_green = scene(green_angle=math.radians(70))
        can_on_floor = scene(green_angle=math.radians(70),
                             can=(BIN_X, BIN_Y["green"], TOP + .026))
        self.assertEqual(self.run_states([scene(), open_green]
                                         + [can_on_floor] * 21), .35)


if __name__ == "__main__":
    unittest.main()
