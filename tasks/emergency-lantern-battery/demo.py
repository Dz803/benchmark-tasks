"""Robot-action-only four-D-cell lantern demonstration; no pose writes.

Usage: python demo.py OUT_DIR [--max-steps 1800]
Records all three YAM cameras, a per-step physical trace, and the final score.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import imageio.v2 as imageio
import numpy as np

CONTROL_HZ = 20
PAD_TIP = 0.013


class Arm:
    def __init__(self, side):
        from yam_arena import ik, station
        from yam_arena.constants import READY

        self.side = side
        self.base = np.asarray(station.BASES[side], float)
        self.q = np.array(READY, float)
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.home = (self.pos.copy(), self.rot.copy())
        self.grip = 1.0
        self.max_error = 0.0

    def move(self, pos, rot, seconds, label, grip=None):
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(seconds * CONTROL_HZ))
        for i in range(1, n + 1):
            u = ik.smoothstep(i / n)
            p = p0 + u * (np.asarray(pos, float) - p0)
            r = ik.slerp_matrix(r0, rot, u)
            self.q, error, _ = ik.solve(p, ik.matrix_to_quat_xyzw(r), q_init=self.q,
                                         base_pos=self.base, iters=80)
            self.max_error = max(self.max_error, float(error))
            self.pos, self.rot, self.grip = p, r, g0 + u * (g1 - g0)
            yield label

    def hold(self, seconds, label):
        for _ in range(max(1, round(seconds * CONTROL_HZ))):
            yield label

    def command(self):
        return np.r_[self.q, self.grip]


def plan(task, left, right, get_states):
    from yam_arena.demo import grip_rotation

    down = grip_rotation(0.0)

    def place_cell(arm, stage_y, slot, turn):
        stage_x = task.LOOSE_X
        cell_name = "loose_1" if arm.side == "left" else "loose_0"
        stage_z = task.TOP + task.D_RADIUS + PAD_TIP - 0.004
        slot_x = task.LANTERN_XY[0] + task.SLOT_X
        slot_y = task.LANTERN_XY[1] + task.SLOT_Y[slot]
        if arm.side == "right":
            # The right gripper settles a few mm toward the near rail after
            # release; center its drop within the measured well clearance.
            slot_y += 0.006
        # Release just above the well: the fingers stay above the divider
        # rails, then gravity settles the cell onto the real holder floor.
        slot_z = task.TOP + 0.068
        yield from arm.move((stage_x, stage_y, task.TOP + 0.13), down, 2.0, "cell_approach")
        cell_xy = get_states()[cell_name].pos[:2]
        grasp_x, grasp_y = cell_xy
        yield from arm.move((grasp_x, grasp_y, task.TOP + 0.13), down, 0.8,
                            "cell_center_above")
        yield from arm.move((grasp_x, grasp_y, stage_z), down, 2.0, "cell_lower")
        yield from arm.move(arm.pos, down, 1.0, "cell_grasp", grip=0.0)
        yield from arm.hold(0.5, "cell_grasp_hold")
        yield from arm.move((grasp_x, grasp_y, task.TOP + 0.125), down, 2.0, "cell_lift")
        if turn:
            yield from arm.move(arm.pos, grip_rotation(math.pi), 2.0, "cell_turn")
        target_rot = grip_rotation(math.pi if turn else 0.0)
        yield from arm.move((slot_x, slot_y, task.TOP + 0.125), target_rot, 2.5, "cell_carry")
        # Cylinder ends have only a few millimetres of clearance at the
        # contacts. Correct grasp slip with a small robot motion measured from
        # the actual rigid-body pose; never write an object pose.
        for _ in range(2):
            states = get_states()
            cell_xy = states[cell_name].pos[:2]
            goal_xy = np.array([slot_x, slot_y])
            delta = np.clip(goal_xy - cell_xy, -0.014, 0.014)
            if np.linalg.norm(delta) < 0.001:
                break
            yield from arm.move((arm.pos[0] + delta[0], arm.pos[1] + delta[1], arm.pos[2]),
                                target_rot, 1.2, "cell_align")
        insert_x, insert_y = arm.pos[:2]
        yield from arm.move((insert_x, insert_y, slot_z), target_rot, 2.5, "cell_insert")
        yield from arm.hold(0.4, "cell_settle")
        yield from arm.move(arm.pos, target_rot, 0.8, "cell_release", grip=1.0)
        yield from arm.move((insert_x, insert_y, task.TOP + 0.105), target_rot,
                            0.8, "cell_pusher_prepare", grip=0.0)
        yield from arm.move((insert_x, insert_y, task.TOP + 0.067), target_rot,
                            1.0, "cell_press_seat")
        yield from arm.hold(0.5, "cell_press_hold")
        yield from arm.move((insert_x, insert_y, task.TOP + 0.125), target_rot,
                            1.5, "cell_clear", grip=1.0)
        safe_y = 0.28 if arm.side == "left" else -0.28
        yield from arm.move((0.29, safe_y, task.TOP + 0.125), target_rot, 2.0, "cell_move_out")

    # Each loose cell starts in one of the two natural terminal directions.
    # Select the well whose visible marking matches that cell's positive end.
    yield from place_cell(left, task.LOOSE_Y[1], 2, False)
    yield from left.move(*left.home, 2.0, "left_retreat")
    yield from place_cell(right, task.LOOSE_Y[0], 1, False)
    yield from right.move(*right.home, 2.0, "right_retreat")

    # Grasp the cover's thumb ridge and lower its inner rim into the battery
    # bay channel. No object pose is written by this controller.
    x, y = task.COVER_XY
    grip_x = x
    lid_grasp_z = task.TOP + 0.0215 + 0.024 + PAD_TIP
    yield from left.move((grip_x, y, task.TOP + 0.16), down, 2.0, "lid_approach")
    yield from left.move((grip_x, y, lid_grasp_z), down, 2.0, "lid_lower")
    yield from left.move(left.pos, down, 1.0, "lid_grasp", grip=0.0)
    yield from left.hold(0.4, "lid_grasp_hold")
    yield from left.move((grip_x, y, task.TOP + 0.14), down, 2.0, "lid_lift")
    ax, ay = task.LANTERN_XY
    goal_grip_x = ax + task.SLOT_X
    yield from left.move((goal_grip_x, ay, task.TOP + 0.14), down, 2.5, "lid_carry")
    # Align the observed cover body over the actual housing before the inner
    # rim enters its narrow channel. This is robot motion, not a pose write.
    for _ in range(4):
        states = get_states()
        goal_xy = states["lantern"].to_world((task.SLOT_X, 0.0, 0.0))[:2]
        error_xy = goal_xy - states["cover"].pos[:2]
        if np.linalg.norm(error_xy) < 0.0005:
            break
        delta = np.clip(error_xy, -0.012, 0.012)
        yield from left.move((left.pos[0] + delta[0], left.pos[1] + delta[1],
                              left.pos[2]), down, 1.0, "lid_align")
    yield from left.move((left.pos[0], left.pos[1],
                          task.TOP + task.COVER_SEATED_Z + 0.024 + PAD_TIP + 0.003),
                         down, 2.5, "lid_place_lower")
    yield from left.move((left.pos[0], left.pos[1],
                          task.TOP + task.COVER_SEATED_Z + 0.024 + PAD_TIP),
                         down, 0.8, "lid_press_fit")
    yield from left.hold(0.5, "lid_settle")
    yield from left.move(left.pos, down, 0.9, "lid_release", grip=1.0)
    yield from left.move((goal_grip_x, left.pos[1], task.TOP + 0.14), down, 1.5, "lid_clear")
    yield from left.move((0.29, 0.28, task.TOP + 0.14), down, 2.0, "lid_move_out")
    yield from left.move(*left.home, 2.0, "lid_arm_retreat")

    # Depress and release the real spring-return slider after closing the lid.
    bx, by = ax + 0.092, ay - 0.068
    yield from right.move((bx, by, task.TOP + 0.15), down, 2.0, "switch_approach", grip=0.0)
    yield from right.move((bx, by, task.TOP + 0.098), down, 2.0, "switch_lower")
    yield from right.move((bx, by, task.TOP + 0.089), down, 1.5, "switch_press")
    yield from right.hold(1.0, "switch_hold")
    yield from right.move((bx, by, task.TOP + 0.15), down, 1.5, "switch_release")
    yield from right.hold(1.0, "final_settle")


def run(out_dir: str, max_steps: int = 1800):
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=False)
    path = str(Path(__file__).with_name("task.py").resolve())
    emb = yam_arena_embodiment(task=path, object_jitter=0, control_hz=CONTROL_HZ,
                               cam_width=640, cam_height=352, top_cam="demo")
    obs = emb.reset(Scene(id="demo", instruction="demo"))
    from yam_arena.scene import object_states

    task = emb.task.module
    left, right = Arm("left"), Arm("right")
    frames = {cam: [np.asarray(img)] for cam, img in obs.images.items()}
    trace = []
    ended = False
    stopped_at = None
    previous_label = None
    for label in plan(task, left, right,
                      lambda: object_states(emb._env, task.ALL_NAMES)):
        if len(trace) >= max_steps:
            break
        if label != previous_label and trace:
            stage_min = {("left_retreat", "cell_approach"): 0.19,
                         ("right_retreat", "lid_approach"): 0.39,
                         ("lid_arm_retreat", "switch_approach"): 0.59}.get((previous_label, label))
            if stage_min is not None and trace[-1]["score"] < stage_min:
                stopped_at = f"after {previous_label}: score {trace[-1]['score']:.2f} < {stage_min:.2f}"
                print(stopped_at, flush=True)
                break
        previous_label = label
        result = emb.step(Action(data=pack(left.command(), right.command())))
        if len(trace) % 2 == 0:
            for cam, img in result.observation.images.items():
                frames[cam].append(np.asarray(img))
        states = object_states(emb._env, task.ALL_NAMES)
        status = task.evaluate(states)
        record = {"step": len(trace), "phase": label, "score": task.score(states),
                  "installed": status["installed"], "loose_slots": status["loose_slots"],
                  "all_correct": status["all_correct"], "wrong": status["wrong"],
                  "cover": status["cover"], "button_m": states["lantern"].sliders["power_button"],
                  "light_on": task._STATE["on"],
                  "switched_with_cover": task._STATE["switched_with_cover"],
                  "stable_frames": task._STATE["stable_frames"],
                  "loose_0_local": states["lantern"].to_local(states["loose_0"].pos).tolist(),
                  "loose_1_local": states["lantern"].to_local(states["loose_1"].pos).tolist(),
                  "loose_0_positive_axis": (states["lantern"].rotation().T @
                                             states["loose_0"].rotation()[:, 0]).tolist(),
                  "loose_1_positive_axis": (states["lantern"].rotation().T @
                                             states["loose_1"].rotation()[:, 0]).tolist(),
                  "cover_local": states["lantern"].to_local(states["cover"].pos).tolist(),
                  "cover_up": (states["lantern"].rotation().T @
                               states["cover"].rotation()[:, 2]).tolist(),
                  "lantern_world": states["lantern"].pos.tolist()}
        trace.append(record)
        if (len(trace) + 1) % 100 == 0:
            print(json.dumps(record), flush=True)
        if result.terminated:
            ended = True
            break
    states = object_states(emb._env, task.ALL_NAMES)
    score = task.score(states)
    report = {"score": score, "success": task.success(states), "terminated": ended,
              "stopped_at": stopped_at,
              "steps": len(trace), "evaluation": task.evaluate(states),
              "switch_state": dict(task._STATE),
              "max_ik_error_m": {"left": left.max_error, "right": right.max_error},
              "poses": {name: states[name].pos.tolist() for name in states},
              "quaternions_xyzw": {name: states[name].quat_xyzw.tolist() for name in states},
              "note": "All movements, including battery insertion and cover seating, used YAM actions; no object pose writes."}
    (out / "result.json").write_text(json.dumps(report, indent=2) + "\n")
    (out / "trace.jsonl").write_text("".join(json.dumps(x) + "\n" for x in trace))
    for cam, images in frames.items():
        imageio.mimsave(out / f"demo_{cam}.mp4", images, fps=CONTROL_HZ // 2, codec="libx264",
                        output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])
    imageio.imwrite(out / "setup.png", frames["top_cam"][0])
    imageio.imwrite(out / "final.png", frames["top_cam"][-1])
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("out_dir")
    parser.add_argument("--max-steps", type=int, default=1800)
    args = parser.parse_args()
    run(args.out_dir, args.max_steps)
