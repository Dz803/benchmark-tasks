"""Attempt all three physical placements and both lid closures using YAM actions."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import imageio.v2 as imageio
import numpy as np

from assets import BIN_HEIGHT, BIN_X, BIN_Y, BOTTLE_XY, CAN_XY, HINGE_X, LID_Z, NAPKIN_XY, TOP


def run(output: Path, stop_after_lid=False, stop_after_over=False, wrong_bin_control=False):
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import ik, station
    from yam_arena.constants import READY
    from yam_arena.demo import grip_rotation
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    from yam_arena.urdf import axis_angle_matrix

    output.mkdir(parents=True, exist_ok=False)
    emb = yam_arena_embodiment(task=str(Path(__file__).with_name("task.py").resolve()),
                               object_jitter=0, control_hz=20,
                               cam_width=320, cam_height=176, top_cam="demo")
    obs = emb.reset(Scene(id="lidded_pet_fixed_0", instruction=emb.task.module.INSTRUCTION))
    from yam_arena.scene import object_states
    task = emb.task.module
    q = {s: np.asarray(READY, float).copy() for s in ("left", "right")}
    tcp, rot = {}, {}
    for s in q:
        tcp[s], rot[s] = ik.tcp_pose(q[s], np.asarray(station.BASES[s], float))
    grip = {s: 1.0 for s in q}
    frames = []
    trace_file = (output / "trace.jsonl").open("w")
    step_count = 0
    worst_ik = 0.0
    item_names = ("pet_bottle", "soup_can", "used_napkin")
    initial_items = {name: item.pos.tolist() for name, item in
                     object_states(emb._env, item_names).items()}

    def snapshot(name):
        for camera, frame in obs.images.items():
            imageio.imwrite(output / f"{name}_{camera}.png", np.asarray(frame))
        checkpoint = {"phase": name, "step": step_count,
                      "score": float(task.score(None)),
                      "items": {item_name: item.pos.tolist() for item_name, item in
                                object_states(emb._env, item_names).items()},
                      "green_angle_deg": float(task.STATE.get("green_angle_deg", 0)),
                      "black_angle_deg": float(task.STATE.get("black_angle_deg", 0))}
        (output / "checkpoint.json").write_text(json.dumps(checkpoint, indent=2) + "\n")

    def step(phase, error):
        nonlocal obs, step_count
        obs = emb.step(Action(data=pack(np.r_[q["left"], grip["left"]],
                                        np.r_[q["right"], grip["right"]]))).observation
        step_count += 1
        if step_count % 3 == 0:
            frames.append(np.asarray(obs.images["top_cam"]))
        if step_count % 5 == 0:
            items = object_states(emb._env, item_names)
            state = task.STATE
            row = {"step": step_count, "phase": phase,
                          "score": float(task.score(None)),
                          "green_angle_deg": float(state.get("green_angle_deg", 0)),
                          "black_angle_deg": float(state.get("black_angle_deg", 0)),
                          "opened": state["opened"].copy(),
                          "settled": state["settled"].copy(),
                          "stable": state["stable"].copy(),
                          "item_pos": {name: item.pos.tolist()
                                       for name, item in items.items()},
                          "tcp": {s: tcp[s].tolist() for s in q},
                          "grip": grip.copy(),
                          "ik_error_m": float(error)}
            trace_file.write(json.dumps(row) + "\n")
            trace_file.flush()

    def move(phase, side, target, opening, steps, orientation=None):
        nonlocal worst_ik
        start, r0, g0 = tcp[side].copy(), rot[side].copy(), grip[side]
        target = np.asarray(target, float)
        end_rot = rot[side] if orientation is None else orientation
        base = np.asarray(station.BASES[side], float)
        for i in range(1, steps + 1):
            u = ik.smoothstep(i / steps)
            p = start + u * (target - start)
            r = ik.slerp_matrix(r0, end_rot, u)
            q[side], e, _ = ik.solve(p, ik.matrix_to_quat_xyzw(r),
                                     q_init=q[side], base_pos=base, iters=80)
            tcp[side], rot[side], grip[side] = p, r, g0 + u * (opening - g0)
            worst_ik = max(worst_ik, float(e))
            step(phase, e)
        snapshot(phase)

    def finish(scope):
        state = task.STATE
        final_items = object_states(emb._env, item_names)
        result = {"scope": scope,
                  "attempted_full_task": scope == "complete three-item robot-action attempt",
                  "score": float(task.score(None)),
                  "success": bool(task.success(None)), "steps": step_count,
                  "run_limit": 2400, "worst_ik_error_m": worst_ik,
                  "initial_items": initial_items,
                  "final_items": {name: {"pos": item.pos.tolist(),
                                         "quat_xyzw": item.quat_xyzw.tolist(),
                                         "bounds": {"low": task._bounds(name, item)[0].tolist(),
                                                    "high": task._bounds(name, item)[1].tolist()}}
                                  for name, item in final_items.items()},
                  "task_state": {k: (v.tolist() if isinstance(v, np.ndarray) else v)
                                 for k, v in state.items()}}
        (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        trace_file.close()
        imageio.mimsave(output / "demo.mp4", frames, fps=20 / 3,
                        codec="libx264", output_params=["-pix_fmt", "yuv420p",
                                                        "-movflags", "+faststart"])
        print(json.dumps(result, indent=2))

    snapshot("initial")
    down = grip_rotation()
    if wrong_bin_control:
        black_y = BIN_Y["black"]
        tab = np.array([BIN_X - .050, black_y, TOP + LID_Z + .039])
        lid_rot = axis_angle_matrix(np.array([0., 1., 0.]), -.8) @ down
        move("black_approach", "left", tab + [0, 0, .040], 1., 35, lid_rot)
        move("black_lower", "left", tab, 1., 35, lid_rot)
        move("black_grasp", "left", tab, 0., 35, lid_rot)
        for k in range(1, 15):
            angle = math.radians(150 * k / 14)
            tilt = -.8 - .6 * k / 14
            lever = HINGE_X + .050
            x = BIN_X + HINGE_X - lever * math.cos(angle) + .026 * math.sin(angle)
            z = TOP + LID_Z + lever * math.sin(angle) + .026 * math.cos(angle)
            target = (x + .013 * math.sin(tilt), black_y, z + .013 * math.cos(tilt))
            orient = axis_angle_matrix(np.array([0., 1., 0.]), tilt) @ down
            move(f"black_open_{k:02d}", "left", target, 0., 12, orient)
        px, py = BOTTLE_XY
        pick_z = TOP + .050 + .013
        travel_z = TOP + BIN_HEIGHT + .050 + .013 + .017
        drop_z = TOP + BIN_HEIGHT + .050 + .013 - .030
        move("wrong_approach", "right", (px, py, travel_z), 1., 45, down)
        move("wrong_lower", "right", (px, py, pick_z), 1., 40, down)
        move("wrong_grasp", "right", (px, py, pick_z), 0., 35, down)
        move("wrong_lift", "right", (px, py, travel_z), 0., 45, down)
        move("wrong_over_black", "right", (BIN_X, black_y, travel_z), 0., 55, down)
        move("wrong_insert", "right", (BIN_X, black_y, drop_z), 0., 40, down)
        move("wrong_release", "right", (BIN_X, black_y, drop_z), 1., 25, down)
        move("wrong_clear", "right", (BIN_X - .06, black_y - .10, TOP + .24), 1., 40, down)
        home, home_rot = ik.tcp_pose(np.asarray(READY, float), np.asarray(station.BASES["right"], float))
        move("right_retreat", "right", home, 1., 45, home_rot)
        for _ in range(80):
            step("wrong_bin_settle", 0.)
        snapshot("wrong_bin_final")
        finish("physical wrong-bin control: recyclable bottle in general waste")
        return

    green_y = BIN_Y["green"]
    tab0 = np.array([BIN_X - .050, green_y - .055, TOP + LID_Z + .026 + .013])
    lid_rot = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), -.8) @ down
    move("lid_approach", "right", tab0 + [0, 0, .040], 1.0, 35, lid_rot)
    move("lid_lower", "right", tab0, 1.0, 35, lid_rot)
    move("lid_grasp", "right", tab0, 0.0, 35, lid_rot)
    # A Y-axis revolute joint holds the real lid. Follow its molded inboard
    # tab through the arc, with the wrist pitched toward the rear as it rises.
    for k in range(1, 15):
        angle = math.radians(150 * k / 14)
        tilt = -.8 - .6 * k / 14
        lever = HINGE_X + .050
        tab_x = BIN_X + HINGE_X - lever * math.cos(angle) + .026 * math.sin(angle)
        tab_z = TOP + LID_Z + lever * math.sin(angle) + .026 * math.cos(angle)
        target = (tab_x + .013 * math.sin(tilt), green_y - .055,
                  tab_z + .013 * math.cos(tilt))
        orient = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), tilt) @ down
        move(f"lid_arc_{k:02d}", "right", target, 0.0, 12, orient)
    for _ in range(20):
        step("lid_hold", 0.0)
    snapshot("lid_hold")
    if stop_after_lid:
        finish("robot-action hinge held-open probe; not a complete bottle demo")
        return

    px, py = BOTTLE_XY
    pick_z = TOP + .050 + .013
    travel_z = TOP + BIN_HEIGHT + .050 + .013 + .017
    drop_z = TOP + BIN_HEIGHT + .050 + .013 - .030
    move("bottle_approach", "left", (px, py, travel_z), 1.0, 40, down)
    move("bottle_lower", "left", (px, py, pick_z), 1.0, 35, down)
    move("bottle_grasp", "left", (px, py, pick_z), 0.0, 35, down)
    move("bottle_lift", "left", (px, py, travel_z), 0.0, 45, down)
    deposit_y = green_y
    move("bottle_over_green", "left", (BIN_X, deposit_y, travel_z), 0.0, 55, down)
    if stop_after_over:
        finish("robot-action bimanual held-lid and bottle-over-opening probe; no release")
        return
    move("bottle_insert", "left", (BIN_X, deposit_y, drop_z), 0.0, 40, down)
    move("bottle_release", "left", (BIN_X, deposit_y, drop_z), 1.0, 25, down)
    move("hand_clear", "left", (BIN_X - .06, green_y - .12, TOP + .23), 1.0, 40, down)
    left_home, left_home_rot = ik.tcp_pose(np.asarray(READY, float),
                                          np.asarray(station.BASES["left"], float))
    move("left_retreat", "left", left_home, 1.0, 45, left_home_rot)

    # Keep the green lid open while the left arm brings a second recyclable.
    can_rot = grip_rotation()
    can_pick = (*CAN_XY, TOP + .015 + .013)
    can_travel_z = TOP + BIN_HEIGHT + .026 + .013 + .035
    can_drop_z = TOP + BIN_HEIGHT + .026 + .013 - .030
    move("can_approach", "left", (*CAN_XY, can_travel_z), 1.0, 45, can_rot)
    move("can_lower", "left", can_pick, 1.0, 40, can_rot)
    move("can_grasp", "left", can_pick, 0.0, 35, can_rot)
    move("can_lift", "left", (*CAN_XY, can_travel_z), 0.0, 45, can_rot)
    # The first bottle settles toward the front-left of the green cavity;
    # place the smaller can at its opposite side to avoid a stack/collision.
    can_drop_x = BIN_X + .020
    can_drop_y = green_y
    move("can_over_green", "left", (can_drop_x, can_drop_y, can_travel_z),
         0.0, 60, can_rot)
    move("can_insert", "left", (can_drop_x, can_drop_y, can_drop_z),
         0.0, 45, can_rot)
    move("can_release", "left", (can_drop_x, can_drop_y, can_drop_z),
         1.0, 25, can_rot)
    move("can_clear", "left", (BIN_X - .06, green_y + .10, TOP + .24),
         1.0, 40, can_rot)
    move("left_home_after_can", "left", left_home, 1.0, 45, left_home_rot)
    for _ in range(30):
        step("green_contents_settle", 0.0)
    snapshot("green_contents")

    # Pull the green lid back through its hinge arc, release, and move clear.
    for k in range(14, -1, -1):
        angle = math.radians(150 * k / 14)
        tilt = -.8 - .6 * k / 14
        lever = HINGE_X + .050
        tab_x = BIN_X + HINGE_X - lever * math.cos(angle) + .026 * math.sin(angle)
        tab_z = TOP + LID_Z + lever * math.sin(angle) + .026 * math.cos(angle)
        target = (tab_x + .013 * math.sin(tilt), green_y - .055,
                  tab_z + .013 * math.cos(tilt))
        orient = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), tilt) @ down
        move(f"green_close_{k:02d}", "right", target, 0.0, 12, orient)
    move("green_lid_release", "right", tab0, 1.0, 25, lid_rot)
    move("green_lid_vertical_clear", "right", tab0 + [0, 0, .080],
         1.0, 30, lid_rot)
    right_home, right_home_rot = ik.tcp_pose(np.asarray(READY, float),
                                            np.asarray(station.BASES["right"], float))
    move("right_home_after_green", "right", right_home, 1.0, 45, right_home_rot)

    # Left hand opens and holds the black lid. Right hand places used paper.
    black_y = BIN_Y["black"]
    black_tab0 = np.array([BIN_X - .050, black_y, TOP + LID_Z + .039])
    move("black_lid_approach", "left", black_tab0 + [0, 0, .040],
         1.0, 35, lid_rot)
    move("black_lid_lower", "left", black_tab0, 1.0, 35, lid_rot)
    move("black_lid_grasp", "left", black_tab0, 0.0, 35, lid_rot)
    for k in range(1, 15):
        angle = math.radians(150 * k / 14)
        tilt = -.8 - .6 * k / 14
        lever = HINGE_X + .050
        tab_x = BIN_X + HINGE_X - lever * math.cos(angle) + .026 * math.sin(angle)
        tab_z = TOP + LID_Z + lever * math.sin(angle) + .026 * math.cos(angle)
        target = (tab_x + .013 * math.sin(tilt), black_y,
                  tab_z + .013 * math.cos(tilt))
        orient = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), tilt) @ down
        move(f"black_open_{k:02d}", "left", target, 0.0, 12, orient)

    napkin_pick = (*NAPKIN_XY, TOP + .002 + .013)
    napkin_travel_z = TOP + BIN_HEIGHT + .010 + .013 + .045
    napkin_drop_z = TOP + BIN_HEIGHT + .010 + .013 - .035
    move("napkin_approach", "right", (*NAPKIN_XY, napkin_travel_z),
         1.0, 45, down)
    move("napkin_lower", "right", napkin_pick, 1.0, 40, down)
    move("napkin_grasp", "right", napkin_pick, 0.0, 35, down)
    move("napkin_lift", "right", (*NAPKIN_XY, napkin_travel_z),
         0.0, 45, down)
    move("napkin_over_black", "right", (BIN_X, black_y, napkin_travel_z),
         0.0, 55, down)
    move("napkin_insert", "right", (BIN_X, black_y, napkin_drop_z),
         0.0, 40, down)
    move("napkin_release", "right", (BIN_X, black_y, napkin_drop_z),
         1.0, 25, down)
    move("napkin_clear", "right", (BIN_X - .06, black_y - .10, TOP + .24),
         1.0, 40, down)
    move("right_home_after_napkin", "right", right_home,
         1.0, 45, right_home_rot)
    for _ in range(30):
        step("black_contents_settle", 0.0)
    snapshot("black_contents")

    for k in range(14, -1, -1):
        angle = math.radians(150 * k / 14)
        tilt = -.8 - .6 * k / 14
        lever = HINGE_X + .050
        tab_x = BIN_X + HINGE_X - lever * math.cos(angle) + .026 * math.sin(angle)
        tab_z = TOP + LID_Z + lever * math.sin(angle) + .026 * math.cos(angle)
        target = (tab_x + .013 * math.sin(tilt), black_y,
                  tab_z + .013 * math.cos(tilt))
        orient = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), tilt) @ down
        move(f"black_close_{k:02d}", "left", target, 0.0, 12, orient)
    move("black_lid_release", "left", black_tab0, 1.0, 25, lid_rot)
    move("black_lid_vertical_clear", "left", black_tab0 + [0, 0, .080],
         1.0, 30, lid_rot)
    move("left_home_after_black", "left", left_home, 1.0, 45, left_home_rot)
    for _ in range(60):
        step("final_settle", 0.0)
    snapshot("final")
    finish("complete three-item robot-action attempt")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--stop-after-lid", action="store_true")
    parser.add_argument("--stop-after-over", action="store_true")
    parser.add_argument("--wrong-bin-control", action="store_true")
    args = parser.parse_args()
    run(args.output, args.stop_after_lid, args.stop_after_over, args.wrong_bin_control)
