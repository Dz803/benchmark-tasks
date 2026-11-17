"""Fixed-layout robot-action contact check; never changes task object state mid-episode."""

from __future__ import annotations

import json
from pathlib import Path
import sys

import imageio.v2 as imageio
import numpy as np


def run(out: Path, guard_only: bool, blind: bool, fast: bool = False,
        aim_only: bool = False):
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import ik, station
    from yam_arena.constants import READY
    from yam_arena.demo import grip_rotation
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    from yam_arena.urdf import axis_angle_matrix

    out.mkdir(parents=True, exist_ok=False)
    emb = yam_arena_embodiment(
        task=str(Path(__file__).with_name("task.py").resolve()),
        object_jitter=0, control_hz=20, cam_width=320 if fast else 640,
        cam_height=176 if fast else 352, top_cam="demo")
    trace = []
    actions = []
    writer = None
    try:
        obs = emb.reset(Scene(id="blackout_contact_demo", instruction=emb.task.module.INSTRUCTION))
        if not fast:
            writer = imageio.get_writer(out / "demo_top.mp4", fps=20, codec="libx264",
                                        pixelformat="yuv420p", quality=8)
            writer.append_data(np.asarray(obs.images["top_cam"]))
        from yam_arena.scene import object_states
        task = emb.task.module
        ql = np.asarray(task.INITIAL_ACTION[:6], float)
        qr = np.asarray(READY, float)
        gl, gr = 0.0, 1.0
        bases = [np.asarray(station.BASES[name]) for name in ("left", "right")]
        tcp_l, rot_l = ik.tcp_pose(ql, bases[0])
        tcp_r, rot_r = ik.tcp_pose(qr, bases[1])
        max_ik_error = 0.0

        def shot(label):
            for name, frame in obs.images.items():
                imageio.imwrite(out / f"{label}_{name}.png", np.asarray(frame))

        def step(label):
            nonlocal obs
            action = pack(np.r_[ql, gl], np.r_[qr, gr])
            actions.append(np.asarray(action, dtype=np.float64).copy())
            result = emb.step(Action(data=action))
            obs = result.observation
            if writer is not None:
                writer.append_data(np.asarray(obs.images["top_cam"]))
            states = object_states(emb._env, ["torch", "glass_lid", "on_lever"])
            origin = states["torch"].to_world(task.torch.meta["spotlight_offset"])
            target = np.array(task.control_point(.018, .018, .130))
            ray = target - origin
            axis = states["torch"].rotation() @ np.array([0., 0., -1.])
            aim_deg = float(np.degrees(np.arccos(np.clip(
                np.dot(axis, ray / np.linalg.norm(ray)), -1., 1.))))
            trace.append({
                "step": len(trace), "phase": label,
                "torch_pos_m": states["torch"].pos.tolist(),
                "lid_deg": float(task.STATE.get("lid_angle_deg", 0.0)),
                "lever_deg": float(task.STATE.get("lever_angle_deg", 0.0)),
                "torch_aimed": bool(task.STATE.get("torch_aimed", False)),
                "aim_deg": aim_deg,
                "torch_axis": axis.tolist(),
                "torch_quat_xyzw": states["torch"].quat_xyzw.tolist(),
                "score": float(task.score(None)),
            })

        def move(arm, pos, orientation, seconds, label):
            nonlocal ql, qr, tcp_l, tcp_r, rot_l, rot_r, max_ik_error
            q, tcp, rot = (ql, tcp_l, rot_l) if arm == "left" else (qr, tcp_r, rot_r)
            base = bases[0 if arm == "left" else 1]
            target_pos = np.asarray(pos, float)
            if arm == "right":
                # Reuse the proven contact waypoints in the cabinet frame
                # after turning its front towards the camera.
                target_pos = np.asarray(task.control_point(
                    target_pos[0] - .31,
                    target_pos[1] - (-.190),
                    target_pos[2] - task.TOP))
                orientation = axis_angle_matrix(
                    np.array([0., 0., 1.]), task.CONTROL_YAW) @ orientation
            start_pos, start_rot = tcp.copy(), rot.copy()
            n = round(seconds * 20)
            for i in range(1, n + 1):
                s = ik.smoothstep(i / n)
                p = start_pos + s * (target_pos - start_pos)
                r = ik.slerp_matrix(start_rot, orientation, s)
                q, error, _ = ik.solve(p, ik.matrix_to_quat_xyzw(r),
                                       q_init=q, base_pos=base, iters=80)
                max_ik_error = max(max_ik_error, float(error))
                if arm == "left":
                    ql = q
                else:
                    qr = q
                step(label)
            if arm == "left":
                tcp_l, rot_l = target_pos, orientation
            else:
                tcp_r, rot_r = target_pos, orientation

        def finger(target, seconds, label):
            nonlocal gr
            start = gr
            n = round(seconds * 20)
            for i in range(1, n + 1):
                gr = start + (target - start) * (i / n)
                step(label)

        def hold(n, label):
            for _ in range(n):
                step(label)

        shot("initial")
        down = grip_rotation()
        lift_30 = axis_angle_matrix(np.array([1., 0., 0.]), np.deg2rad(30.)) @ down
        lift_60 = axis_angle_matrix(np.array([1., 0., 0.]), np.deg2rad(60.)) @ down
        lift_85 = axis_angle_matrix(np.array([1., 0., 0.]), np.deg2rad(85.)) @ down
        tilt = (axis_angle_matrix(np.array([0., 0., 1.]), np.deg2rad(40.))
                @ axis_angle_matrix(np.array([1., 0., 0.]), np.deg2rad(-45.))
                @ down)
        side_aim = (axis_angle_matrix(np.array([0., 0., 1.]), np.deg2rad(80.))
                    @ axis_angle_matrix(np.array([1., 0., 0.]), np.deg2rad(-75.))
                    @ down)
        if blind:
            hold(95, "leave_torch_unaimed")
        else:
            # Begin aimed straight down, then search left to right across the
            # bench. The final cabinet aim is a separate robot action.
            start_rot = rot_l.copy()
            rows = (
                ("left_1", (.36, .16, 1.02), 0.0, .55),
                ("left_2", (.36, .09, 1.02), 0.0, .55),
                ("center", (.36, .02, 1.02), 0.0, .55),
                ("right_1", (.36, -.05, 1.02), 0.0, .55),
                ("right_2", (.36, -.12, 1.02), 0.0, .55),
            )
            for label, position, mix, seconds in rows:
                move("left", position, ik.slerp_matrix(start_rot, tilt, mix),
                     seconds, "scan_" + label)
                hold(3, "inspect_" + label)
                shot("scan_" + label)
            move("left", (.26, .020, 1.035), tilt, .9, "find_control")
            hold(10, "aim_hold")
            move("left", (.110, -.065, 1.060), side_aim, 1.4, "move_torch_clear")
            hold(15, "side_aim_hold")
        shot("aimed")
        if aim_only:
            result = {"mode": "aim_only", "steps": len(trace),
                      "score": float(task.score(None)),
                      "torch_aimed": bool(task.STATE.get("torch_aimed", False)),
                      "aim_deg": trace[-1]["aim_deg"],
                      "torch_axis": trace[-1]["torch_axis"],
                      "max_ik_error_m": max_ik_error}
            (out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
            (out / "trace.jsonl").write_text("".join(json.dumps(row) + "\n" for row in trace))
            np.savez_compressed(out / "actions.npz", actions=np.asarray(actions))
            print(json.dumps(result), flush=True)
            return
        move("right", (.31, -.272, 1.016), down, 1.2, "approach_guard")
        shot("approach_guard")
        move("right", (.31, -.272, .947), down, 1.0, "descend_guard")
        finger(0.0, .7, "grasp_guard")
        hold(8, "guard_grip_hold")
        shot("guard_gripped")
        move("right", (.31, -.240, 1.005), lift_30, .9, "lift_lid_30deg")
        move("right", (.31, -.181, 1.056), lift_60, .9, "lift_lid_60deg")
        move("right", (.31, -.078, 1.083), lift_85, 1.2, "flip_lid_up")
        move("right", (.31, -.020, 1.060), lift_85, .9, "fold_lid_past_vertical")
        hold(8, "guard_hold")
        shot("guard_opened")
        if not guard_only:
            finger(1.0, .5, "release_guard")
            move("right", (.31, -.020, 1.145), lift_85, .7, "clear_guard")
            move("right", (.46, -.145, 1.065), lift_60, 1.0, "route_right_of_torch")
            move("right", (.328, -.135, 1.010), lift_30, 1.0, "approach_lever_from_right")
            move("right", (.328, -.135, .925), down, .8, "above_lever")
            shot("above_lever")
            move("right", (.328, -.135, .901), down, .75, "descend_lever")
            finger(0.0, .6, "grip_lever")
            hold(8, "lever_grip_hold")
            shot("lever_gripped")
            move("right", (.328, -.225, .873), down, 2.0, "pull_lever_on")
            hold(12, "lever_on_hold")
            shot("lever_on")
            finger(1.0, .5, "release_lever")
            move("right", (.328, -.225, .950), down, .8, "clear_lever")
            hold(12, "settle_on")
            shot("finish")
        final = object_states(emb._env, ["torch", "glass_lid", "on_lever"])
        result = {
            "mode": "blind_press_control" if blind else ("guard_only" if guard_only else "full_contact"),
            "steps": len(trace), "score": float(task.score(None)),
            "torch_used": bool(task.STATE.get("torch_used", False)),
            "guard_opened_lit": bool(task.STATE.get("guard_opened_lit", False)),
            "valid_activation": bool(task.STATE.get("valid_activation", False)),
            "worklight_on": bool(task.STATE.get("worklight_on", False)),
            "lid_deg": float(task.STATE.get("lid_angle_deg", 0.0)),
            "lever_deg": float(task.STATE.get("lever_angle_deg", 0.0)),
            "max_ik_error_m": max_ik_error,
        }
        (out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        (out / "trace.jsonl").write_text("".join(json.dumps(row) + "\n" for row in trace))
        np.savez_compressed(out / "actions.npz", actions=np.asarray(actions))
        print(json.dumps(result), flush=True)
    finally:
        if writer is not None:
            writer.close()
        emb.close()


if __name__ == "__main__":
    out = Path(sys.argv[1])
    run(out, "--guard-only" in sys.argv[2:], "--blind" in sys.argv[2:],
        "--fast" in sys.argv[2:], "--aim-only" in sys.argv[2:])
