"""Replay an exact saved YAM action sequence in the task's physical scene."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import imageio.v2 as imageio
import numpy as np


def run(actions_file: Path, output: Path):
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    output.mkdir(parents=True, exist_ok=False)
    emb = yam_arena_embodiment(
        task=str(Path(__file__).with_name("task.py").resolve()),
        object_jitter=0, control_hz=20, cam_width=320, cam_height=176,
        top_cam="demo")
    obs = emb.reset(Scene(id="chipped_plate_replay", instruction=emb.task.module.INSTRUCTION))
    from yam_arena.scene import object_states
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, READY

    right_robot = emb._env.unwrapped.scene["right_robot"]
    right_arm_ids = [right_robot.joint_names.index(n) for n in ARM_JOINTS]
    right_finger_ids = [right_robot.joint_names.index(n) for n in FINGER_JOINTS]

    actions = np.load(actions_file)["actions"]
    frames, trace = [], []
    for i, action in enumerate(actions, 1):
        obs = emb.step(Action(data=action)).observation
        if i % 3 == 0:
            frames.append(np.asarray(obs.images["top_cam"]))
        if i % 5 == 0:
            bodies = object_states(emb._env, ["plate_a", "plate_b"])
            task = emb.task.module
            face_normal = task._face_normal(bodies["plate_b"])
            camera_ray = task._CAMERA_POS - bodies["plate_b"].pos
            camera_dot = float(face_normal @ camera_ray / np.linalg.norm(camera_ray))
            measured_joints = right_robot.data.joint_pos.torch[0].detach().cpu().numpy()
            right_tcp, _ = ik.tcp_pose(measured_joints[right_arm_ids],
                                       station.BASES["right"])
            trace.append({"step": i, "score": float(task.score(None)),
                          "inspected": task.STATE["inspected"].copy(),
                          "inspect_ticks": task.STATE["inspect_ticks"].copy(),
                          "plate_pos": {n: b.pos.tolist() for n, b in bodies.items()},
                          "plate_b_face_normal": face_normal.tolist(),
                          "plate_b_camera_dot": camera_dot,
                          "right_tcp_actual": right_tcp.tolist(),
                          "right_finger_gap_m": float(-sum(measured_joints[right_finger_ids]))})
    for camera, frame in obs.images.items():
        imageio.imwrite(output / f"final_{camera}.png", np.asarray(frame))
    task = emb.task.module
    result = {"source_actions": str(actions_file), "steps": len(actions),
              "score": float(task.score(None)), "success": bool(task.success(None)),
              "inspected": task.STATE["inspected"].copy(),
              "failures": {k: task.STATE[k] for k in
                           ("wrong_service", "wrong_discard", "dropped")}}
    result["final_ready_max_joint_error_rad"] = {}
    for side in ("left", "right"):
        robot = emb._env.unwrapped.scene[f"{side}_robot"]
        ids = [robot.joint_names.index(n) for n in ARM_JOINTS]
        measured = robot.data.joint_pos.torch[0, ids].detach().cpu().numpy()
        result["final_ready_max_joint_error_rad"][side] = float(
            np.max(np.abs(measured - np.asarray(READY))))
    (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    (output / "trace.jsonl").write_text("".join(json.dumps(x) + "\n" for x in trace))
    imageio.mimsave(output / "replay.mp4", frames, fps=20 / 3,
                    codec="libx264", output_params=["-pix_fmt", "yuv420p"])
    print(json.dumps(result))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("actions", type=Path)
    ap.add_argument("output", type=Path)
    args = ap.parse_args()
    run(args.actions, args.output)
