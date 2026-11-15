"""Render an unchanged robot-action rollout at presentation resolution."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import imageio.v2 as imageio
import numpy as np


def run(out: Path, actions_file: Path, width: int):
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    out.mkdir(parents=True, exist_ok=False)
    height = width * 9 // 16
    emb = yam_arena_embodiment(task=str(Path(__file__).with_name("task.py")),
                               object_jitter=0, control_hz=20,
                               cam_width=width, cam_height=height,
                               top_cam="demo")
    obs = emb.reset(Scene(id="locker_demo_high_resolution_replay",
                          instruction=emb.task.module.INSTRUCTION))
    actions = np.load(actions_file)["actions"]
    writer = imageio.get_writer(out / "demo_top.mp4", fps=5,
                                codec="libx264", output_params=[
                                    "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                                    "-crf", "18"])
    frame_count = 0
    try:
        imageio.imwrite(out / "initial_top.png", np.asarray(obs.images["top_cam"]))
        for i, a in enumerate(actions, 1):
            obs = emb.step(Action(data=a)).observation
            if i % 4 == 0:
                writer.append_data(np.asarray(obs.images["top_cam"]))
                frame_count += 1
            if i in (420, 750) or i == len(actions):
                for camera, frame in obs.images.items():
                    imageio.imwrite(out / f"step_{i}_{camera}.png", np.asarray(frame))
        st = emb.task.module.STATE
        result = {"source_actions_sha256": hashlib.sha256(actions_file.read_bytes()).hexdigest(),
                  "steps": len(actions), "frame_count": frame_count,
                  "resolution": [width, height], "score": float(st["score"]),
                  "success": bool(emb.task.module.success(None)),
                  "invalid": bool(st["invalid"]),
                  "door_deg": float(st["door_deg"]),
                  "bolt_m": float(st["bolt_m"])}
        (out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result))
    finally:
        writer.close()
        emb.close()


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("out", type=Path)
    p.add_argument("--actions", type=Path, required=True)
    p.add_argument("--width", type=int, default=960)
    a = p.parse_args()
    run(a.out, a.actions, a.width)
