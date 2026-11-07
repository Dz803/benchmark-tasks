"""Capture initial RGB views of the scene without taking any robot actions.

Usage: python preview.py OUTPUT_DIRECTORY
The script calls reset once and writes each provided camera image; it does
not call embodiment.step, run an episode, or test a policy or scorer.
"""

from __future__ import annotations

import argparse
from pathlib import Path


def capture(output: Path, width: int = 640, height: int = 352):
    import imageio.v2 as imageio
    import numpy as np
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    output.mkdir(parents=True, exist_ok=False)
    task_path = str(Path(__file__).with_name("task.py").resolve())
    embodiment = yam_arena_embodiment(
        task=task_path, object_jitter=0, control_hz=20,
        cam_width=width, cam_height=height, top_cam="demo",
    )
    observation = embodiment.reset(Scene(id="lantern_scene_preview", instruction="Scene preview"))
    for name, frame in observation.images.items():
        imageio.imwrite(output / f"{name}.png", np.asarray(frame))
    (output / "README.txt").write_text(
        "Initial camera views only. No robot action, rollout, score, or physical validation.\n"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=352)
    args = parser.parse_args()
    capture(args.output, args.width, args.height)
