"""Capture initial 320×176 views and 20-step idle hinge stability."""

from pathlib import Path
import argparse
import json

import imageio.v2 as imageio
import numpy as np


def run(output: Path):
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    output.mkdir(parents=True, exist_ok=False)
    emb = yam_arena_embodiment(task=str(Path(__file__).with_name("task.py").resolve()),
                               object_jitter=0, control_hz=20,
                               cam_width=320, cam_height=176, top_cam="demo")
    obs = emb.reset(Scene(id="lidded_bins_preview", instruction=emb.task.module.INSTRUCTION))
    for name, frame in obs.images.items():
        imageio.imwrite(output / f"initial_{name}.png", np.asarray(frame))
    for _ in range(20):
        obs = emb.step(Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))).observation
    for name, frame in obs.images.items():
        imageio.imwrite(output / f"idle_{name}.png", np.asarray(frame))
    state = emb.task.module.STATE
    from yam_arena.scene import object_states
    objects = object_states(emb._env, ["pet_bottle", "soup_can", "used_napkin"])
    task = emb.task.module
    result = {"score": float(emb.task.module.score(None)),
              "green_angle_deg": state["green_angle_deg"],
              "black_angle_deg": state["black_angle_deg"],
              "items": {name: {"pos": item.pos.tolist(),
                                "low": task._bounds(name, item)[0].tolist(),
                                "high": task._bounds(name, item)[1].tolist()}
                        for name, item in objects.items()},
              "no_action_steps": 20}
    (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    run(parser.parse_args().output)
