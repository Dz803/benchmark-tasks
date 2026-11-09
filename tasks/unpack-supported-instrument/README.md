# Unpack a supported optical instrument without dropping it

This narrow-foot optical module requires its matching service base for operation and travels in a shaped supporting cradle. Its loose protective cap does not carry its weight. Transfer the instrument safely onto the stand and put both foam pieces into the packing tray. The task permits direct transfer to the stand before removing the empty cradle.

## Runtime and files

Use an installed Isaac Lab / Arena / YAM / Inspect Robots environment with NumPy and imageio with an H.264 ffmpeg backend. The stock YAM 14-D joint/gripper action interface is unchanged. `partkit.py` constructs compound rigid bodies and registers scoring/reset callbacks; all loose task bodies have real gravity and contacts. The shared task-reset camera is used by both the scripted demo and model evaluations.

Required files: `task.py`, `partkit.py`, `score_logic.py`, `physical_demo.py`, `task.json`, `SCORING.md`, and this README. `test_score.py` provides additional CPU checks. No concept-animation scripts or precomputed object trajectories are required.

From this task directory:

```sh
python test_score.py
python physical_demo.py runs/demo --mode demo
python physical_demo.py runs/idle --mode idle
python physical_demo.py runs/drop --mode drop
```

Each output directory must be new. Use the official `yam_arena` embodiment with `task=task.py`, `top_cam=demo`, `object_jitter=0`, `control_hz=20`, `cam_width=320`, and `cam_height=176` for model evaluation. On Linux aarch64 runtimes that require it, preload the system libgomp before launching Isaac.

The run limit is 2,400 control steps at 20 Hz. The fixed layout uses no object jitter. A fresh reset restores objects, arms and scorer history. The scripted demo sends only robot actions and reads object poses for waypoint alignment; it never writes object poses or creates grasp attachments.

The `result.json`, `trace.jsonl`, phase images and `physical.mp4` in each run contain the measured outcome. Score semantics and tolerances are documented in `SCORING.md`; scoring tests are synthetic boundary checks, not physical evidence. The action-only physical demo scored 1.0 and terminated successfully after 1,665 steps (83.25 simulation seconds), within the 2,400-step limit. Wall time on the tested runtime was approximately six minutes. The 150-step idle control scored 0; its complete evidence was written before a simulator teardown segmentation fault (process exit 139). The full demo process exited normally. Dream-1 has not yet been evaluated.

Official Dream evaluation, from this directory with credentials configured:

```sh
inspect-robots run --embodiment yam_arena --policy dream --policy-checkpoint dreamlabs/dream-1-beta -E task=task.py -E top_cam=demo -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "Unpack the instrument onto its service stand and put the shipping foam in the packing tray, keeping the instrument supported throughout." --scorer task_score --max-steps 2400 --store-frames --seed 42 --no-prompt --log-dir runs/dream
```

Tested with Isaac Lab Arena 0.3.1 and Inspect Robots 1.0.0. A model trial allows 120 simulated seconds; wall time depends on renderer and model latency.
