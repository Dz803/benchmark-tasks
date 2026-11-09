# Recover a caught sample carrier with coordinated two-arm motion

Move a laboratory sample carrier to its receiving stand while preserving the upright samples. An orange retaining lip overlaps one carrier ear. Both arms must coordinate the shared load: clearing the visible lip before lifting is valid, and a caught carrier can be lowered and slid free. This setup uses capped dummy samples, with no liquid or biological material.

The source and receiver are 165 mm apart. The carrier is 120×240 mm, with two passive rotating grip bars 280 mm apart. Four independent rigid sample tubes each have a 30 mm diameter, 72 mm body and a cap. The source lip and both stands are fixed; the carrier, bars and tubes use gravity and contact. The revolute bars approximate folding handles, without claiming every real carrier universally requires two hands.

## Run

Use the standard YAM/Isaac task environment with NumPy and imageio installed. From this task directory:

```sh
python demo.py runs/demo --mode demo
python demo.py runs/idle --mode idle
python demo.py runs/unsafe --mode unsafe
```

Output directories must not already exist. Each run saves `setup.png`, `final.png`, `demo.mp4`, `trace.jsonl` and `result.json`. Keep stdout/stderr as the raw run log. `--mode probe` performs only the initial grasp, horizontal clearance and lift. The task uses a fixed layout and no jitter in these scripts; limit 2,400 control steps at 20 Hz. Each run creates a fresh scene. Startup adds simulation initialization time.

`demo.py` sends only robot joint and gripper commands. It reads the visible carrier pose to correct placement drift, so it is a scripted oracle controller rather than a learned policy. It does not move objects directly or create robot/object attachments. The camera pose is set in the task reset and shared across controllers.

## Score

A historical hard failure occurs if carrier tilt exceeds 12°, or any of 16 circumference samples around a tube bottom exits the carrier-frame bounds x ±60 mm, y ±120 mm, z −12 to 45 mm. The interval covers base contact tolerance and clearing the carrier rim. Every tube must be upright within 10° for nonzero credit.

- 0.5: the carrier's entire rim clears the source stand's rear edge, carrier tilt is at most 5°, and all tubes remain contained and upright.
- 1.0: carrier translation is within 10 mm of the receiving pose and full orientation error is at most 5°; both grippers are over 75% open and their TCPs are over 30 mm from the bars; carrier linear speed is below 15 mm/s and angular speed below 0.15 rad/s continuously for one second.

A historical failure overrides all later states. Stopping alone, a prescribed action order and grasp counts earn no credit. Geometry and thresholds are design values. The model contains no fluid, deformable material or motor fault simulation; the obstruction is physical contact with a fixed lip.

## Validation

The scripted physical demo scored 1.0 in 600 control steps (30 seconds), with peak carrier tilt 4.502°, final target-position error 3.67 mm, and released stability of 2.85 seconds. The no-action control scored 0 with a stable initial state. The continued one-side-lift control physically tilted the carrier to 19.526° and toppled samples, triggering historical zero. These are single fixed-layout trials, not a robustness estimate. One official Dream-1 beta trial completed all 2,400 control steps with score 0.0 and no errored trials. The video preserves the complete model run.

Official model evaluation, from this task directory with the YAM runtime active and Dream credentials configured:

```sh
inspect-robots run --embodiment yam_arena --policy dream --policy-checkpoint dreamlabs/dream-1-beta -E task=task.py -E top_cam=demo -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "Move the sample carrier onto the receiving stand, keeping the samples upright and contained. If it catches, stop and free it without forcing or tipping it, then complete the transfer." --scorer task_score --max-steps 2400 --store-frames --seed 42 --no-prompt --log-dir runs/dream
```

Model horizon: 120 simulated seconds; measured evaluation wall time: about 8.6 minutes, excluding environment startup. Tested with Isaac Lab Arena 0.3.1 and Inspect Robots 1.0.0.
