# Recover a caught sample carrier with coordinated two-arm motion

Move a laboratory sample carrier to its receiving stand while preserving the upright samples. An orange retaining lip overlaps one carrier ear. Both arms must coordinate the shared load: clearing the visible lip before lifting is valid, and a caught carrier can be lowered and slid free. This setup uses capped dummy samples, with no liquid or biological material.

The source and receiver are 195 mm apart. The carrier is 120×240 mm, with two passive rotating grip bars 380 mm apart. Four independent rigid sample tubes each have a 30 mm diameter, 72 mm body and a cap. The source uses two 165×18×18 mm longitudinal polymer sliding strips with uncalibrated friction coefficient 0.10. The receiver uses transverse steel supports. The orange lip has a nominal 4 mm ear gap. The source lip and both stands are fixed; the carrier, bars and tubes use gravity and contact. The revolute bars approximate folding handles, without claiming every real carrier universally requires two hands.

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

- 0.5: the carrier's entire rim clears the source support strips' forward edge at x=335 mm, carrier tilt is at most 5°, and all tubes remain contained and upright.
- 1.0: carrier translation is within 10 mm of the receiving pose and full orientation error is at most 5°; both grippers are over 75% open and their TCPs are over 30 mm from the bars; carrier linear speed is below 15 mm/s and angular speed below 0.15 rad/s continuously for one second.

A historical failure overrides all later states. Stopping alone, a prescribed action order and grasp counts earn no credit. Geometry and thresholds are design values. The model contains no fluid, deformable material or motor fault simulation; the obstruction is physical contact with a fixed lip.

## Validation

The revised scripted physical demo scored 1.0 in 700 control steps (35 seconds). Actual pre-lift horizontal travel was 72.63 mm, with a conservative lip gap of 43.41 mm. Peak carrier tilt was 0.682° and peak sample tilt was 0.683°. Final target-position error was 0.833 mm, with 2.95 seconds of released stability. This is one fixed-layout trial, not a robustness estimate. Contact-event reporting was unavailable; positive conservative geometry gaps support separation at recorded control frames, not a blanket absence-of-contact claim.

The same-version no-action control scored 0.0 in 45 steps. A single-sided unsafe lift reached 12.070 degrees of carrier tilt in 186 steps, latched historical failure and scored 0.0. That control intentionally stops at the first safety violation; its script emits a RuntimeError after saving evidence.

## Official model evaluation

From this task directory, using the same scene and scoring:

```sh
inspect-robots run --embodiment yam_arena --policy dream --policy-checkpoint dreamlabs/dream-1-beta -E task=task.py -E top_cam=demo -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "Move the sample carrier onto the receiving stand, keeping the samples upright and contained. If it catches, stop and free it without forcing or tipping it, then complete the transfer." --scorer task_score --max-steps 2400 --store-frames --seed 42 --no-prompt --log-dir runs/dream
```

Dream-1-beta completed one valid trial: score 0.0 after 2,400 steps, max-step termination, zero errored trials. Wall duration was 524 seconds. The complete action episode is recorded.
