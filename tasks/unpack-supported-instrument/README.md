# Unpack a supported optical instrument without dropping it

A narrow-foot optical module travels in fitted upper and lower protective foam inside a telescopic corrugated carton. The deep outer cover starts removed beside the carton base. Remove the upper foam, transfer the instrument onto its service stand while maintaining support, and collect both foam inserts in the packing tray. Direct supported transfer is valid; there is no required hand assignment or hazardous removal order.

The scene represents the protected-content stage of unpacking, not cutting tape or opening a sealed parcel. Upper foam has recessed grip wells instead of an added lid handle. All carton and foam parts are rigid proxies: foam compression, cardboard flex and optics damage are not simulated. Original compound geometry approximates fitted instrument shipping protection; no downloaded product asset is required.

## Run

Use an installed Isaac Lab / Arena / YAM / Inspect Robots environment with NumPy and imageio with an H.264 ffmpeg backend. The stock 14-D joint/gripper action interface is unchanged. From this task directory:

```sh
python test_score.py
python physical_demo.py runs/demo --mode demo
python physical_demo.py runs/idle --mode idle
python negative_control.py runs/drop --reference runs/demo
```

Each output directory must be new. Keep stdout/stderr as the raw log. Each physical run writes result.json, trace.jsonl, phase images and physical.mp4. The fixed layout has no object jitter. A fresh reset restores objects, arms and score history. On Linux aarch64 runtimes that require it, preload the system libgomp before launching Isaac.

The run limit is 2,400 control steps at 20 Hz (120 simulation seconds). The successful scripted demo uses 1,500 steps (75 simulation seconds). Wall time depends on hardware and renderer initialization. The demo reads object poses for waypoint alignment and sends only robot actions; it never writes object poses or creates grasp attachments. The task-reset camera is shared with model evaluation.

## Score and evidence

Any tilt over 25 degrees or unsupported descent over 30 mm permanently forces zero. Otherwise 0.5 requires stable, released placement on the service stand; 1.0 additionally requires both complete foam geometries inside the packing tray and below the 300 mm height envelope, stable for one second. Full numeric tolerances and support proxies are in SCORING.md. Synthetic boundary tests are not physical trials.

The complete physical demo reached 1.0 in 1,500 steps and terminated successfully. Peak instrument tilt was 4.458 degrees; the final released arrangement was stable for 3.35 seconds. These are fixed-layout results, not a real-hardware success-rate estimate.

## Official model evaluation

From this task directory, using the same scene and scoring:

```sh
inspect-robots run --embodiment yam_arena --policy dream --policy-checkpoint dreamlabs/dream-1-beta -E task=task.py -E top_cam=demo -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "The outer carton cover has already been removed. Unpack the instrument onto its service stand and put both protective foam inserts in the packing tray, keeping the instrument supported throughout." --scorer task_score --max-steps 2400 --store-frames --seed 42 --no-prompt --log-dir runs/dream
```

Same-version physical controls: idle scored 0.0 over 150 steps. The lift-and-release control replayed the successful demo prefix, verified a 137.16 mm held lift, then opened the gripper. It reached 49.64 degrees of tilt and 52.14 mm of unsupported descent and latched zero. The original direct `--mode drop` path cannot access the covered handle and is not evidence of a drop; use `negative_control.py` after generating the successful demo trace.

Dream-1-beta completed one valid trial: score 0.0 after 2,400 steps, max-step termination, zero errored trials. Wall duration was 549 seconds. The complete model video and successful physical demo accompany this task.
