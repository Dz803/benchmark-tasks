# Measure a robot joint's lost motion with controlled opposing pulls

A robot-development bench holds a geared actuator, a 100 mm output lever and
two opposed compliant loading connections. The robot grasps ordinary gauge
bodies, establishes and releases a controlled test load in both directions,
and leaves the apparatus unloaded. Force and output-angle records characterize
load reversal; no particular measured angle is considered the correct answer.
This is an acquisition task, not a task to repair or reduce the joint's play.

## Recorded results

- Continuous scripted YAM demonstration: **1.0**, 536 control steps, 26.8 seconds.
- Dream-1 beta through Inspect Robots: **0.0**, 2,400 control steps, 120 simulated
  seconds, 983 seconds wall time, termination `max_steps`, zero errored trials.
  The official client returned 50 action chunks across 93 request attempts,
  recovering expired requests and one protocol error; no trial was restarted.
- The scene, physics, instruction and scorer are the same in both recordings.
  The demo captures 640 x 352 observations; Dream receives the official 320 x 176
  observations. The model video enlarges the top view and stacks both wrist
  views. Demo captions show recorded diagnostics and are not policy inputs.
- `evidence/demo_telemetry.json` and `demo_audit.json` retain the full successful
  acquisition record. An earlier real lever-push control earned 0.5, then
  cleared to zero; `lever_push_audit.json` records that outcome. That control
  predates the expanded eyelet/distal-link guard. No exhaustive contact test
  or physical calibration is claimed.

## Reproduce

Use the official Isaac Lab/Arena YAM and Inspect Robots environment with
`inspect-robots-yam` and `inspect-robots-dream` installed. Additional Python
packages used here are NumPy, trimesh, Pillow, OpenCV, imageio and imageio-ffmpeg.
Tested source revisions: YAM `0059d02a1a91bc6834c1d0846022891df0c78730`,
Arena `c7edf3a9d5d8276565208f72422f8f8a0806d6cf`, Dream adapter
`b5e936a201cec66c54c297e522c920d98445e339`, Inspect Robots 1.0.0.
Run one simulator process per GPU. From this task directory:

```bash
python full_demo.py runs/demo --actions demo_actions.json
python verify_rollout.py evidence/demo_telemetry.json
python run_model.py
```

The replay sends the saved joint/gripper actions to the real simulated robot;
objects are moved by contact and forces, without pose edits or attachment to
the gripper. The demo must use an absent output directory. `run_model.py` runs
one Dream-1 evaluation using the instruction in task.json, `object_jitter=0`,
20 Hz, `top_cam=demo`, 320 x 176 images, seed 42, `task_score`, and 2,400 steps.
It requires an existing Dream API credential and consumes API credit. It
refuses to overwrite its log directory. Simulation setup and inference latency
vary; the recorded model trial took about 16.4 minutes wall time.

The runtime applies equal/opposite tension forces at moving body attachment
points, including moment arms. The gauge bodies are free dynamic bodies; there
is no kinematic hand-following. The integrated output-lever angle is quantized
to 4096 counts/revolution. `scorer.py` consumes 10 Hz force/encoder records and
integrity flags; its complete contract is in task.json. `verify_rollout.py`
recomputes the score and verifies the recorded load/unload windows offline.

## Physical reconstruction and limits

`LAYOUT.md` gives the actual geometry, force law, joint parameters, reset and
proposed real acquisition chain. Use the supplied source holder/lever CAD,
a fixed base plate, a UART servo with an output-shaft encoder, two 0-2 N spring
scales (PASCO ME-9510 or a checked equivalent), open supports and soft loading
links. The gauge envelope is a proxy, not a manufacturer drawing. Adjust the
supports to the actual gauge body while preserving free axial motion and the
100 mm lever arm. Both scales must start unloaded and remain visible.

Indicative ordinary-part allowances, excluding the robot/computer: servo and
horn $18-30; prints $10-25; two scales $55-100; plate/fasteners $20-35; supports,
links and hooks $10-20; UART adapter/supply $15-25. The subtotal is $128-235;
prices are estimates, not vendor quotations. A reusable camera/light and
reference loads are additionally needed for the proposed optical readout.
A gauge camera resolves at least 200 pixels per 100 mm scale; calibrate five
loads and check held-out thresholds to within 0.05 N, including hysteresis.
Synchronize these readings with output-shaft UART counts at 10 Hz. Video and
a ruler check the alignment, contact and fixture conditions. The recorder
pointers are a simulated display of these channels, not three real sensors.
This acquisition chain has not been physically built or validated.

The reduced-order compliance is uncalibrated: it supports effective lost-motion
measurement, not certified gear-backlash accuracy. The spring-gauge internals
and flexible cords are not individually simulated. The two-second bands and
2 N overload rule define this acquisition protocol, not a manufacturer's
accuracy or damage specification. A real trial must qualify its scales,
load path and encoder before applying the rubric.

## Sources and licenses

The method and two CAD assets come from Boris Kotov / Robonine,
[An open-source test stand for backlash measurement in low-cost UART servo motors](https://pmc.ncbi.nlm.nih.gov/articles/PMC13087586/)
and its [source repository](https://github.com/roboninecom/Measuring-Backlash-in-Popular-UART-Servos).
`source_assets/servo_holder_single.stl` and
`source_assets/test_lever_100mm_single.stl` are **CC BY 4.0**; they are scaled
from millimetres to metres and transformed into this adapted fixture.
Simplified collision bodies are independently defined in task.py. Exact source
URLs and blob identifiers are in `source_assets/manifest.json`.
The reference GPL-3.0 software was not copied. The remaining proxy geometry and
recorder-face image are generated for this task. Official YAM/Arena robot assets
are external runtime dependencies and are not redistributed in this folder.
