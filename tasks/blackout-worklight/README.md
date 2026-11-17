# Find and switch on battery-backed workbench lighting during a blackout

A power cut leaves the workbench dark. A separate battery bank supplies an emergency worklight, but the robot must find its protected ON handle using a flashlight already held in one gripper. Its first illuminated patch misses the access case. A large yellow `BACKUP POWER` label on the clear lid becomes visible only when the flashlight reaches it. The robot sweeps the work area from left to right with overlapping beam positions, flips the clear lid up, and pulls the top bar of a folding U-shaped handle down to switch on the battery-backed worklight. This does not restore mains power. The route is not part of the score. The fixed-layout scripted demonstration shows a search motion, but does not establish that a learned policy can search visually.

## Scene and real-trial setup

The official two-arm YAM station provides a 1.1 × 1.3 m workbench. A roughly 180 mm, 204 g rigid flashlight begins lit and contact-held by one gripper near station-frame (0.468, 0.214, 0.912 m). A fixed 180 × 330 mm low access case is centred at (0.35, −0.190 m), with its front toward the overhead/front view. Its clear rigid 147 × 150 × 6 mm polycarbonate-look lid has two yellow `BACKUP POWER` lettering rows, pivots up and back on a rear hinge, and has a 140° hard stop and a 102 mm grip bar at the front. Inside, a separate rigid folding U-shaped handle turns on a revolute joint from 0° OFF to a 60° ON stop; an ON pawl engages after 50° and prevents reverse travel below 49°. The room and ambient lights begin at zero; after the handle reaches 48°, the overhead backup emitter lights the bench. `object_jitter=0` and seed 42 specify the tested layout. The [setup image](setup.png) is the actual dark reset image.

[REAL_TRIAL.md](REAL_TRIAL.md) lists dimensions, parts, wiring, estimated fixture cost and reset steps for a proposed low-voltage bench reproduction. [build_plan.svg](build_plan.svg) gives top and side dimensions. This fixture has **not** been built or calibrated. The simulated case is assembled from original procedural rigid primitives; no third-party mesh or texture is redistributed. The folding bail shape and 64 mm hinge spacing reference the [Southco P9 grab handle](https://media.southco.com/media/static/Literature/p9.en.pdf), while the longer arms and low-voltage switch linkage are custom and described in the build guide. The flashlight size and form reference the [Streamlight 4AA ProPolymer Lux](https://www.streamlight.com/products/detail/4aa-propolymer-lux-div-1); the listed electrical parts are linked in the build guide.

## Score

`task_score` returns 0, 0.2, 0.5 or 1.0. The flashlight lens must be 8–65 cm from the handle, its optical axis within 28° of the handle centre, and the ray to that point unobstructed by opaque case walls for 10 consecutive 20 Hz steps to earn 0.2. Measured lid opening starts at 0.5°, above the recorded reset jitter (<0.06°); from this first motion until first activation the beam must stay aimed **on every step**. A lid angle of at least 115° under that continuous illumination earns 0.5. The first handle pull starts at 1.5° from OFF, above the measured passive motion (<1.0°). A one-way ON catch engages after 50° and prevents reverse travel below 49°; it does not move the handle forward or award score. An early pull, or loss of aim at any time during opening and first activation, irreversibly sets the score to 0. The physical switch can still light the backup lamp on an invalid pull, but that does not earn credit. Full score requires a valid first pull to at least 48°, the backup light on for 10 steps, and both grippers at least 5 cm from the handle while its angle remains within 1° over 10 consecutive steps. No exact arm assignment is scored. Beam geometry is a reproducible proxy, not a lux measurement.

The separate synthetic scorer regression replays the measured contact trace with darkness at first measurable lid motion, darkness during opening, an early pull, or darkness at first pull. These all score 0; the unmodified physical success trace scores 1.0. These synthetic counterfactuals are not separate physical rollouts. Measurements use the joint states and flashlight pose, not commanded robot targets or hidden object repositioning.

## Reproduction

After activating an environment with `yam_arena` and `inspect-robots`, run from the repository root:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  --policy-checkpoint dreamlabs/dream-1-beta -P timeout_s=90 \
  -E task=tasks/blackout-worklight/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 -E top_cam=demo \
  --instruction "The room lights are out. A lit flashlight is already held in one gripper, pointing straight down; the emergency-light cabinet is not visible at the start. Sweep the work area from left to right with overlapping beam positions to find the glass access case. Keep the case illuminated while the other gripper flips the clear access lid upward, then reach inside, grasp the top bar of the folding U-shaped emergency-light handle, and pull it down to ON. Release it and leave the backup light on." \
  --scorer task_score --max-steps 1200 --store-frames --seed 42 --no-prompt \
  --grader none --no-rerun \
  --log-dir /tmp/blackout-worklight-model
python tasks/blackout-worklight/demo_contact.py /tmp/blackout-worklight-demo
```

The run limit is 1,200 control steps, or 60 seconds of simulated time at 20 Hz. The fixed-layout scripted contact demo scored 1.0 in 498 steps (24.9 simulated seconds), with the lid at about 140° and handle at 60° after release. Its light first aimed at the control after 109 steps (5.45 s). Its trajectory is open-loop and does not establish robustness to rearranged layouts. A separate Dream-1 beta run on this exact revision scored 0.2 at the 1,200-step limit: the model illuminated the case but did not complete the lid-and-handle operation. The scripted demonstration and Dream-1 score are reported separately in `task.json`.

The flashlight beam follows its measured body pose. Its central 34° cone and weaker 70° spill visually represent one source, but brightness is renderer-tuned rather than measured in lux. Cabinet friction, the over-centre lid hold, handle catch and lamp wiring are approximations, so simulation success does not prove a real robot can operate a certified emergency-light installation.
