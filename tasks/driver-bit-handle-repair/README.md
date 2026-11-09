# Fit the Right Bit and Secure a Loose Drawer Handle

Choose the bit by its working-tip shape, assemble the empty electric screwdriver, steady the removed drawer and seat the loose handle. Return the stopped tool to its stand and withdraw both hands. Category: Impact on jobs.

## Runtime

Use the official Isaac Lab Arena / YAM environment with Inspect Robots and the Dream policy installed. This directory includes its local geometry helper and all task/demo dependencies. No external asset downloads are needed: the drawer, interchangeable bits and driver are procedural rigid-body prototypes; robot and station assets come from YAM.

From this directory, run the complete scripted demonstration:

```bash
python demo.py runs/demo
```

Run one official model evaluation:

```bash
inspect-robots run --embodiment yam_arena --policy dream --policy-checkpoint dreamlabs/dream-1-beta \
 -E task=task.py -E top_cam=demo -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
 --instruction "Inspect the loose drawer-handle screw. Fit a suitable bit into the electric screwdriver, steady the drawer, tighten the handle, and leave the stopped tool released in its stand." \
 --scorer task_score --max-steps 4800 --store-frames --seed 42 --no-prompt --log-dir runs/model
```

The model receives the official three-camera images and robot state. The scripted demonstration uses known scene state and inverse kinematics and is not a learned-policy result. Its overview camera is framed for presentation at 640 x 352; model input remains 320 x 176. Control runs at 20 Hz with 240 Hz physics; maximum simulated duration is four minutes. Wall time depends on hardware. Reset restores the fixed initial layout; no object jitter is applied.

## Score and mechanism

Score is binary and evaluated at the end. The handle gap must be at most 0.2 mm; the motor must be off; the tool root must be less than 25 mm from its rest position and its axis less than 15 degrees from the rest axis; both gripper TCPs must be more than 60 mm from the tool grasp centre. These conditions must hold continuously for one second. During that window, tool displacement from its first pose must be at most 2 mm, orientation change at most 1 degree, and unwrapped spindle angle range at most 0.02 rad. All other outcomes score zero. There is no score for selecting a bit, following scripted phases or merely approaching a screw.

Grip and axial-pressure switches physically move before the torque-limited motor runs. Contact between the socket and retained hex shank transmits rotation; tip/recess contact drives the screw. An ideal 0.7 mm/revolution PhysX joint coupling links screw rotation to a guided handle carriage. The second visible fastener is a guide rather than an independent tightening target. Tightening torque, helical surface contact, wear and hardware success rates are not calibrated or established.

`task.py` is the public task entry. `powered_task.py` applies the final physical mechanism and score over geometry layers. Intermediate layer names/comments reflect construction stages; only the final imported scene is evaluated. `demo.py` rejects diagnostic checkpoint starts, so the complete demonstration always begins at reset. Checkpoint files emitted during the run are diagnostic records, not substitute full-demo evidence.

## Evidence

Final-layout full demo: 1.0 in 3797/4800 steps (189.85 simulated seconds). Dream-1 beta: 0.0 after the full 4800-step limit, seed 42; one completed trial, zero errored trials. Wall runtime was about 23.3 minutes. The official client automatically renewed expired API sessions (100 returned action chunks, 200 attempts); the simulation episode remained continuous. The previous-layout wrong-installed-hex-tip diagnostic: 0.0. The latter starts from a diagnostic preinstalled fixture and is not a full rack-to-install trial.
