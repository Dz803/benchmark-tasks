# Inspect Washed Plates and Reject Chipped Crockery

A restaurant kitchen should keep damaged crockery out of service. The robot receives two washed, visually similar plates on a countertop drainer, inspects their faces and rims, and routes the intact plate to the serving hatch while discarding the chipped plate. The defect is a missing piece of rim geometry, not a colour label. The handling belongs to routine restaurant work: [dishwashers](https://www.onetonline.org/link/details/35-9021.00) place clean dishes in storage or serving areas and sort refuse, while [dining-room attendants](https://www.onetonline.org/link/details/35-9011.00) replenish clean dishes, carry them to serving counters and remove broken ones. The chipped-dish check is a food-safety decision within that workflow. Wigan Council's [food-safety guidance](https://www.wigan.gov.uk/Docs/PDF/Business/Environmental-Health/Food-safety-pack-for-home-bakers.pdf) recommends discarding chipped or cracked dishes because damage can retain dirt and harmful bacteria.

The instruction is: **“Inspect the face and rim of each washed plate for chips or cracks before sorting. Place each intact plate in the tray at the serving hatch, and discard each damaged plate in the reject bin.”** It states the kitchen outcome without assigning a gripper or prescribing a camera pose.

## Scene and reset

Use the standard `yam_arena` station, two YAM arms and its overhead and wrist cameras. Coordinates below are metres in the station frame, with `x` away from the robot and `y` across the table; the station table top is at `z=0.7475` m. All task-specific solids are procedural rigid bodies, not third-party meshes or textures.

| Object | Initial pose and physical description |
| --- | --- |
| Two ceramic-coloured service plates | Round, 180 mm diameter, 285 g each, with continuous rolled rims and no handles. Centres at `(0.140, +0.125)` and `(0.140, -0.125)`, 55 mm above the table at reset, tilted 15 degrees. One rim has a missing arc and fracture-coloured ends. Each plate settles onto its rack rails under gravity. |
| Plate drainer | Fixed steel-coloured 226 × 506 mm wire-frame rack centred at `(0.140, 0)`, with two separate bays, shallow inclined support rails and upright tines. Plate-centre spacing is 250 mm; the rims do not touch at rest. |
| Serving tray and hatch | Fixed stainless-coloured 300 × 300 mm receiving tray centred at `(0.410, 0)` and a roughly 300 mm wide, 224 mm high framed `PICKUP` pass opening at `x=0.580`, also steel-coloured. The tray is the destination for the intact plate. |
| Reject bin | Fixed dark-polymer bin centred at `(0.410, -0.320)`, 260 mm square inside and 135 mm high, with a red open lid, `WASTE` marking and a pedal cue. The bin is the destination for the damaged plate. |

`PLATE_SEED=0` places the defect on plate B, as in the submitted screenshot and demonstration. `PLATE_SEED=1` moves the actual missing-rim geometry to plate A and updates the correct sorting outcome; this can be used to probe fixed-slot guessing. Set `object_jitter=0` for the documented layout. Reset both arms to the standard ready pose, place both plates at the positions above, and leave the tray and bin empty. Neither the bin lid nor the hatch moves in this task.

## Score

The scorer measures the sorting outcome, not a staged inspection gesture. It records an inspection pose for diagnostics only; a camera pose alone cannot prove that a policy recognised the chip.

| Score | Measured condition |
| --- | --- |
| 0.0 | Neither plate is correctly sorted, or a safety failure has occurred. |
| 0.5 | Exactly one plate is correctly sorted and released, with the placement stable for 20 consecutive 20 Hz steps. |
| 1.0 | Both plates are correctly sorted, released and stable for the same interval. |

For the intact plate, “correct” means the complete oriented disk fits inside the serving tray, its food-contact face points upward (`normal_z >= 0.94`), and its bottom is 6–35 mm above table height. For the damaged plate, the complete disk must be inside the bin and below its 135 mm rim, allowing 5 mm contact tolerance within the 6 mm wall. A placement is stable when its centre moves no more than 8 mm and its face normal changes no more than 0.12 rad over one second. A gripper within 145 mm of a plate must be at least 70% open for that plate to count as released. Releasing the damaged plate on the service tray, discarding the intact plate, or dropping either plate more than 30 mm below the worktop permanently sets the episode score to zero, including after a later correction. Sorting order and arm choice do not affect the score. The demo returns both arms to ready, but arm parking earns no extra credit.

The scorer knows which plate's mesh is chipped at reset. It does not infer damage from the policy's camera stream. Therefore a 1.0 scripted run establishes physical feasibility and scoring consistency for one fixed layout, not autonomous visual recognition or real-world food-safety performance. Evaluate both plate seeds to distinguish visual inspection from memorising the fixed slot.

## Reproduce

Run from this task directory with the official YAM / Isaac Lab Arena environment and Inspect Robots installed. Control is 20 Hz; the documented run limit is 1,800 control steps (90 simulated seconds). The submitted action file is a fixed-layout, open-loop demonstration: it sends only robot actions through the simulation, with no object teleportation or score edits. It is specific to `PLATE_SEED=0` and `object_jitter=0`.

```bash
PLATE_SEED=0 python replay.py demo_actions.npz runs/demo

PLATE_SEED=0 inspect-robots run --embodiment yam_arena \
  --policy dream --policy-checkpoint dreamlabs/dream-1-beta \
  -E task=task.py -E top_cam=demo -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Inspect the face and rim of each washed plate for chips or cracks before sorting. Place each intact plate in the tray at the serving hatch, and discard each damaged plate in the reject bin." \
  --scorer task_score --max-steps 1800 --store-frames --seed 42 --no-prompt \
  --log-dir runs/model
```

The validated physical demonstration scored **1.0** in **1,320** control steps (66 simulated seconds), including both arms returning to ready while the placements remained stable. `demo_result.json` records the scorer output. Dream-1 beta scored **0.0** in one official 1,800-step trial with no trial errors; `model_result.json` records its result. As a negative control, replaying the same actions with `PLATE_SEED=1` (the chip moved to the other plate) scored **0.0**, with `wrong_service=true`: the unchanged placement sent the damaged plate to the serving tray. `negative_swapped_seed_result.json` records that check. Neither video claims robustness to randomised object positions.

## Fidelity and sources

The plates are rigid compound shapes; the chip is represented by omitted rim segments. The rack, tray, bin and hatch are simplified rigid fixtures. Plate friction is increased to retain a feasible rim pinch in this simulator and has not been calibrated against glazed ceramic. Breakage, sharp fragments, microbial contamination, cleaning processes, food contact and real-world grasp forces are not simulated. The open reject-bin lid is a fixed pose, so closing or operating it is outside this task.

The scene distributes no downloaded asset. The YAM station and robot are supplied by the simulation runtime. [Isaac Lab-Arena is Apache-2.0 licensed](https://github.com/isaac-sim/IsaacLab-Arena/blob/main/LICENSE.md); Isaac Sim components have [separate NVIDIA terms](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/license.html). The kitchen safety rationale is drawn from the Wigan Council document linked above; no text or graphics from it are included in the scene.
