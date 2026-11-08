# Sort Household Waste into Lidded Bins and Close Them

Clear three items from a household snack table. The bottle and can belong in the green mixed-recycling bin; the stained napkin belongs in the black general-waste bin. Both bins start closed. Their physical rear hinges require opening, clearance during insertion, and closure after disposal. The full-score demonstration holds a lid with one arm while the other deposits waste.

This tests a complete lidded-bin cleanup sequence under an explicit sorting rule. It does not test discovering a city's waste policy or infer a universal meaning for bin colours. The score checks physical outcomes, not an inferred internal reasoning process.

## Runtime and reproduction

Validated packages: Inspect Robots 1.0.0, inspect-robots-yam 0.38.0 and Isaac Lab 17.0.2. Validated YAM revision: `0059d02a1a91bc6834c1d0846022891df0c78730`. Use the official YAM Arena / Inspect Robots Isaac runtime with `yam_arena`, `inspect_robots`, `inspect_robots_yam`, NumPy, ImageIO and its ffmpeg backend installed. Keep all Python files in this folder together. The can is resolved through Arena's `tomato_soup_can` asset registry; no external mesh is bundled. The procedural geometry is supplied in `assets.py`; `partkit.py` adds compound rigid bodies and passive hinges through task hooks without editing the YAM package.

From the repository root, run the physical full-score demonstration:

```bash
python tasks/lidded-household-waste-sort/demo_full.py runs/waste-demo
```

It sends robot joint/gripper actions; it does not teleport objects or programmatically open joints. Result JSON, per-phase camera images, a state trace and H.264 video are written under the output directory. The fixed demonstration takes 2,356 control steps, 117.8 seconds of simulated time, within the 2,400-step limit. Wall time depends on rendering hardware.

Run the official model trial:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  --policy-checkpoint dreamlabs/dream-1-beta \
  -E task=tasks/lidded-household-waste-sort/task.py -E top_cam=demo \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Sort the empty plastic drinks bottle, empty metal soup can, and food-stained paper napkin. Put the bottle and can in the green MIXED RECYCLING bin and the used napkin in the black GENERAL WASTE bin. Open each lid to put items in, then leave both lids closed. Do not put anything in the wrong bin." \
  --scorer task_score --max-steps 2400 --store-frames --seed 42 --no-prompt \
  --log-dir runs/waste-dream
```

The model receives the ordinary RGB observations and action interface, with no scripted waypoints. The scene is fixed, with zero object jitter. The task uses end scoring, not peak score. Dream-1 credentials use the official client configuration and are not included.

## Score and controls

- Each of three correctly contained, released and settled items: **0.25**.
- Each robot-opened bin with at least one correctly settled item: **0.10**.
- All three items settled and both lids closed to at most 20 degrees: **0.05**.
- Any wrong-bin entry: **permanent 0**, even after recovery.

Opening an empty bin gives no credit. All correct items with lids open gives 0.95. Sorting and bin order are unrestricted. `task.json` states every numerical tolerance; `task.py` implements them.

```bash
python tasks/lidded-household-waste-sort/test_score_contract.py
python tasks/lidded-household-waste-sort/preview.py runs/waste-idle
python tasks/lidded-household-waste-sort/demo_full.py runs/waste-wrong --wrong-bin-control
```

The wrong-bin control physically deposits the recyclable bottle into general waste using robot actions. CPU tests cover score milestones and the permanent wrong-bin latch; they supplement, rather than replace, physical trials.

## Scope

The bins and waste are representative tabletop surrogates. Paper does not deform, the can is not cut open, PET does not crush, and waste contamination is a scoring convention rather than a material simulation. The passive hinges and contact-dependent grasping are simulated. No real-hardware success rate or force calibration is claimed.

## Measured validation

The full physical demo scored **1.0 in 2,356 steps**. The current-scene 20-step idle control scored **0.0**. The 723-step wrong-bin control physically placed the bottle into the opened black bin and latched `wrong_bin=True`, with score **0.0**. All four CPU contract tests passed.

The single official Dream-1 beta trial scored **0.0**, ending at **2,400 steps (`max_steps`)** with no runtime error. Wall time was **555 seconds** for the 120-second simulated episode on the evaluation system. This is a valid unsuccessful policy rollout, distinct from the scripted feasibility demonstration.
