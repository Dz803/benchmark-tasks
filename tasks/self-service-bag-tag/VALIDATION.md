# Validation — self-adhesive tag revision (rev3)

## Response to the previous review (TK-NF88FL)

> Provide a purchasable product for the tape, and modify the tape model such that the backside
> is self-adhesive. See "thermal baggage tag rolls".

1. **Purchasable product.** The tag is modelled on linerless self-adhesive airline bag tags:
   Eezeetags (Security Label GmbH) for the adhesive behaviour, and the retail POS Supply
   AIR-SLFTAG21RL-SS thermal self-tag roll (2 x 21 in, 51 x 533 mm, 175 tags per roll) for the
   size. Paper weight and thickness come from the Sihl PRESSURETAC BT 28/3 datasheet
   (177 g/m², 170 µm). README "Real procedure and equipment" gives the links and states which
   property comes from which source.
2. **Self-adhesive back.** The whole back of every panel now carries the adhesive (drawn as a
   faint beige layer); the printed front never does. Following the reference product, the
   adhesive sticks only to itself, so it bonds back to back and never to the case, the handle
   or the grippers. Any two non-neighbouring panels can bond if their backs are pressed
   together; the score counts only a seal that closes the tag round the handle with the barcode
   outside. The previous revision's coloured end patches are gone.

## Changes since the previous revision

| Area | Previous | Now | Why |
| --- | --- | --- | --- |
| Tag length / mass | 430 mm in places; 20 g | 533 mm, 4.8 g | reference roll size and Sihl grammage |
| Discretisation | 10 panels x 53.3 mm | 14 panels x 38.1 mm | 53 mm rigid panels could not drape over the 30 mm handle |
| Bending stiffness | Clark 150, EI 1.7e-4 N·m² | Clark 60, EI 6.8e-5 N·m² | lower end of the patent's preferred range; k = EI / L per hinge |
| Air drag | none | linear damping 2.0 1/s | flat-plate drag; a hanging strip otherwise swings undamped |
| Solver | 32 position iterations | 64 position, 8 velocity | keep light hinged panels joined under gripper contact |
| Shell friction | 0.15 | 0.4 | the lower strand slid away under a press at 0.15 |
| Adhesive faces | blue patch on panel 09 front to panel 00 back | whole back, back to back only | review request and reference product |
| Bond gate | ≥ 1 N for 0.4 s on the patches | tack ≥ 0.25 N; seal ≥ 0.5 N; both 0.4 s | the fingertip press next to the handle cannot hold 1 N (runs 069/070); see README "Adhesive" |
| Closure | end pinched in one gripper | right strand laid over the left strand, then pressed twice | the self-only back must meet another back |

## Current full demonstration

`runs/rev3-final-074/result.json`: one continuous run from reset with the code defaults (no
environment overrides) and `demo_commands`; score **1.0**, 3,208 control steps (160.4
simulated seconds) at 20 Hz.

* The tag is pulled from the presenter, the leading tip is fed under the handle by a
  closed-loop feed that keeps it on the shell, and the right hand pinches and pulls it through.
* The left hand lays its strand on the shell beside the handle and returns to ready. The right
  hand turns the end over, carries it over the handle onto the left strand and lets go.
* The right fingertips press the two layers twice at the same spot: the first press tacks
  them, the second seals them. The seal forms at step 2340 between panels 6 and 12, back to
  back, at 1.16 N. No bond of any kind forms earlier in the run.
* Both hands end open and clear; the sealed loop stays round the handle with the barcode
  (panel 7) exposed and is still for 3.95 s at the end of the hold.
* Largest hinge separation 2.5 mm. Handle audit: one step with 0.017 mm penetration.
* It reproduces `runs/rev3-replay-068` exactly (largest panel-position difference 0.0 m).

## Controls

* **No press:** the same commands stopped after the right hand lets go of the laid end
  (`runs/rev3-ctrl-nopress`). Result: score 0.0 through step 1,614 with no
  bond of any kind; the run was then stopped to free the GPU for the Dream run.
* **Below threshold:** runs 069, 070 and snapshot V used a 1 N seal threshold. Their presses
  reached 1.1–2.1 N peaks but never held 1 N for 0.4 s, and no seal formed (score 0). This is
  why the seal threshold is 0.5 N (see README "Adhesive").
* **Drop:** the previous revision's drop-after-extraction control (`BAGTAG_NEGATIVE`) is kept
  in the probe; it was not rerun for this revision.

## Dream-1-beta

One episode on this revision with a 1,000-step cap (`--max-steps 1000`): task_score 0.0,
ended by the step cap. The arms stayed near their start poses and never took the tag from the
presenter.

## Regression tests

- `test_score_outcomes.py`: 192 synthetic combinations of the score tiers, including release
  boundaries 0, 0.249, 0.25, 1.99 and 2 s.
- `test_score_geometry.py`: barcode exposure on a recorded 14-panel closed state, occlusion by
  another paper face, rotating and oscillating corners, hold resets and seal-break latching.
- `test_hold_window.py`: hold-window resets.
- `test_press_gate.py`: no force, weak force, off-panel contact, back on front, tangential
  contact, interrupted dwell and non-finite values; a 0.24 N contact never tacks and a 0.3 N
  contact held 0.4 s does.

## Known limits

* Adhesive thresholds, break limits and friction values are simulation approximations; the
  reference products publish no application force or bond strength.
* Paper friction (1.25) is higher than real paper-on-paper friction so that the grippers can
  pull the tag from the presenter.
* Hinged rigid panels approximate a continuous strip. Hinge separations up to about 2.5 mm
  occur under gripper contact.
* The release audit's 5 mm limit is not met when a hand lets go of a hanging strand: the
  strand then falls or swings under gravity (74–223 mm at the four releases in the 1.0 run).
  The audit records finger–panel contacts during each release so that following the fingers
  can be told apart from falling.
* Robot targets are recomputed from simulator poses (closed loop); the demo restores no object
  state. Preflight IK tolerance on Cartesian segments is 8°; arm-to-arm clearance on joint
  routes is checked at 2 mm.
* Earlier evidence (runs 001–067, 10- and 20-panel models, older adhesive rule) is superseded
  and not counted here.
