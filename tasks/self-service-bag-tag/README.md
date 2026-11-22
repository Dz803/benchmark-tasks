# Loop a printed baggage tag through a suitcase handle and seal it

At a self-service bag-drop counter, take the waiting printed tag from the outlet,
thread the flexible strip under the suitcase's closed side handle, lay one strand
over the other with the adhesive backs together and press them to seal the loop.
Release the label and leave the barcode exposed. This evaluates flexible-strip
routing, receiving a partly hidden end, regrasping and back-to-back adhesive closure. No handle lift, hand
assignment or exact arm resting pose is required for scoring.

## Real procedure and equipment

The tag is a linerless, self-adhesive airline bag tag of the kind used at self-service
bag drops: direct thermal printable paper whose back carries an adhesive that **only sticks
to itself**, so there is no backing paper to peel off. The reference product is
[Eezeetags](https://eezeetags.com/products-services), made and sold by
[Security Label GmbH](https://www.security-label.de/en-gb/solutions/self-adhesive-linerless-tags-eezeetags)
(Eezeetags family; ordered by enquiry to the manufacturer; used for self-tagging by airlines
such as easyJet, KLM and Lufthansa). The manufacturer describes the construction as direct
thermal printable paper, a silicone release layer and a special adhesive that only sticks to
itself, designed to IATA Recommended Practice 1740, and states that the adhesive does not stick
to bags, clothes or floors.

**Because the adhesive only sticks to itself, the tag does not stick to the suitcase, its
handle or the robot's grippers.** That is why this task bonds only tag back to tag back. The
passenger loops the printed tag round the handle and presses the two adhesive backs together,
so the printed faces stay outside. A back pressed onto a printed front does not bond.

The manufacturer does not publish dimensions or paper weight. The outline therefore borrows a
retail-listed self-tag roll of the standard airline format,
[POS Supply 2" x 21" Thermal Self-Tag Baggage Tags](https://www.possupply.com/2-21-thermal-self-tag-baggage-tags-freezer-grade-adhesive-10-rolls)
(item AIR-SLFTAG21RL-SS, 51 x 533 mm per tag, 175 tags per roll). Only its size is used; its
listing does not describe a self-only adhesive. The barcode is a visible pattern; no scanner or
passenger check-in is evaluated.

Use a small cabin suitcase, a bench/scale, a printed tag from that roll and a
passive presenter with a single outlet. An operational airport kiosk is not
required: printing is complete before reset. A low-cost presenter can be made
from ordinary sheet material using `PRESENTER.md`; its retention holds the tag
until pulled, without supporting the hanging portion. The mechanism being
scored is the strip/handle/seal, using full-size strip width and ordinary handle
clearance. The decorative terminal does not perform any hidden manipulation.

## Setup

The official `yam_arena` station supplies two YAM arms with parallel grippers,
one overhead camera and two wrist cameras. Use the fixed task layout and seed 42,
with object jitter zero. Coordinates, materials and all collision shapes are
specified in `task.py`. Coordinates are metres in the station world frame.

* A hard-shell cabin-case visual, approximately 450 mm long, 178 mm deep and
  334 mm tall while standing on its narrow side, is centred at (0.330, 0) on
  the counter. Its long-side handle faces upward. The shell is supported/fixed
  on the scale; lifting or moving the suitcase is outside this task.
* A closed padded strap-style handle spans approximately 170 mm between mounts.
  The central aperture is about 30 mm above the shell. A passive 40 mm slider
  approximates handle lift, but the demonstrated route uses its resting gap.
* A 460 by 145 mm presenter/terminal (outlet face at (0.497, -0.265)) extends
  back from the outlet, with outlet plane
  x=0.500 and tag centre 240 mm above the table. The internal channel is hollow,
  21 mm high and 450 mm long. Approximately 90 mm of the 533 by 50.8 mm tag
  protrudes initially.
* One thermal self-tag, 533 by 50.8 mm, represented by fourteen hinged 38.1 by
  50.8 mm panels, 4.8 g in total. The barcode is printed on the front (the face up
  as the tag leaves the printer); the whole back carries the self-only adhesive,
  drawn in a faint beige tint. The entire tag exists at reset; pulling never spawns
  new paper.

Replace the used label, reload its trailing end and reset both arms between
trials. No hot, sharp, powered or contaminating equipment is used.

## Primary scoring

Score the current outcome, not the highest previous milestone. Use the highest
applicable tier, not their sum. All nonzero tiers require an intact, fully
extracted strip whose designated ends are bonded into a closed loop around the
actual handle, with an outward-facing barcode clear of the suitcase shell.
The printed barcode must face away from the loop interior and remain exposed
after release; sealing the tag with its barcode hidden inside is not a successful outcome.
This is an orientation/exposure requirement, not a barcode-scanner accuracy test.

* **0:** any prerequisite absent, including extraction alone, loose threading,
  a label merely on the case, an unlinked loop, or a broken label/connection.
* **0.75:** the prerequisites hold, but the robot has not yet released it stably.
* **0.85:** both grippers are open and clear of the label, and the loop remains within 5 mm for at least 0.25 seconds.
* **1.0:** the released stable state persists for at least 2 seconds.

In a real trial, inspect the loop and bonded overlap from video/photos, with a
ruler or an area grid if needed. At least 600 mm² of the tag's adhesive back must be
stuck onto another part of the same tag's back, so that the bonded tag closes round the
handle; a loose crossing or a back pressed on the printed front is not a seal. Check
that the printed front is exposed,
both grippers are open and at least 75 mm from the strip, and that it remains
attached without any corner drifting more than 5 mm from its hold-start position for
two seconds. Returning to a particular joint pose is not scored.

The simulation uses the physical strip's geometry to test handle linkage,
adhesive state and integrity. It additionally checks TCP clearance of 120 mm and
finger/palm-origin clearance of 75 mm. All outcome conditions, including barcode
exposure and complete extraction, must hold throughout the two-second interval.
A PhysX seal-break event permanently invalidates the trial; current overlap, face
gap and alignment are also checked. Rotations and intermediate oscillations enter
the stability check through panel-corner displacement from the hold-start pose. The barcode-side test is a geometric
visibility proxy with nine rays across the printed front against the other paper
panels. Only the printed front counts; an upward exterior viewing direction
is required. This is not barcode decoding or an exhaustive occlusion proof. See `tag_closure.py` for all thresholds.

## Fidelity

**Strip.** The tag is fourteen colliding rigid panels joined by elastic hinges (a
discretised beam), not a continuous paper/cloth solver. Its outline matches the reference
roll (533 x 50.8 mm). Mass is 4.8 g, from 177 g/m² bag-tag stock
([Sihl PRESSURETAC BT 28/3](https://sihl.com/en/products/pressure-tac-bt-283/) datasheet,
170 µm); the reference product publishes no weight. Bag-tag laminates have a Clark stiffness
(JIS P 8143) of 50–200, preferably 60–150
([WO2003027950A1](https://patents.google.com/patent/WO2003027950A1/en)). We use 60, the lower
end of the preferred range; neither reference product publishes its own stiffness. Clark
stiffness S = L³/100 (L in cm) gives a 18.2 cm critical length, and the self-weight buckling
relation EI = w·g·L³/7.837 gives EI ≈ 6.8e-5 N·m² (our conversion, not a figure from the
patent). Each hinge has k = EI / L_panel = 1.8e-3 N·m/rad, so EI does not depend on the panel
count. Earlier drafts used ten 53.3 mm panels and Clark 150 (EI ≈ 1.7e-4 N·m²); the coarser
panels could not drape over the 30 mm handle.

**Air drag and numerics.** PhysX has no aerodynamics, so a hanging strip would swing
undamped. Each panel has a linear damping of 2.0 1/s: flat-plate drag ½·ρ·Cd·A·v² with
ρ = 1.204 kg/m³, Cd = 1.17 (Hoerner, *Fluid-Dynamic Drag*, 1965) and the paper's area per
mass, linearised at 0.5 m/s. It does not depend on the panel count. The strip then settles
within a few swings, like real paper. The tag bodies use 64 position and 8 velocity solver
iterations so that the light hinged panels stay joined under gripper contact (largest hinge
separation 2.5 mm in the demo); this is a numerical setting, not a material change.

**Collision and friction.** Collision thickness is 1 mm (the real paper is 0.17 mm) so that
PhysX resolves the finger contacts. Paper friction is 1.25: this is simulation tuning so that
the grippers can pull the tag from the presenter, and it is higher than real paper-on-paper
friction (about 0.3–0.5). The suitcase shell's friction is 0.4, a plausible value for paper on a
hard plastic shell but not a measured one; at the earlier 0.15 the lower strand slid away
under a press. The
presenter's 2 N release is representative tuning.

**Adhesive.** The whole back of every panel carries the self-only adhesive. A bond forms only
back to back, between two non-neighbouring panels of the same tag: opposed faces (within
15°), each lying on the other's back side, at least 600 mm² overlap at any in-plane angle,
and surface contact within the 1 mm collision-sheet margin. Following the manufacturer's
description, the back never bonds to a printed front, the case, the handle or a gripper.
Bonding is pressure-gated in two levels, both needing the contact to be held for 0.4 s with
the two panels still relative to each other:

* **Tack:** at least 0.25 N on both backs. It makes a weak joint (2 N / 0.05 N·m) that keeps
  the layers together, like a light first touch of a cohesive coating. A tack does not count
  as a seal.
* **Seal:** at least 0.5 N on both backs. It makes the full joint (8 N / 0.15 N·m). Only a
  seal counts towards the score. The previous revision used 1 N with a gripper squeeze; here
  the two layers lie on the shell next to the handle, the 68 mm-wide jaws cannot reach under
  the lower layer, and the fingertip press peaks at about 1–2 N but does not hold 1 N for
  0.4 s. 0.5 N is therefore a lower, disclosed threshold, not a measured property.

Force comes from GPU-native PhysX contact impulses divided by the physics timestep. Because
only the last physics substep of each control step is sampled and paper-on-paper impulses
arrive intermittently, the gate uses the peak of the last three samples (0.15 s), and a dip
of up to 20 % below the threshold pauses the 0.4 s dwell instead of resetting it.
Alignment or proximity without physical pressure cannot create either level. Once sealed,
the fixed joint holds the panels' relative pose; integrity checks allow 20 % jitter on the
formation overlap so that numerical flicker at exactly 600 mm² is not counted as a break.
All thresholds and break limits are explicit simulation approximations: the reference
products publish no application force, dwell, peel or shear values (see
`ADHESION_REALISM.md`). Peel propagation and adhesive chemistry are not modelled. The handle
is a spring-loaded rigid proxy for a strap, and the suitcase, scale and presenter are fixed
supports.

## Reproduction

Install the official Inspect Robots `yam_arena` embodiment and Dream plugin,
Isaac dependencies, NumPy, SciPy, trimesh and imageio/imageio-ffmpeg in the same
Python environment. The task runs through the official embodiment. Tested package
versions are listed in `runtime_versions.json`.

From this task folder, in that activated environment:

```bash
BAGTAG_RENDER_STRIDE=4 BAGTAG_VIDEO_WRIST=1 BAGTAG_REPLAY_COMMANDS=demo_commands BAGTAG_REPLAY_LIMIT=31 \
  python direct_thread_transport_probe.py runs/demo
python finalize_demo.py runs/demo
python compose_closeup_video.py runs/demo   # top view + wrist close-ups
python test_score_outcomes.py
python test_score_geometry.py
python test_hold_window.py
python test_press_gate.py
BAGTAG_RENDER_STRIDE=4 BAGTAG_NEGATIVE=drop_after_extraction \
  python direct_thread_transport_probe.py runs/drop-control
```

The demo is one continuous reset, using simulator poses to choose robot targets.
It does not restore object state. The rev3 demonstration (`runs/rev3-final-074`, code
defaults only) took 3,208 steps (160.4 simulated seconds) at 20 Hz and scored 1.0, with the
seal formed at 1.16 N and 3.95 s of released stability at the end. Actual wall time depends on
rendering (about 20 minutes on the test machine). See `VALIDATION.md` for evidence and limits.
Rendering every fourth step affects recording only; Dream uses its normal camera observation
cadence. Dream-1-beta scored 0.0 on this revision in a 1,000-step
episode: the arms stayed near their start poses and never took the tag from the presenter.

Dream (one episode, 1,000 control steps maximum):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  --policy-checkpoint dreamlabs/dream-1-beta -P timeout_s=90 \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 -E top_cam=demo \
  --instruction "Attach the printed baggage tag securely to the suitcase's long-side handle, with the barcode facing out. Its back is self-adhesive and sticks only to itself: loop it through the handle, lay one strand over the other with the backs together and press them, then release the tag and withdraw both arms." \
  --scorer task_score --max-steps 1000 --store-frames --seed 42 \
  --no-prompt --grader none --no-rerun --log-dir runs/dream
```

## Third-party assets and licenses

The suitcase visual is [Innerscene Rolling suitcase](https://www.innerscene.com/tools/library/3d-parts/rolling-suitcase-ee9d7578),
released under CC0 1.0. It was resized and converted to USD, and the original
recessed handle was replaced by the task-authored strap visual. Asset details,
source hash and changes are in `assets/SOURCE.md`. This is an adapted generic
case, not an exact replica of a commercial product. The remaining task geometry,
markings and presenter are procedurally authored. Official YAM/Arena robot assets
are installed runtime dependencies, not redistributed here. The cited product
manual is a reference, not an included asset.
