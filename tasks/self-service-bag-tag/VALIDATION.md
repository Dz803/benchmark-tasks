# Validation

## Continuous full demonstration
The final hardened scorer achieved 1.0 in one continuous initial-reset rollout:
2,452 control steps at 20 Hz (122.6 simulated seconds). The seal remained linked,
the barcode face was exposed, and both hands were open and clear for 5.75 seconds.
Both arms also returned to their ready poses; home position is not scored.
No saved object poses were replayed: the controller replayed robot targets.
The source hashes are recorded with the rollout.

## Failure controls
- Actual printer extraction followed by gripper release dropped the label onto
the table and scored zero. This physical test predates the additional exposure
and stability checks; the no-extraction-credit rule is unchanged.
- A separate staged two-coupon physical diagnostic formed a bond, removed its
support and lowered the diagnostic break force to 0.001 N. Gravity broke the
joint. The actual PhysX break event was received and the failure flag latched.
This is not a robot demonstration or a calibrated adhesive strength test.
- 128 CPU outcome cases and eight targeted CPU regressions and five runtime hold-window tests passed, including
paper-on-barcode occlusion, rotational motion, returning oscillation, continuous
validity timing and break-event handling. Synthetic controls are not physical
contact evidence.

## Limits
The label is a hinged rigid-panel approximation with a breakable fixed joint
for adhesion. Barcode exposure uses sampled geometric rays, not decoding.
Linkage uses winding/aperture geometry within the fixed handle scene. These
checks address known failure cases; they do not prove absence of every exploit
or real-world hardware reliability. Dream-1-beta completed one 4,800-step episode (240 simulated seconds),
score 0.0, termination max_steps, no terminal error. The label was not
attached to the handle. One recoverable protocol-response retry was logged;
no scene reset or scripted fallback occurred.

Final full demonstration: full-demo-019, after replacing sub-millimetre speed wording with a 5 mm corner-displacement hold. Same physical scene and robot route.

Model video contains all 4,801 initial/action observation frames from all three cameras at 20 fps. Both videos passed complete decoding and are under 50 MB. Task, scoring and part-builder hashes match between model and final demo.
