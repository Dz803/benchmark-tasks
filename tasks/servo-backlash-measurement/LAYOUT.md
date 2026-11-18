# Fixture geometry and real measurement route

## Implemented fixed-layout geometry (2026-11-17)

The table surface is world z = 0.7475 m. The assembled shaft is at world
(0.345, 0, 0.8575) m and the unloaded lever tip is at approximately
(0.345, 0, 0.9575) m. The original source STL remains a visual mesh; the
simple boxes below are the colliders. This is an adaptation of the cited
source CAD, not a dimensional copy of a commercial spring gauge.

| Simulated item | As-built collision/kinematic specification |
| --- | --- |
| Base | 450 × 300 × 16 mm box, yaw 90°, fixed at (0.345, 0, table z); its long direction is world Y. Four 4.5 mm radius bolt-head visuals and four witness marks are visual only. |
| Holder/servo | 44 × 70 mm holder footprint with 6 mm bottom and 22 mm upright rails; 42 × 40 × 50 mm servo proxy. Both fixed at the base origin. |
| Rotor/output | 12 × 16 × 12 mm rotor at local z 110 mm, 9 × 14 × 90 mm beam centered at z 165 mm, 25 × 14 × 12 mm tip at z 210 mm. The rotor hinge has ±12° travel, 25 drive stiffness, 0.15 damping and 0.35 N·m limit; the lever hinge has ±0.7° travel, zero drive stiffness and 0.01 damping. The held target is 0°. These are solver settings, not manufacturer constants. |
| Gauge bodies | One on each side at reset world (0.345, ±0.250, 0.9605) m; 140 × 20 × 18 mm free dynamic box with 80 g mass and 0.7 friction. Ordinary open-top low-lip cradle: 160 × 26 × 8 mm horizontal support at local z 200 mm, four 22 × 3 × 3 mm end lips; no axial drive/snap. Each gauge body must be grasped to move. |
| Links | One common lever-tip attachment at local z 210 mm and two body-eye offsets at ±73 mm local X. Tension only: F = 100 N/m × max(0, length − 180 mm), applied in equal/opposite pairs at moving endpoints, with COM-relative torque. 230 mm is the modeled extension-failure limit, not a break sensor. |
| Recorder | Fixed enclosure centered at world (0.100, 0, table z), yaw 270°, 320 × 170 × 48 mm, with a 296 × 142 mm upward screen 0.5 mm above its face. It is outside the normal body-grasp corridor. The screen now has one immutable printed scale texture and three separately movable USD pointers, driven by the same measured forces and output encoder count as the logger; this is visual feedback, not the score source. |

Reset uses the unchanged fixed assembly, free gauge bodies in their open
cradles, 0° held input target and the stock YAM READY arm pose. A valid
record begins only after two seconds with both link forces ≤0.10 N. The
output encoder is the physically integrated lever orientation rounded to
4096 counts/revolution. The simple unilateral link is applied once per
control step and held by PhysX over its internal substeps; it is a
reduced-order elastic element, not a modeled rope mesh or calibrated PASCO
spring. The gauge is neither teleported nor made kinematic to the hand.
The eyelet is a small collider attached to the gauge body and is guarded as
a prohibited direct-push surface. The simulated force pointer is visual
only; there is no separately moving plunger or spring mechanism inside the
body. Thus the runtime can test direct contact with the exposed eyelet, but
does not claim to validate a real plunger's independent force path.

### Proposed real readout chain (not hardware-validated)

The two analog gauges do **not** provide an electronic force stream. Mount a
fixed auxiliary camera above both 100 mm scale faces, illuminate them without
glare, and record the ordinary indicator/scale at ≥200 pixels per 100 mm.
Calibrate each gauge separately at 0, 0.5, 1.0, 1.5 and 2.0 N using traceable
test loads; repeat in increasing and decreasing directions to measure
hysteresis. Fit a pixel-position to force curve and check held-out points
near 0.10, 1.2, 1.8 and 2.0 N. At 50 mm/N ideal span, a 1 mm tracking error
is 0.02 N; lens, parallax, friction and scale error must also be measured.
Reject or recalibrate if the combined error exceeds 0.05 N. Read the DUT's
own **output-shaft** encoder counts via UART, as described in section 1 of
the [reference paper](https://pmc.ncbi.nlm.nih.gov/articles/PMC13087586/)
for its tested servo; qualify the chosen UART model as output-shaft sensing
and check that the horn/lever does not slip on that shaft. An internal
motor-side encoder on a different model would not measure this output angle
without an added output sensor. Do not infer counts from the screen bitmap. Timestamp
camera frames and UART packets on one host clock; log
`timestamp_s, left_indicator_px, right_indicator_px, left_force_n,
right_force_n, encoder_count, calibration_id, frame_id` at ≥10 Hz. Check
time alignment with a visible marker and one known pull event before using
2-second windows. This describes a build and verification procedure only:
no physical gauges, camera or logger have been assembled or calibrated here.

## Fixed-scale recorder as built, 2026-11-18

The 296 × 142 mm face within the existing 320 × 170 mm desk enclosure is
static. Two horizontal 0–2 N rulers show left/right tension, with a green
1.2–1.8 N **task qualification band**; this is not an equipment safety limit.
A third ruler shows output angle from −5° to +5°; this is a display range,
not a servo manufacturer's specification. Each force pointer is a separate
bright red USD cube, and the angle pointer is cyan. Their x positions come
from `runtime.py`'s actual tension-only link forces and 4096-count output
shaft encoder conversion. No texture update, scored-value substitution,
frame overlay, or physics change is involved. The original 10 Hz raw force
and encoder log remains the measurement record; the desk face is a visible
aid for the robot. The proposed real build would show the two physical
analog spring-gauge pointers and the output-shaft UART encoder reading from
the same synchronized acquisition chain. That hardware has not been built
or calibrated here.


See README.md for source CAD, parts and calibration limitations.
