# Adhesive realism

## Reference products

* **Adhesive behaviour:** Eezeetags linerless self-adhesive bag tags (Security Label GmbH). The
  manufacturer describes direct thermal printable paper whose back carries an adhesive that only
  sticks to itself, with no liner to peel, designed to IATA Recommended Practice 1740. It does
  not stick to bags, clothes or floors.
* **Size:** POS Supply AIR-SLFTAG21RL-SS thermal self-tag roll, 2 x 21 in (51 x 533 mm) per tag,
  sold retail. Only its size is used; its listing does not describe a self-only adhesive.
* **Stock:** Sihl PRESSURETAC BT 28/3 datasheet, 177 ± 8 g/m², 170 ± 12 µm. The manufacturer
  states that the linerless coating bonds coating to coating: form a loop, press the ends
  together, smooth out.

None of these sources gives an application force, dwell time, peel strength or shear strength.
Paper tensile strength is not adhesive bond strength.

## Simulation model (approximation)

| Quantity | Value | Basis |
| --- | --- | --- |
| Bonding faces | back to back only, any two non-neighbouring panels | manufacturer: self-only adhesive |
| Overlap | ≥ 600 mm², any in-plane angle | task design |
| Tack | ≥ 0.25 N on both backs for 0.4 s → 2 N / 0.05 N·m joint; not a seal | simulation approximation |
| Seal | ≥ 0.5 N on both backs for 0.4 s → 8 N / 0.15 N·m joint | simulation approximation; the previous revision used 1 N |
| Force sampling | peak of the last 3 control samples; a dip to ≥ 80 % pauses the dwell | numerical: one sampled substep per control step |
| Stillness | the two mating panels' relative motion < 0.04 m/s | sliding would smear the adhesive |

Contact force in newtons is not pressure in pascals, and geometric overlap is not measured real
contact area. A breakable fixed joint created at the current relative pose does not reproduce
pressure-dependent bond growth or peel. A real finger press on a bag tag is probably well above
1 N; the 0.5 N seal threshold is lower than that because the robot's fingertip press next to the
handle holds only about 0.5–1 N (it peaks at 1–2 N). The thresholds are approximations, not
calibrated values.

## What the gate guarantees

* Proximity or alignment without contact force never bonds (tested in `test_press_gate.py`).
* A back on a printed front, the case, the handle or a gripper never bonds.
* Only the seal level counts towards the score; a tack alone scores like no bond.
* In the continuous demo no bond forms before the deliberate press at the end.
