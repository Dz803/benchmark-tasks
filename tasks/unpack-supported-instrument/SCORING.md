# Numeric scoring and support history

The task scores the final state at 20 Hz. History is cleared on reset. Tilt above 25 degrees or unsupported downward travel above 30 mm permanently forces zero. There is no prescribed arm assignment or hazardous removal order; a safe lift to the stand is valid.

The instrument's four lower foot corners are transformed into the cradle and stand coordinates. Support requires every corner inside the support deck (3 mm horizontal tolerance), the lowest corner 37–43 mm above the support origin and the highest no more than 52 mm, on an upright deck. The nominal deck is 40 mm above its origin. The cradle deck half-size is 90 x 65 mm; the stand deck is 55 x 45 mm. These geometric/contact proxies are not force measurements.

A grasp is treated as supporting only while a gripper is below 38% of its open stroke, its TCP is within 18 mm of the carrying-handle center, and the handle's position relative to the gripper changes by less than 6 mm between control steps. During a period without deck or grasp support, the scorer tracks the highest instrument origin, including the last controlled pose. Downward travel above 30 mm latches failure. The fall is checked before clearing the interval on landing, so touching the stand does not erase a preceding drop. Sampling quantizes the onset of a detected slip; this is documented rather than called continuous force validation.

For 0.5, stand support must hold, the instrument origin must be within 25 mm of stand center, its tilt below 10 degrees, its translational speed below 15 mm/s and angular speed below 0.10 rad/s. Both grippers must be at least 65 mm from the instrument, cap and cradle handles for one second.

For 1.0, all the partial-score conditions and packing conditions must hold together for one second. Every box corner and conservative cylinder-bounding-box corner of both foam pieces, including their texture geometry, must be inside the tray interior: x within 134 mm and y within 154 mm of tray center, allowing 1 mm numerical tolerance. The lowest point must be at least 5 mm above the table (tray floor top is 8 mm); the highest must be below 300 mm. Both parts must move below 20 mm/s and 0.12 rad/s. Stacking is allowed if all these physical placement conditions hold.

Synthetic history tests are in test_score.py. They do not replace the physical idle, failed-action control or complete successful episode.

The upper-foam grip/release reference is its recessed side rim at local (-87,0,0) mm; the lower-foam reference is its recessed shoulder at (-87,-70,137.5) mm. The packing height envelope is 300 mm above the table.
