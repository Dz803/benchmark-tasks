# Check polarity and close an emergency lantern before switching it on

A four-D-cell work light is open. Its two outer cells are installed; two loose cells must be placed in the inner wells according to the visible, opposing `+`/`−` marks. The operator then seats the battery cover and turns on the lamp. One reversed cell is a hard failure. This tests the everyday safety habit of reading polarity before acting, then closing the compartment before relying on the device.

The cover is a separate 90 g rigid body, not a visual state change. Its continuous inner rim descends between the carrier ribs and housing lips; the outer runners bear on the lips. The demo moves it above the lantern, aligns its measured pose and presses it vertically into that channel. Four 139 g D-cell bodies have 34.2 mm diameter and 61.5 mm length, following the [Energizer E95 reference](https://data.energizer.com/pdfs/e95.pdf). The housing and cover are original matte-polymer analogues; the [Energizer four-D-cell area-light sheet](https://data.energizer.com/pdfs/fl452od.pdf) informs the broad diffuser and approximate unladen mass. All geometry and physical parameters are in `assets.py`.

## Run and score

In an official YAM/Isaac setup, from this directory with a Dream key in `~/.dreamlabs/credentials`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  --policy-checkpoint dreamlabs/dream-1-beta \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Finish installing the emergency lantern's two loose D batteries. Inspect the + and - marks in the empty wells to orient each battery, close the battery cover, and switch the light on." \
  --scorer task_score --max-steps 1800 --store-frames --seed 42 \
  --log-dir /tmp/emergency-lantern-dream

python demo.py /tmp/emergency-lantern-demo --max-steps 1800
```

The Dream trial scored **0.00** in 1,800 steps; the robot-action scripted demo reached **1.00** and success in 1,637 steps. The demo issues YAM joint actions through inverse kinematics and does not write part poses. Both use `task.py`. Each run permits 90 seconds of simulated time at 20 Hz; on the recorded DGX Spark, the full scripted rollout took about 6.5 minutes wall time. `demo.py` saves the setup/final images, all three camera videos, a per-step measured-state trace and `result.json`. A reset restores all objects and circuit state, with no consumables.

`SCORE_OVER="end"` measures the final state. Each distinct, correctly oriented loose cell in an inner well earns 0.20; the close-fitting, seated cover adds 0.20; turning on the lamp with that cover seated adds 0.20. Full 1.00 requires all four correct cells, seated cover, released button and lit lamp stable for 10 frames. A reversed cell or displaced original cell returns 0.00. `task.py` contains the exact numerical position and angle tolerances; the execution hand and claimed inspection are not scored separately.

The cells, cover and button use gravity, contact and friction in PhysX. Electrical continuity and visible light output are a discrete proxy, and the rigid rim has no compliant snap force. Housing mass distribution, friction and button spring are design assumptions; no real-device calibration, force measurement or continuous-collision proof is claimed. The recorded rollout shows one successful simulation, not measured repeated-run reliability.
