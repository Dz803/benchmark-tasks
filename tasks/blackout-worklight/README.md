# Find and activate a guarded backup light switch in a blackout

The room lights are out. A lit flashlight starts between one gripper's fingers, but the covered ON control is outside the initially illuminated patch. The robot must sweep the beam across the workbench to find the control, then keep it illuminated while the free gripper slides the clear guard and presses the button. The backup worklight then illuminates the bench. The task tests active illumination during simultaneous physical manipulation, rather than merely selecting a switch from an image. The score does not demand a particular scan path; the scripted raster is one successful way to find the control in the fixed layout.

## Scene and reset

The scene uses the official two-arm YAM station table (1.1 × 1.3 m wood-look top). At reset, a 204 g, approximately 18 cm yellow-and-black plastic-look rigid flashlight is held by the left gripper at station-frame (x≈0.468, y≈0.214 m), about 16 cm above the table and about 37 cm horizontally from the control. A fixed 18 × 33 cm metal-look rigid control housing is at (x=0.31, y=−0.135 m). Its 14.7 × 14.2 cm clear rigid plastic-look guard slides up to 15.5 cm along negative Y on rails; a 10.2 cm yellow rigid bar is its grip. The red rigid ON button is 36 mm in diameter and spring-travels 7 mm. A battery-backed worklight is represented by an overhead light source outside the camera view. The ordinary room lamps, dome and backup source start off; the flashlight starts on. The layout is fixed with `object_jitter=0`.

The supplied setup image shows the actual scored reset: only the flashlight's illuminated patch is visible. The beam initially falls away from the covered control; the robot must search with it before opening the guard. Nothing is consumed: reset the grippers and flashlight to their recorded start poses, slide the guard closed, let the button spring out, and switch the backup light off.

## Score

The task file returns 0.0, 0.2, 0.5 or 1.0. At 20 Hz, the flashlight lens must be 8–65 cm from the ON button and its beam axis within 28° of the button centre for 10 consecutive steps (0.5 s) to earn 0.2. Opening the physical guard by at least 12.5 cm while still aimed, after that sustained aim, earns 0.5. Full score requires the **first** physical ON actuation to happen with the guard open and beam aimed, the button to travel inward at least 4.5 mm, and the backup lighting to remain on for another 10 steps. A blind first actuation gives 0.0 and cannot be repaired by later pointing the torch at the control. The score does not prescribe an exact arm path or require the guard to remain open after a valid press.

## Reproduce

Use Isaac Lab Arena with YAM Arena, Inspect Robots, the Dream-1 policy plugin, and the dependencies imported by the four Python files here. Activate that environment and run from the repository root:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  --policy-checkpoint dreamlabs/dream-1-beta \
  -E task=tasks/blackout-worklight/task.py -E top_cam=demo \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The room lights are out. A lit flashlight is already held in one gripper, and the covered ON control is not visible at the start. Sweep the beam across the workbench to find the control. Once you find it, keep it illuminated while your other gripper slides the clear guard aside and presses ON. Leave the backup worklight illuminating the bench." \
  --scorer task_score --max-steps 1200 --store-frames --seed 42 --no-prompt \
  --log-dir /tmp/blackout-worklight-dream

python tasks/blackout-worklight/demo_contact.py /tmp/blackout-worklight-demo
```

The run limit is 1,200 control steps (60 simulated seconds at 20 Hz); model API calls and rendering can extend wall-clock runtime. The deterministic fixed-layout scripted demo uses joint commands and finger contact only after reset, not mid-episode object poses or teleports. It sweeps the held beam through three left–right raster rows, then aims at the control and scores 1.0 in 399 steps (19.95 simulated seconds) on this scene. All scan rows score 0.0; aiming, guard travel and valid activation earn the later milestones. This sweep is open-loop and does not prove visual search by a model. One official Dream-1 beta trial on this same scene and scorer completed all 1,200 steps without errors and scored 0.0. The prepared model video pairs overhead and flashlight-holding left-wrist cameras, each labeled as a view of the same flashlight.

## Fidelity and sources

The flashlight, housing, slider and button are original procedural USD primitives; no third-party mesh, texture or scene asset is redistributed. The flashlight's approximate dimensions, mass and yellow/black form are informed by the [Streamlight 4AA ProPolymer Lux product information](https://www.streamlight.com/products/detail/4aa-propolymer-lux-div-1). A manually operated battery worklight after mains loss is documented in the [ABB WORK-LITE manual](https://library.e.abb.com/public/4bc74b8345a04db8870b27f75afae481/vanlien_handleiding_bimo.pdf); protective pushbutton covers are catalogued by [Schneider Electric](https://iportal2.schneider-electric.com/Contents/docs/PUSH%20BUTTONS%20AND%20OPERATOR%20INTERFACE_CATALOGUE.PDF). These sources motivate the composite scene; it is not a 3D reproduction of any one product.

The flashlight beam follows its measured PhysX body pose, with co-located 20° central and weaker 60° spill components representing one flashlight lens. The spill is 15% of central intensity; the central renderer intensity is 3,000,000. Both the station light and ambient light are off before activation. The paired model video shows overhead and left-wrist views of the same illuminated area, not two separate flashlight beams. Intensity is tuned to renderer exposure, not calibrated lux; the visual plastic/metal materials and contact friction/stiffness are not product-calibrated. The overhead source and room-wide fill after switch activation approximate battery-backed work lighting; electrical wiring, battery discharge, exact optics and room-scale travel are omitted. The scorer checks the beam's geometric aim and physical joint travel, not camera pixels.
