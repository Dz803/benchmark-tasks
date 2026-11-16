"""Prototype a dark-room, torch-assisted guarded worklight activation task."""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from lettering import label
from partkit import Slider, on_reset, on_step, part, set_light_intensity, sync_light_pose
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
from yam_arena.task_file import TABLE_TOP

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = (
    "The room lights are out. A lit flashlight is already held in one gripper, "
    "and the covered ON control is not visible at the start. Sweep the beam "
    "across the workbench to find the control. Once you find it, keep it "
    "illuminated while your other gripper slides the clear guard aside and "
    "presses ON. Leave the backup worklight illuminating the bench."
)

TORCH_XY = (.43, .225)
CONTROL_XY = (.31, -.135)
WORKLIGHT_XY = (.30, 0.0)
WORKLIGHT_INTENSITY = 2000000.0
TORCH_INTENSITY = 3000000.0
EMERGENCY_FILL_INTENSITY = 250.0
BLACKOUT_AMBIENT_INTENSITY = 0.0
# Measured arm and loose torch state from step 99 of a successful grip/lift.
# The beam begins farther across the bench; the control is not illuminated.
INITIAL_ACTION = (-.18395, 2.36322, 2.78725, -1.69297,
                  0.0, -.18394, 0.0,
                  0.0, 1.0, 1.0, -.6, 0.0, 0.0, 1.0)
INITIAL_TORCH_POSE = ((.46789882, .21435076, .91164285),
                      (-.02738022, .00198765, .01539668, .99950445))
INK = (.045, .055, .065)
CASE = (.17, .205, .23)
STEEL = (.50, .54, .56)
BLUE = (.08, .42, .82)
GLASS = (.64, .82, .91)
WHITE = (.92, .94, .90)
YELLOW = (.98, .73, .10)
RED = (.74, .14, .10)


def b(x, y, z, sx, sy, sz, color):
    return ((x, y, z), (sx, sy, sz), color)


def c(x, y, z, radius, height, color, axis="Z"):
    return ((x, y, z), radius, height, color, axis)


torch = part("torch", (*TORCH_XY, TOP + .112), [
    # Opposed flats give the parallel YAM fingers a repeatable 35 mm grip.
    b(0, -.016, .040, .032, .005, .100, YELLOW),
    b(0, .016, .040, .032, .005, .100, YELLOW),
], .204, cylinders=[
    c(0, 0, -.032, .020, .066, INK),
    c(0, 0, -.067, .022, .012, CASE),
    c(0, 0, -.074, .019, .003, WHITE),
    c(0, 0, .038, .016, .095, YELLOW),
    c(0, 0, .0975, .018, .016, INK),
    c(0, 0, -.006, .0185, .004, INK),
    c(0, 0, .087, .0185, .004, INK),
], friction=1.5)
torch.meta["spotlight"] = True
torch.meta["spotlight_follow_body"] = True
torch.meta["spotlight_offset"] = (0.0, 0.0, -.080)
torch.meta["spotlight_radius"] = .004
torch.meta["spotlight_color"] = (.91, .96, 1.0)
torch.meta["spotlight_cone_angle"] = 20.0
torch.meta["spotlight_cone_softness"] = .35
torch.meta["spotlight_spill"] = (60.0, .72, .15)

# A fixed guarded control: the clear window slides right on two rails. The
# red button is separated from the window by 24 mm of vertical clearance.
on_button = Slider("on_button", rest=(.018, .018, .078),
                   cylinders=(c(.018, .018, .078, .018, .012, RED),),
                   travel=.007, rate=330.0, damping=5.0, mass=.012)
window = Slider("window", rest=(0, 0, .111), boxes=(
    b(0, 0, .111, .147, .142, .006, GLASS),
    b(0, -.082, .122, .102, .020, .020, YELLOW),
    b(-.070, 0, .114, .007, .141, .012, STEEL),
    b(.070, 0, .114, .007, .141, .012, STEEL),
), travel=.155, rate=0.0, damping=8.0, mass=.11, axis="Y", sign=-1)
control = part("control", (*CONTROL_XY, TOP), [
    b(0, -.070, .022, .180, .330, .044, CASE),
    b(-.079, .004, .074, .010, .160, .104, STEEL),
    b(.079, .004, .074, .010, .160, .104, STEEL),
    # Low near-side lip preserves the guard rail without shadowing the ON key.
    b(0, .074, .050, .168, .010, .052, STEEL),
    b(0, -.069, .074, .168, .010, .104, STEEL),
    b(-.084, -.070, .111, .008, .322, .012, INK),
    b(.084, -.070, .111, .008, .322, .012, INK),
    b(.018, .018, .060, .120, .090, .010, WHITE),
    b(-.030, .018, .053, .030, .032, .010, INK),
    *label("ON", .018, .018, .060, .0035, WHITE),
], 2.0, sliders=(on_button, window), fixed_to_world=True)
control.meta["opacity_colors"] = {GLASS: .22}
control.meta["visual_boxes"] = label("BACKUP", 0, .106, .047, .0030, YELLOW)

# The backup ceiling light sits outside the policy camera's tabletop view;
# only its illumination changes when the physical ON button travels.
worklight = part("worklight", (*WORKLIGHT_XY, TOP + .55), [
    b(0, 0, 0, .002, .002, .002, INK),
], 1.0, fixed_to_world=True)
worklight.meta["spotlight"] = True
worklight.meta["spotlight_cone_angle"] = 80.0
worklight.meta["spotlight_cone_softness"] = .45

OBJECTS = [torch, control, worklight]
STATE: dict = {"score": 0.0}


def _dim_station():
    """Switch off station lighting so only the held torch lights the reset scene."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdLux

    for prim in get_current_stage().Traverse():
        if prim.IsA(UsdLux.DomeLight):
            UsdLux.DomeLight(prim).GetIntensityAttr().Set(BLACKOUT_AMBIENT_INTENSITY)
        elif prim.IsA(UsdLux.SphereLight) and "task_light" not in str(prim.GetPath()):
            UsdLux.SphereLight(prim).GetIntensityAttr().Set(0.0)


def _set_emergency_fill(enabled: bool):
    """Restore room-wide indirect light when the backup supply is switched on."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdLux

    intensity = EMERGENCY_FILL_INTENSITY if enabled else BLACKOUT_AMBIENT_INTENSITY
    for prim in get_current_stage().Traverse():
        if prim.IsA(UsdLux.DomeLight):
            UsdLux.DomeLight(prim).GetIntensityAttr().Set(intensity)


def _torch_aimed(torch_state) -> bool:
    origin = torch_state.to_world(torch.meta["spotlight_offset"])
    target = np.array([CONTROL_XY[0] + .018, CONTROL_XY[1] + .018, TOP + .080])
    ray = target - origin
    distance = float(np.linalg.norm(ray))
    if not .08 <= distance <= .65:
        return False
    direction = torch_state.rotation() @ np.array([0., 0., -1.])
    return (float(np.dot(direction, ray / distance)) >= math.cos(math.radians(28))
            and torch_state.pos[2] > TOP + .135)


@on_reset
def reset(environment=None):
    # Set a reproducible, contact-held starting state. No attachment or pose
    # changes occur after reset; all subsequent torch motion is through fingers.
    sim = environment.unwrapped
    robot = sim.scene["left_robot"]
    arm_ids = robot.find_joints(ARM_JOINTS, preserve_order=True)[0]
    finger_ids = robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]
    q = robot.data.joint_pos.torch.clone()
    q[0, arm_ids] = q.new_tensor(INITIAL_ACTION[:6])
    q[0, finger_ids] = 0.0
    robot.write_joint_position_to_sim_index(position=q)
    robot.write_joint_velocity_to_sim_index(velocity=q.new_zeros(q.shape))
    asset = sim.scene["torch"]
    pose = asset.data.root_pose_w.torch.clone()
    pose[0, :3] = pose.new_tensor(INITIAL_TORCH_POSE[0]) + sim.scene.env_origins[0]
    pose[0, 3:7] = pose.new_tensor(INITIAL_TORCH_POSE[1])
    asset.write_root_pose_to_sim_index(root_pose=pose)
    asset.write_root_velocity_to_sim_index(
        root_velocity=asset.data.default_root_vel.torch.new_zeros(asset.data.default_root_vel.torch.shape))
    STATE.clear()
    STATE.update(torch_lit_steps=0, torch_used=False, guard_opened_lit=False,
                 worklight_on=False, valid_activation=False, invalid_activation=False,
                 lit_steps=0, previous_guard=0.0, score=0.0)
    _dim_station()
    _set_emergency_fill(False)
    set_light_intensity("torch", TORCH_INTENSITY)
    set_light_intensity("worklight", 0.0)
    from yam_arena.scene import object_states
    sync_light_pose("torch", object_states(environment, ["torch"])["torch"])


@on_step
def monitor(environment):
    from yam_arena.scene import object_states

    states = object_states(environment, ["torch", "control"])
    control_state = states["control"]
    sync_light_pose("torch", states["torch"])
    aimed = _torch_aimed(states["torch"])
    STATE["torch_lit_steps"] = STATE["torch_lit_steps"] + 1 if aimed else 0
    STATE["torch_used"] |= STATE["torch_lit_steps"] >= 10
    guard_open = control_state.sliders["window"] < -.125
    button_pressed = control_state.sliders["on_button"] < -.0045
    if guard_open and STATE["previous_guard"] >= -.125 and aimed and STATE["torch_used"]:
        STATE["guard_opened_lit"] = True
    # The first activation is decisive: illuminating the area later cannot
    # retroactively turn a blind press into a torch-guided activation.
    if button_pressed and guard_open and not STATE["worklight_on"]:
        STATE["worklight_on"] = True
        if STATE["guard_opened_lit"] and aimed:
            STATE["valid_activation"] = True
        else:
            STATE["invalid_activation"] = True
    STATE["lit_steps"] = STATE["lit_steps"] + 1 if STATE["worklight_on"] else 0
    set_light_intensity("worklight", WORKLIGHT_INTENSITY if STATE["worklight_on"] else 0.0)
    _set_emergency_fill(STATE["worklight_on"])
    STATE.update(guard_open=guard_open, button_pressed=button_pressed,
                 torch_aimed=aimed,
                 previous_guard=control_state.sliders["window"])
    STATE["score"] = score(None)


def score(_objects):
    if STATE.get("invalid_activation"):
        return 0.0
    if not STATE.get("torch_used"):
        return 0.0
    if not STATE.get("guard_opened_lit"):
        return .2
    if not STATE.get("valid_activation") or STATE.get("lit_steps", 0) < 10:
        return .5
    return 1.0


def success(_objects):
    return score(None) == 1.0
