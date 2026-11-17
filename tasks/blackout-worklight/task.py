"""Search a dark bench, open a glass access case, and pull its light switch."""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from lettering import label
from partkit import hinge_lower_limit, motor, on_reset, on_step, part, set_light_intensity, sync_light_pose
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
from yam_arena.task_file import TABLE_TOP

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = (
    "The room lights are out. A lit flashlight is already held in one gripper, "
    "pointing straight down; the emergency-light cabinet is not visible at "
    "the start. Sweep the work area from left to right with overlapping "
    "beam positions to find the glass access case. Keep the case illuminated while the other "
    "gripper flips the clear access lid upward, then reach inside, grasp the top bar of the "
    "folding U-shaped emergency-light handle, and pull it down to ON. "
    "Release it and leave the backup light on."
)

TORCH_XY = (.43, .225)
CONTROL_XY = (.35, -.190)
CONTROL_YAW = -math.pi / 2  # local front (-Y) faces the camera / near table edge
WORKLIGHT_XY = (.30, 0.0)
WORKLIGHT_INTENSITY = 2000000.0
TORCH_INTENSITY = 6000000.0
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
STEEL = (.73, .77, .78)
BLUE = (.08, .42, .82)
GLASS = (.64, .82, .91)
WHITE = (.92, .94, .90)
YELLOW = (.98, .73, .10)
RED = (.74, .14, .10)


def b(x, y, z, sx, sy, sz, color):
    return ((x, y, z), (sx, sy, sz), color)


def c(x, y, z, radius, height, color, axis="Z"):
    return ((x, y, z), radius, height, color, axis)


def control_point(x, y, z):
    """A position on the front-facing cabinet, in world/table coordinates."""
    co, si = math.cos(CONTROL_YAW), math.sin(CONTROL_YAW)
    return (CONTROL_XY[0] + co * x - si * y,
            CONTROL_XY[1] + si * x + co * y, TOP + z)


def control_direction(world_direction):
    """Express an observed world direction in the cabinet's local frame."""
    co, si = math.cos(CONTROL_YAW), math.sin(CONTROL_YAW)
    x, y, z = world_direction
    return np.array([co * x + si * y, -si * x + co * y, z])


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
torch.meta["spotlight_cone_angle"] = 34.0
torch.meta["spotlight_cone_softness"] = .35
torch.meta["spotlight_spill"] = (70.0, .72, .25)

# The lid pivots at the back edge and lifts over the handle's sweep.
control = part("control", (*CONTROL_XY, TOP), [
    b(0, -.070, .022, .180, .330, .044, CASE),
    b(-.079, .004, .093, .010, .160, .174, GLASS),
    b(.079, .004, .093, .010, .160, .174, GLASS),
    b(0, .074, .050, .168, .010, .052, GLASS),
    b(0, -.069, .074, .168, .010, .104, GLASS),
    b(-.084, .004, .093, .005, .160, .174, STEEL),
    b(.084, .004, .093, .005, .160, .174, STEEL),
    b(-.084, .004, .177, .008, .160, .012, STEEL),
    b(.084, .004, .177, .008, .160, .012, STEEL),
    b(.018, .018, .060, .120, .090, .010, WHITE),
    b(.018, .018, .074, .034, .034, .025, INK),
    *label("OFF", .018, .046, .060, .0028, INK),
    *label("ON", .018, -.054, .060, .0035, RED),
], 2.0, yaw=CONTROL_YAW, fixed_to_world=True)
control.meta["opacity_colors"] = {GLASS: .22}
control.meta["glass_colors"] = {GLASS}
control.meta["visual_boxes"] = label("BACKUP", 0, .106, .047, .0030, YELLOW)

LID_PIVOT = control_point(0, .075, .180)
lid = part("glass_lid", LID_PIVOT, [
    b(0, -.075, 0, .147, .150, .006, GLASS),
    b(0, -.151, .011, .102, .020, .020, YELLOW),
    b(-.071, -.075, .003, .007, .150, .012, STEEL),
    b(.071, -.075, .003, .007, .150, .012, STEEL),
], .11, yaw=CONTROL_YAW, friction=1.0, hinge_attachment=(
    "control", (0, .075, .180), (0, 0, 0),
    0.0, -140.0, 0.0, "X", 0.0, .003, .2,
))
# Light-scattering clear polycarbonate: unlike ideal glass it remains visible
# where the handheld flashlight hits it in a room with no ambient lighting.
lid.meta["opacity_colors"] = {GLASS: .42}
# Large lettering on the outside of the clear guard becomes legible only
# when the handheld beam finds the cabinet; it does not add a light source.
lid.meta["visual_boxes"] = (
    label("BACKUP", 0, -.065, .0055, .0050, YELLOW)
    + label("POWER", 0, -.105, .0055, .0050, YELLOW)
)

LEVER_PIVOT = control_point(.018, .018, .078)
lever = part("on_lever", LEVER_PIVOT, [
    # One articulated U-shaped folding bail: two arms and a graspable crossbar.
    # 80 mm overall width and 64 mm hinge spacing, with room for YAM fingers
    # around the crossbar. The single X-axis hinge moves the entire bail.
    b(-.032, 0, .040, .012, .018, .072, CASE),
    b(.032, 0, .040, .012, .018, .072, CASE),
    b(0, 0, .077, .080, .022, .018, RED),
    b(0, -.012, .077, .030, .003, .013, RED),
], .12, yaw=CONTROL_YAW, cylinders=(
    c(-.032, 0, 0, .013, .014, INK, "X"),
    c(.032, 0, 0, .013, .014, INK, "X"),
), friction=1.0, hinge_attachment=(
    "control", (.018, .018, .078), (0, 0, 0),
    60.0, 0.0, 0.0, "X", .0005, .003, .2,
))

# The backup ceiling light sits outside the policy camera's tabletop view;
# only its illumination changes when the physical lever crosses its ON detent.
worklight = part("worklight", (*WORKLIGHT_XY, TOP + .55), [
    b(0, 0, 0, .002, .002, .002, INK),
], 1.0, fixed_to_world=True)
worklight.meta["spotlight"] = True
worklight.meta["spotlight_cone_angle"] = 80.0
worklight.meta["spotlight_cone_softness"] = .45

OBJECTS = [torch, control, lid, lever, worklight]
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
    target = np.array(control_point(.018, .018, .130))
    ray = target - origin
    distance = float(np.linalg.norm(ray))
    if not .08 <= distance <= .65:
        return False
    # The optical-axis proxy must also have a clear path into the case.
    # Transparent lid geometry is deliberately excluded; solid case walls are not.
    a = control_direction(origin - np.array((*CONTROL_XY, TOP)))
    b = control_direction(target - np.array((*CONTROL_XY, TOP)))
    for center, size in (
        ((0., -.070, .022), (.180, .330, .044)),
        ((-.079, .004, .093), (.010, .160, .174)),
        ((.079, .004, .093), (.010, .160, .174)),
        ((0., .074, .050), (.168, .010, .052)),
        ((0., -.069, .074), (.168, .010, .104)),
    ):
        lower = np.array(center) - np.array(size) / 2
        upper = np.array(center) + np.array(size) / 2
        lo, hi = 0.0, 1.0
        for axis in range(3):
            delta = b[axis] - a[axis]
            if abs(delta) < 1e-9:
                if a[axis] < lower[axis] or a[axis] > upper[axis]:
                    break
            else:
                t0 = (lower[axis] - a[axis]) / delta
                t1 = (upper[axis] - a[axis]) / delta
                lo, hi = max(lo, min(t0, t1)), min(hi, max(t0, t1))
                if lo > hi:
                    break
        else:
            return False
    direction = torch_state.rotation() @ np.array([0., 0., -1.])
    return (float(np.dot(direction, ray / distance)) >= math.cos(math.radians(28))
            and torch_state.pos[2] > TOP + .135)


def _hands_clear_of_lever(environment, lever_state) -> bool:
    from yam_arena import ik, station

    tip = lever_state.to_world((0.0, 0.0, .077))
    for side in ("left", "right"):
        robot = environment.unwrapped.scene[side + "_robot"]
        ids = robot.find_joints(ARM_JOINTS, preserve_order=True)[0]
        q = robot.data.joint_pos.torch[0, ids].detach().cpu().numpy()
        tcp, _ = ik.tcp_pose(q, np.asarray(station.BASES[side]))
        if np.linalg.norm(tcp - tip) < .05:
            return False
    return True


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
                 lit_steps=0, guard_motion_started=False, first_pull_started=False,
                 first_pull_valid=False, previous_guard=0.0, lid_angle_deg=0.0,
                 lever_angle_deg=0.0,
                 previous_lever_angle_deg=0.0, released_stable_steps=0,
                 hands_clear=False, detent_engaged=False, score=0.0)
    motor("on_lever", 0.0, stiffness=.0005, damping=.003, max_force=.2)
    hinge_lower_limit("on_lever", 0.0)
    _dim_station()
    _set_emergency_fill(False)
    set_light_intensity("torch", TORCH_INTENSITY)
    set_light_intensity("worklight", 0.0)
    from yam_arena.scene import object_states
    sync_light_pose("torch", object_states(environment, ["torch"])["torch"])


@on_step
def monitor(environment):
    from yam_arena.scene import object_states

    states = object_states(environment, ["torch", "glass_lid", "on_lever"])
    sync_light_pose("torch", states["torch"])
    aimed = _torch_aimed(states["torch"])
    lid_normal = control_direction(states["glass_lid"].rotation() @ np.array([0., 0., 1.]))
    guard_angle = math.degrees(math.atan2(lid_normal[1], lid_normal[2]))
    guard_open = guard_angle >= 115.0
    lever_direction = control_direction(states["on_lever"].rotation() @ np.array([0., 0., 1.]))
    lever_angle = math.degrees(math.atan2(-lever_direction[1], lever_direction[2]))
    advance_score_state(STATE, aimed, guard_angle, lever_angle)
    lever_on = lever_angle >= 48.0
    if lever_angle >= 50.0 and not STATE["detent_engaged"]:
        STATE["detent_engaged"] = True
        # A one-way pawl engages at the measured 50° crossing and prevents
        # reverse travel below 49°. No rigid body pose or score is assigned.
        hinge_lower_limit("on_lever", 49.0)
    clear = _hands_clear_of_lever(environment, states["on_lever"])
    stable = abs(lever_angle - STATE["previous_lever_angle_deg"]) <= 1.0
    STATE["released_stable_steps"] = (
        STATE["released_stable_steps"] + 1
        if STATE["valid_activation"] and lever_on and clear and stable else 0)
    set_light_intensity("worklight", WORKLIGHT_INTENSITY if STATE["worklight_on"] else 0.0)
    _set_emergency_fill(STATE["worklight_on"])
    STATE.update(guard_open=guard_open, lid_angle_deg=guard_angle, lever_angle_deg=lever_angle,
                 lever_on=lever_on, torch_aimed=aimed, hands_clear=clear,
                 previous_guard=guard_angle,
                 previous_lever_angle_deg=lever_angle)
    STATE["score"] = score(None)


def advance_score_state(state, aimed, guard_angle_deg, lever_angle_deg):
    """Score measured motion every control step, including first unsafe motion."""
    state["torch_lit_steps"] = state["torch_lit_steps"] + 1 if aimed else 0
    state["torch_used"] |= state["torch_lit_steps"] >= 10
    guard_open = guard_angle_deg >= 115.0
    # Start at the first measurable motion: the successful reset/opening trace
    # stays below 0.06° lid jitter and below 1.0° passive lever motion.
    guard_moving = guard_angle_deg >= 0.5
    pull_started = lever_angle_deg >= 1.5
    lever_on = lever_angle_deg >= 48.0
    if guard_moving and not state["guard_motion_started"]:
        state["guard_motion_started"] = True
        if not aimed or not state["torch_used"]:
            state["invalid_activation"] = True
    if state["guard_motion_started"] and not state["worklight_on"]:
        if not aimed:
            state["invalid_activation"] = True
        if guard_open and not state["invalid_activation"]:
            state["guard_opened_lit"] = True
    if pull_started and not state["first_pull_started"]:
        state["first_pull_started"] = True
        state["first_pull_valid"] = bool(
            guard_open and state["guard_opened_lit"] and aimed
            and not state["invalid_activation"])
        if not state["first_pull_valid"]:
            state["invalid_activation"] = True
    if state["first_pull_started"] and not state["worklight_on"] and not aimed:
        state["invalid_activation"] = True
    if lever_on and not state["worklight_on"]:
        # The real lever can switch the lamp on even during an unsafe pull;
        # the scorer still permanently rejects that sequence.
        state["worklight_on"] = True
        state["valid_activation"] = bool(
            state["first_pull_valid"] and guard_open and aimed
            and not state["invalid_activation"])
        if not state["valid_activation"]:
            state["invalid_activation"] = True
    state["lit_steps"] = state["lit_steps"] + 1 if state["worklight_on"] else 0


def score(_objects):
    if STATE.get("invalid_activation"):
        return 0.0
    if not STATE.get("torch_used"):
        return 0.0
    if not STATE.get("guard_opened_lit"):
        return .2
    if (not STATE.get("valid_activation") or not STATE.get("lever_on")
            or STATE.get("lit_steps", 0) < 10
            or STATE.get("released_stable_steps", 0) < 10):
        return .5
    return 1.0


def success(_objects):
    return score(None) == 1.0
