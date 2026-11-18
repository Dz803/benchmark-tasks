"""Measured, tension-only link dynamics and acquisition for the YAM fixture.

The links are a reduced-order elastic model, not calibrated PASCO gauges. Forces
and torques come from rigid body poses and are equal/opposite at the two ends.
No output-body pose or encoder value is commanded.
"""
from __future__ import annotations

import math
import numpy as np

from scorer import AcquisitionScore, Reading

K_N_PER_M = 100.0
REST_M = .180
LEVER_TIP_LOCAL = np.array([0., 0., .210])
GAUGE_EYE_LOCAL = {'left': np.array([.073, 0., 0.]),
                   'right': np.array([-.073, 0., 0.])}


def _np(t):
    t = getattr(t, 'torch', t)
    return t.detach().cpu().numpy().astype(float)


def _rotate(q, v):
    x, y, z, w = q
    return v + 2 * np.cross(q[:3], np.cross(q[:3], v) + w * v)


def _body(env, name):
    a = env.unwrapped.scene[name]
    p = _np(a.data.root_link_pose_w)[0]
    com = _np(a.data.root_com_pose_w)[0, :3]
    return a, p[:3], p[3:7], com


def _wrench(asset, force, moment):
    import torch
    device = asset.device
    f = torch.as_tensor(force, dtype=torch.float32, device=device).reshape(1, 1, 3)
    m = torch.as_tensor(moment, dtype=torch.float32, device=device).reshape(1, 1, 3)
    asset.set_external_force_and_torque(f, m, is_global=True)


class FixtureRuntime:
    def __init__(self):
        self.reset()

    def reset(self):
        self.acquisition = AcquisitionScore()
        self.step_count = 0
        self.rows = []
        self.last = None
        self.forces = {'left': 0., 'right': 0.}
        self.tip = None
        self.eyes = {}
        self.gauge_poses = {}
        self.diagnostics = []
        self.display_chars = None
        self.segment_translations = {}

    def update_visuals(self, env, encoder):
        """Move only render prims from measured endpoints and measured readings."""
        from isaaclab.sim.utils.stage import get_current_stage
        from partkit import _SPECS
        from pxr import Gf, UsdGeom
        stage = get_current_stage()
        _, bp, bq, _ = _body(env, 'base_plate')
        inv = np.array([-bq[0],-bq[1],-bq[2],bq[3]])
        root = _SPECS['base_plate'].meta['prim_path']
        for side in ('left','right'):
            line = UsdGeom.Cylinder.Get(stage, f'{root}/body/band_{side}')
            a = _rotate(inv, self.tip-bp)
            b = _rotate(inv, self.eyes[side]-bp)
            delta = b-a
            length = float(np.linalg.norm(delta))
            direction = delta/length
            v = np.cross(np.array([1.,0.,0.]),direction)
            q = np.r_[1+direction[0],v]
            if np.linalg.norm(q)<1e-7:
                q=np.array([0.,0.,0.,1.])
            else:
                q=q/np.linalg.norm(q)
            line.GetHeightAttr().Set(length)
            line.GetPrim().GetAttribute('xformOp:translate').Set(Gf.Vec3d(*((a+b)/2)))
            line.GetPrim().GetAttribute('xformOp:orient').Set(Gf.Quatf(float(q[0]),*map(float,q[1:])))
            gauge_root = _SPECS[f'{side}_spring_gauge'].meta['prim_path']
            pointer = stage.GetPrimAtPath(f'{gauge_root}/body/force_pointer')
            x = -.052 + .052*min(2.,max(0.,self.forces[side]))
            pointer.GetAttribute('xformOp:translate').Set(Gf.Vec3d(x,0.,.0104))
        from readout_texture import update as update_readout
        recorder_root = _SPECS['measurement_recorder'].meta['prim_path']
        update_readout(stage, recorder_root, self.forces['left'], self.forces['right'],
                       encoder, self.step_count)

    def step(self, env):
        self.step_count += 1
        lever, lp, lq, lcom = _body(env, 'output_lever')
        tip_offset = _rotate(lq, LEVER_TIP_LOCAL)
        tip = lp + tip_offset
        self.tip = tip
        lever_force = np.zeros(3)
        lever_torque = np.zeros(3)
        angle = math.degrees(2 * math.atan2(float(lq[1]), float(lq[3])))
        gauge_data = {}
        for side in ('left', 'right'):
            gauge, gp, gq, gcom = _body(env, f'{side}_spring_gauge')
            eye_offset = _rotate(gq, GAUGE_EYE_LOCAL[side])
            eye = gp + eye_offset
            self.eyes[side] = eye
            self.gauge_poses[side] = (gp, gq)
            delta = eye - tip
            length = float(np.linalg.norm(delta))
            tension = K_N_PER_M * max(0., length - REST_M)
            direction = delta / length if length > 1e-8 else np.zeros(3)
            force_on_lever = tension * direction
            force_on_gauge = -force_on_lever
            lever_force += force_on_lever
            lever_torque += np.cross(tip-lcom, force_on_lever)
            _wrench(gauge, force_on_gauge, np.cross(eye-gcom, force_on_gauge))
            self.forces[side] = tension
            tangent_error = math.degrees(math.atan2(abs(delta[0])+abs(delta[2]), abs(delta[1])))
            gauge_data[side] = dict(eye_xyz_m=eye.tolist(), length_m=length,
                                    force_n=tension, tangent_error_deg=tangent_error)
        _wrench(lever, lever_force, lever_torque)
        if self.step_count % 2:
            return
        # This is a conservative body-space exclusion test: an end effector
        # inside the protected lever/servo envelope invalidates acquisition.
        direct_contact = False
        arm_data = {}
        try:
            from yam_arena import ik, station
            from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, FINGER_OPEN
            for name in ('left_robot', 'right_robot'):
                arm = env.unwrapped.scene[name]
                joints = list(arm.joint_names)
                q = _np(arm.data.joint_pos)[0]
                q6 = q[[joints.index(n) for n in ARM_JOINTS]]
                finger = float(np.mean(q[[joints.index(n) for n in FINGER_JOINTS]]))
                tcp, _ = ik.tcp_pose(q6, station.BASES[name.split('_')[0]])
                shaft = lp + _rotate(lq, np.array([0., 0., .110]))
                servo_center = lp + _rotate(lq, np.array([0., 0., .075]))
                link_poses = _np(arm.data.body_link_pose_w)[0]
                points = [tcp]
                centers = {}
                for link_name, pose in zip(arm.body_names, link_poses):
                    if ('gripper' in link_name or 'tip_' in link_name or
                            'link5' in link_name or 'link6' in link_name):
                        centers[link_name] = pose[:3]
                        points.append(pose[:3])
                # The point-only origin check misses an extended finger or
                # distal link crossing a protected part between its origins.
                # Sample the real link-centre spans at <=10 mm spacing.
                span_count = 0
                for a_name,b_name in (('link5','gripper'),('gripper','tip_left'),
                                      ('gripper','tip_right')):
                    if a_name not in centers or b_name not in centers:
                        raise RuntimeError(f'missing protected robot span: {name} {a_name}->{b_name}')
                    a,b = centers[a_name],centers[b_name]
                    subdivisions = max(2,math.ceil(float(np.linalg.norm(b-a))/.010))
                    points.extend(a+(b-a)*(i/subdivisions)
                                  for i in range(1,subdivisions))
                    span_count += 1
                if len(points) < 2:
                    raise RuntimeError(f'arm link collision envelope unavailable: {name}')
                segment = tip-shaft
                protected = min(min(np.linalg.norm(p-servo_center)-.037,
                                    np.linalg.norm(p-(shaft+np.clip(np.dot(p-shaft,segment)/
                                    max(np.dot(segment,segment),1e-9),0.,1.)*segment))-.018)
                                for p in points)
                # The exposed gauge eyelet is also a forbidden direct
                # push point; the body centre remains grippable.
                eyelet_clearance = min(np.linalg.norm(p-self.eyes[s])-.009
                              for p in points for s in ('left','right'))
                direct_contact |= protected < .018 or eyelet_clearance < .018
                arm_data[name] = dict(tcp_xyz_m=tcp.tolist(), finger_m=finger,
                                      protected_distance_m=float(protected),
                                      eyelet_clearance_m=float(eyelet_clearance),
                                      sampled_spans=span_count)
            gauges_released = all(v['finger_m'] <= .9 * FINGER_OPEN for v in arm_data.values())
            # Require all open fingers to be spatially clear of each gauge's
            # actual oriented body, not merely clear of its inner eyelet.
            for s in ('left','right'):
                gp, gq = self.gauge_poses[s]
                q_inv = np.array([-gq[0],-gq[1],-gq[2],gq[3]])
                for v in arm_data.values():
                    local = _rotate(q_inv, np.array(v['tcp_xyz_m'])-gp)
                    body_gap = np.linalg.norm(np.maximum(np.abs(local)-np.array([.070,.010,.009]),0.))
                    gauges_released &= body_gap > .040
        except Exception as exc:
            # Missing contact telemetry is a hard acquisition failure.
            direct_contact = True
            gauges_released = False
            self.diagnostics.append(f'robot state unavailable at step {self.step_count}: {exc}')
        # Fixed attachments and the held input are queried from simulated
        # bodies/drive, rather than using unconditional validity defaults.
        from yam_arena.task_file import TABLE_TOP
        fixture_intact = True
        for name, expected in (('base_plate', (.345, 0., TABLE_TOP['station'])),
                               ('printed_servo_holder', (.345, 0., TABLE_TOP['station'])),
                               ('held_uart_servo', (.345, 0., TABLE_TOP['station']))):
            _, p, q, _ = _body(env, name)
            fixture_intact &= np.linalg.norm(p-np.array(expected)) < .005
            expected_q = np.array([0.,0.,math.sin(math.pi/4),math.cos(math.pi/4)])
            fixture_intact &= abs(float(np.dot(q,expected_q))) > math.cos(math.radians(3)/2)
        links_intact = bool(all(np.isfinite(gauge_data[s]['length_m']) and
                                gauge_data[s]['length_m'] < .230 for s in ('left','right')))
        try:
            from isaaclab.sim.utils.stage import get_current_stage
            from pxr import UsdPhysics
            stage = get_current_stage()
            from partkit import _HINGES
            prim = stage.GetPrimAtPath(_HINGES['motor_rotor'])
            drive = UsdPhysics.DriveAPI(prim, 'angular')
            setting_unchanged = bool(prim.IsValid() and abs(float(drive.GetTargetPositionAttr().Get())) < 1e-6)
        except Exception:
            setting_unchanged = False
        t = self.step_count / 20.0
        reading = Reading(t=t, left_n=self.forces['left'], right_n=self.forces['right'],
                          encoder_count=round(angle * 4096 / 360),
                          left_tangent_error_deg=gauge_data['left']['tangent_error_deg'],
                          right_tangent_error_deg=gauge_data['right']['tangent_error_deg'],
                          direct_contact=bool(direct_contact), fixture_intact=bool(fixture_intact),
                          links_intact=links_intact, setting_unchanged=setting_unchanged,
                          gauges_released=bool(gauges_released))
        self.acquisition.update(reading)
        self.update_visuals(env, reading.encoder_count)
        self.last = reading
        self.rows.append({**reading.__dict__, 'output_deg':angle,
                          'motor_setpoint_deg':0., 'tip_xyz_m':tip.tolist(),
                          'gauges':gauge_data, 'arms':arm_data,
                          'score':self.acquisition.score})


RUNTIME = FixtureRuntime()
