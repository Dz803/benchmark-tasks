"""Continuous two-arm body-grasp measurement episode; no pose manipulation."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import imageio
import imageio.v3 as iio
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('out', type=Path)
parser.add_argument('--actions', type=Path, help='replay saved action14 sequence without rerunning IK')
parser.add_argument('--preview-steps', type=int, help='run only the first N saved actions for visual inspection')
args = parser.parse_args()
args.out.mkdir(parents=True, exist_ok=False)


from inspect_robots import Scene
from inspect_robots.types import Action
from inspect_robots_yam.packing import pack
from yam_arena import ik, station
from yam_arena.constants import READY, FINGER_OPEN, FINGER_CLOSED
from yam_arena.demo import ArmPlan, Waypoint
from yam_arena.inspect_embodiment import yam_arena_embodiment
from yam_arena.urdf import axis_angle_matrix


class TiltArmPlan(ArmPlan):
    def target(self, t):
        pos, rot, finger, label = super().target(t)
        blend = ik.smoothstep(min(1., max(0., t / self.waypoints[0].duration)))
        tilt = axis_angle_matrix(np.array([0., 1., 0.]), math.radians(-20) * blend)
        return pos, tilt @ rot, finger, label


def plan(side, sign):
    body = (.345, sign*.250, .970)
    outward = (.345, sign*.280, .970)
    slack = (.345, sign*.244, .970)
    retreat = (.280, sign*.335, .980)
    yaw = -sign * math.pi/2
    return TiltArmPlan([
        Waypoint(1.6, body, FINGER_OPEN, 'approach body', yaw),
        Waypoint(.8, body, FINGER_CLOSED, 'grasp body', yaw),
        Waypoint(.8, outward, FINGER_CLOSED, 'pull body', yaw),
        Waypoint(2.3, outward, FINGER_CLOSED, 'hold load', yaw),
        Waypoint(.8, body, FINGER_CLOSED, 'unload body', yaw),
        Waypoint(2.3, body, FINGER_CLOSED, 'hold unloaded', yaw),
        Waypoint(.7, slack, FINGER_CLOSED, 'clear residual link tension', yaw),
        Waypoint(.6, slack, FINGER_OPEN, 'open fingers', yaw),
        Waypoint(1.2, retreat, FINGER_OPEN, 'retreat clear', yaw),
    ], station.BASES[side], q_start=READY, finger_start=FINGER_OPEN)


left_plan = plan('left', 1)
right_plan = plan('right', -1)
idle = np.r_[READY, 1.0]
baseline_s = 2.3
total_s = baseline_s + left_plan.duration + right_plan.duration + 2.3
nsteps = math.ceil(total_s*20)
saved_actions = json.loads(args.actions.read_text()) if args.actions else None
if saved_actions is not None:
    if not saved_actions or any(len(row['action14']) != 14 for row in saved_actions):
        raise ValueError('saved actions must contain nonempty 14D action rows')
    nsteps = len(saved_actions)
    total_s = float(saved_actions[-1]['t'])
if args.preview_steps is not None:
    if saved_actions is None or not 1 <= args.preview_steps <= nsteps:
        raise ValueError('--preview-steps requires saved actions and a valid step count')
    nsteps = args.preview_steps
    total_s = float(saved_actions[nsteps-1]['t'])
emb = yam_arena_embodiment(task=str(Path(__file__).with_name('task.py').resolve()),
                          object_jitter=0, control_hz=20, cam_width=640,
                          cam_height=352, top_cam='demo')
video = None
try:
    emb.reset(Scene(id='servo_continuous_episode', instruction=emb.task.module.INSTRUCTION))
    video = imageio.get_writer(str(args.out/'episode.mp4'), fps=10, codec='libx264', quality=7)
    phases=[]
    actions=[]
    last_info={}
    for k in range(nsteps):
        if saved_actions is not None:
            t = float(saved_actions[k]['t'])
            phase = saved_actions[k]['phase']
            packed = np.asarray(saved_actions[k]['action14'], dtype=float)
        else:
            t=(k+1)/20
            if t < baseline_s:
                la,ra=idle,idle
                phase='baseline'
            elif t < baseline_s+left_plan.duration:
                la,lbl=left_plan.action7(t-baseline_s)
                ra=idle
                phase='left_robot:'+lbl
            elif t < baseline_s+left_plan.duration+right_plan.duration:
                la,_=left_plan.action7(left_plan.duration)
                ra,lbl=right_plan.action7(t-baseline_s-left_plan.duration)
                phase='right_robot:'+lbl
            else:
                la,_=left_plan.action7(left_plan.duration)
                ra,_=right_plan.action7(right_plan.duration)
                phase='final clear hold'
            packed=pack(la,ra)
        actions.append({'t':t,'phase':phase,'action14':np.asarray(packed,dtype=float).tolist()})
        result=emb.step(Action(data=packed))
        phases.append(phase)
        last_info=result.info
        if k%2==0:
            imgs=result.observation.images
            top=np.asarray(imgs['top_cam'])[:,:,:3].astype(np.uint8)
            left=np.asarray(imgs['left_cam'])[:,:,:3].astype(np.uint8)
            right=np.asarray(imgs['right_cam'])[:,:,:3].astype(np.uint8)
            wrist=np.vstack((cv2.resize(left,(320,176)),cv2.resize(right,(320,176))))
            canvas=np.hstack((top,wrist))
            rt=emb.task.module.RUNTIME
            if rt.last:
                cv2.putText(canvas,f't={t:.1f}s L={rt.last.left_n:.2f}N R={rt.last.right_n:.2f}N enc={rt.last.encoder_count} score={rt.acquisition.score:.1f}',
                            (12,340),cv2.FONT_HERSHEY_SIMPLEX,.48,(255,255,255),1,cv2.LINE_AA)
            video.append_data(canvas)
        if k in (0,45,110,160,210,270,320,370,430,nsteps-1):
            for name,pixels in result.observation.images.items():
                rgb=np.asarray(pixels)[:,:,:3].astype(np.uint8)
                iio.imwrite(args.out/f'{k+1:04d}_{name}.png',rgb)
                if name == 'top_cam':
                    iio.imwrite(args.out/f'{k+1:04d}_{name}_320.png',cv2.resize(rgb,(320,176),interpolation=cv2.INTER_AREA))
    rt=emb.task.module.RUNTIME
    for row in rt.rows:
        row['robot_phase']=phases[min(nsteps-1,round(row['t']*20)-1)]
    (args.out/'telemetry.json').write_text(json.dumps(rt.rows,indent=2)+'\n')
    (args.out/'actions.json').write_text(json.dumps(actions,indent=2)+'\n')
    report={'scope':'continuous action-only YAM two-arm measurement episode',
            'steps':nsteps,'duration_s':total_s,
            'action_source':str(args.actions) if args.actions else 'generated with official IK',
            'left_ik_mm':None if saved_actions is not None else left_plan.worst_ik_mm,
            'right_ik_mm':None if saved_actions is not None else right_plan.worst_ik_mm,
            'score':rt.acquisition.score,'acquisition':rt.acquisition.report(),
            'last_info':last_info,'diagnostics':rt.diagnostics[:20],
            'peak_force_n':{s:max(r[s+'_n'] for r in rt.rows) for s in ('left','right')},
            'video':'episode.mp4','telemetry':'telemetry.json','actions':'actions.json'}
    (args.out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'score':report['score'],'steps':nsteps,
                      'peak_force_n':report['peak_force_n'],
                      'events':report['acquisition']['events']},indent=2),flush=True)
finally:
    if video is not None:
        video.close()
    emb.close()
