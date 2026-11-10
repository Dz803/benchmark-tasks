"""Robot-actions-only physical evaluation. No object writes or attachments."""
import os,json,argparse,math,hashlib
from pathlib import Path
os.environ.pop('UNPACK_PRESENTATION',None)
import numpy as np
import imageio.v2 as imageio
from inspect_robots import Scene
from inspect_robots.types import Action
from inspect_robots_yam.packing import pack
from yam_arena import ik,station
from yam_arena.constants import READY,ARM_JOINTS,FINGER_JOINTS,FINGER_OPEN
from yam_arena.demo import grip_rotation
from yam_arena.urdf import axis_angle_matrix
from yam_arena.inspect_embodiment import yam_arena_embodiment
p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--mode',default='negative_replay');p.add_argument('--reference',type=Path,default=Path('runs/demo'));a=p.parse_args()
out=a.output;out.mkdir(parents=True,exist_ok=False)
source_hashes={n:hashlib.sha256(Path(__file__).with_name(n).read_bytes()).hexdigest()for n in ('task.py','partkit.py','score_logic.py','physical_demo.py')}
(out/'source_hashes.json').write_text(json.dumps(source_hashes,indent=2)+'\n')
emb=yam_arena_embodiment(task=str(Path(__file__).with_name('task.py').resolve()),object_jitter=0,control_hz=20,cam_width=320,cam_height=176,top_cam='demo')
t=emb.task.module;obs=emb.reset(Scene(id='unpack_physical_'+a.mode,instruction=t.INSTRUCTION))
import torch
from yam_arena.scene import object_states
base=emb._env.unwrapped  # Camera is configured by task reset for every policy.
q={s:np.array(READY,float)for s in ('left','right')};tcp={};rot={};grip={s:1. for s in q}
for s in q:tcp[s],rot[s]=ik.tcp_pose(q[s],station.BASES[s])
trace=[];frames=[];worst=0.
class AbortEpisode(Exception):pass
def step(phase):
 global obs,last_step
 last_step=emb.step(Action(data=pack(np.r_[q['left'],grip['left']],np.r_[q['right'],grip['right']])))
 obs=last_step.observation
 st=object_states(emb._env,['instrument','cap','cradle']);actual={}
 for side in q:
  robot=base.scene[side+'_robot'];data=getattr(robot.data.joint_pos,'torch',robot.data.joint_pos)[0];aq=data[[robot.joint_names.index(n)for n in ARM_JOINTS]].detach().cpu().numpy();pt,_=ik.tcp_pose(aq,station.BASES[side]);actual[side]={'tcp':pt.tolist(),'opening':float(data[[robot.joint_names.index(n)for n in FINGER_JOINTS]].mean().item()/FINGER_OPEN)}
 trace.append({'step':len(trace),'phase':phase,'physical_score':t.score(None),'state':dict(t.STATE),'objects':{n:{'pos':o.pos.tolist(),'quat':o.quat_xyzw.tolist(),'linear_velocity':o.lin_vel.tolist(),'angular_velocity':o.ang_vel.tolist()}for n,o in st.items()},'robot':actual,'action':np.r_[q['left'],grip['left'],q['right'],grip['right']].tolist()})
 if len(trace)%3==0:frames.append(np.asarray(obs.images['top_cam']))
def hold(n,phase):
 for _ in range(n):step(phase)
def shot(name):imageio.imwrite(out/(name+'.png'),np.asarray(obs.images['top_cam']))

# Replay recorded robot actions only. Never restore or write any object state.
expected=json.loads((a.reference/'source_hashes.json').read_text())
assert source_hashes==expected,'Frozen sources changed'
rows=[json.loads(line)for line in(a.reference/'trace.jsonl').read_text().splitlines()]
end=max(i for i,r in enumerate(rows)if r['phase']=='instrument_lift_hold')
for r in rows[:end+1]:
 action=np.array(r['action']);q['left']=action[:6];grip['left']=action[6];q['right']=action[7:13];grip['right']=action[13]
 step(r['phase'])
 if len(trace)==25:shot('setup')
 if len(trace)%100==0:print(json.dumps({'step':len(trace),'phase':r['phase'],'invalid':t.STATE['invalid']}),flush=True)
shot('before_release')
z0=trace[0]['objects']['instrument']['pos'][2];release_z=trace[-1]['objects']['instrument']['pos'][2]
held=release_z-z0>.10 and t.STATE['grasp_support'] and not t.STATE['invalid']
if held:
 initial_opening=grip['left']
 for i in range(1,16):grip['left']=initial_opening+(1-initial_opening)*i/15;step('intentional_release')
 shot('release')
 hold(100,'free_settle');shot('final')
first_invalid=next(({'step':r['step'],'phase':r['phase'],'reason':r['state']['reason']}for r in trace if r['state']['invalid']),None)
post=trace[end+1:]
result={'mode':'negative_replay','physical_score':t.score(None),'success':t.success(None),'steps':len(trace),'robot_actions_only':True,'prefix_steps':end+1,'prefix_source':str(a.reference),'held_before_release':bool(held),'actual_lift_before_release_m':release_z-z0,'max_post_release_descent_m':None if not post else release_z-min(r['objects']['instrument']['pos'][2]for r in post),'max_tilt_deg':max(r['state']['tilt']for r in trace),'first_invalid':first_invalid,'state':dict(t.STATE),'source_hashes':source_hashes}
(out/'result.json').write_text(json.dumps(result,indent=2)+'\n');(out/'trace.jsonl').write_text(''.join(json.dumps(r)+'\n'for r in trace));imageio.mimsave(out/'physical.mp4',frames,fps=20/3,codec='libx264',output_params=['-pix_fmt','yuv420p','-movflags','+faststart']);print(json.dumps(result),flush=True);emb.close()
