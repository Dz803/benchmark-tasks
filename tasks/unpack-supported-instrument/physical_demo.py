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
p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--mode',default='probe',choices=['probe','demo','idle','drop']);a=p.parse_args()
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
def move(phase,p,g,R,n=35):
 global worst
 s='left';p0,r0,g0=tcp[s].copy(),rot[s].copy(),grip[s]
 for i in range(1,n+1):
  u=ik.smoothstep(i/n);pt=p0+u*(np.asarray(p)-p0);rt=ik.slerp_matrix(r0,R,u)
  q[s],e,re=ik.solve(pt,ik.matrix_to_quat_xyzw(rt),q_init=q[s],base_pos=station.BASES[s],iters=100);worst=max(worst,float(e));tcp[s],rot[s],grip[s]=pt,rt,g0+u*(g-g0);step(phase)
 print(json.dumps({'phase':phase,'state':t.STATE,'objects':trace[-1]['objects'],'left':trace[-1]['robot']['left']}),flush=True);shot(phase)
 if t.STATE['invalid'] and a.mode!='drop':raise AbortEpisode('irreversible scoring failure in '+phase)
def current(name):return object_states(emb._env,[name])[name]
def grasp(name,handle,tilt,yaw):
 R=axis_angle_matrix(np.array([0.,1.,0.]),math.radians(-tilt))@grip_rotation(math.radians(yaw));away=-R[:,2]*.075;p=current(name).to_world(handle)
 move('approach_'+name,p+away,1.,R,45);move('align_'+name,p,1.,R,40);move('grasp_'+name,p,0.,R,30);hold(15,'grip_settle_'+name);shot('grasped_'+name)
 return R,p,away
def transfer(name,handle,destination,tilt,yaw,lift):
 R,p,away=grasp(name,handle,tilt,yaw);d=np.array(destination)+np.array(handle)
 move('lift_'+name,p+[0,0,lift],0.,R,45);hold(15,'lift_hold_'+name)
 if name=='cap':
  high=tcp['left'][2]
  move('cap_clear_instrument_side',(tcp['left'][0],.235,high),0.,R,45)
  move('cap_lower_in_clear_corridor',(tcp['left'][0],.235,t.TOP+.16),0.,R,40)
  move('carry_cap',(d[0],d[1],t.TOP+.16),0.,R,55)
 elif name=='cradle':
  # Clear the already packed cap with the entire carried cradle, not only TCP.
  move('cradle_front_corridor',(tcp['left'][0],-.03,tcp['left'][2]),0.,R,40)
  move('cradle_across_front',(d[0],-.03,d[2]+lift),0.,R,55)
  move('carry_cradle',d+[0,0,lift],0.,R,40)
 else:move('carry_'+name,d+[0,0,lift],0.,R,60)
 # Full-01 showed floor compression sliding the held cradle outside the tray.
 placement_clearance=.015 if name=='cradle' else .003
 move('lower_'+name,d+[0,0,placement_clearance],0.,R,45);move('release_'+name,tcp['left'],1.,R,25);move('retreat_'+name,tcp['left']+away,1.,R,35);hold(25,'settle_'+name)
hold(25,'initial_idle');shot('setup')
abort_reason=None
try:
 if a.mode=='idle':hold(100,'idle')
 elif a.mode in('probe','drop'):
  R,p,away=grasp('instrument',(-.06,0,.184),90,90)
  move('lift_instrument',p+[0,0,.075],0.,R,50);hold(35,'lift_hold');shot('lift_hold')
  if a.mode=='drop':move('drop_release',tcp['left'],1.,R,15);move('drop_retreat',tcp['left']+away,1.,R,30);hold(45,'drop_settle')
 else:
  transfer('cap',(.02,0,.026),(t.TRAY[0],t.TRAY[1]+.075,t.TOP+.055),60,0,.05)
  Rcap=axis_angle_matrix(np.array([0.,1.,0.]),-math.pi/3)@grip_rotation()
  Rinst=axis_angle_matrix(np.array([0.,1.,0.]),-math.pi/2)@grip_rotation(math.pi/2)
  move('cap_clear_outboard',(tcp['left'][0],.30,tcp['left'][2]),1.,Rcap,40)
  move('cap_to_high_side',(.36,.30,t.TOP+.27),1.,Rcap,40)
  move('side_to_back',(.18,.235,t.TOP+.25),1.,Rcap,40)
  move('orient_in_empty_back',(.18,.235,t.TOP+.224),1.,Rinst,40)
  move('back_to_instrument_approach',(.205,.082,t.TOP+.224),1.,Rinst,40)
  transfer('instrument',(-.06,0,.184),(*t.DEST,t.TOP+.04),90,90,.11)
  move('stand_to_empty_side',(.18,.235,t.TOP+.227),1.,Rinst,40)
  move('orient_for_cradle',(.18,.235,t.TOP+.16),1.,Rcap,40)
  move('back_to_cradle_approach',(.185,.09,t.TOP+.10),1.,Rcap,40)
  transfer('cradle',(-.09,0,.062),(t.TRAY[0],t.TRAY[1]-.065,t.TOP+.009),60,0,.075)
except AbortEpisode as exc:abort_reason=str(exc)
hold(25,'final');shot('final')
z0=trace[0]['objects']['instrument']['pos'][2];result={'mode':a.mode,'physical_score':t.score(None),'success':t.success(None),'terminated':bool(last_step.terminated),'termination_reason':last_step.termination_reason,'steps':len(trace),'worst_ik_error_m':worst,'instrument_max_lift_m':max(r['objects']['instrument']['pos'][2]for r in trace)-z0,'instrument_final_lift_m':trace[-1]['objects']['instrument']['pos'][2]-z0,'state':t.STATE,'robot_actions_only':True,'abort_reason':abort_reason,'source_hashes':source_hashes}
(out/'result.json').write_text(json.dumps(result,indent=2)+'\n');(out/'trace.jsonl').write_text(''.join(json.dumps(r)+'\n'for r in trace));imageio.mimsave(out/'physical.mp4',frames,fps=20/3,codec='libx264',output_params=['-pix_fmt','yuv420p','-movflags','+faststart']);print(json.dumps(result),flush=True);emb.close()
