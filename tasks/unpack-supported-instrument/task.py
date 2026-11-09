"""Rigid shipping support prototype; scoring still requires physical validation."""
import sys,pathlib,math
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
from partkit import part,on_step,on_reset
from yam_arena.task_file import TABLE_TOP
SCENE='station';SCORE_OVER='end';TOP=float(TABLE_TOP[SCENE])
INSTRUCTION='Unpack the instrument onto its service stand and put the shipping foam in the packing tray, keeping the instrument supported throughout.'
FOAM=(.28,.31,.32);BODY=(.86,.87,.83);DARK=(.055,.075,.085);METAL=(.42,.49,.52)
SOURCE=(.34,.09);DEST=(.34,-.17);TRAY=(.56,.10)
def b(x,y,z,sx,sy,sz,c):return ((float(x),float(y),float(z)),(float(sx),float(sy),float(sz)),c)
# Fine shallow bead geometry makes the shared foam material legible without hazard colors.
def foam_grain(boxes):
 out=[]
 for centre,size,col in boxes:
  x,y,z=centre;sx,sy,sz=size
  if sx<.035 or sy<.03:continue
  for xx in np.arange(x-sx/2+.006,x+sx/2-.003,.009):
   for yy in np.arange(y-sy/2+.006,y+sy/2-.003,.009):
    v=.017*math.sin(float(xx)*183+float(yy)*231)
    out.append(((float(xx),float(yy),float(z+sz/2+.00018)),.0014,.00055,tuple(float(c+v) for c in FOAM),'Z'))
 return out
cradle_boxes=[b(0,0,.02,.18,.13,.04,FOAM),b(-.067,0,.07,.025,.142,.06,FOAM),b(.086,0,.07,.025,.142,.06,FOAM),b(.0095,-.058,.065,.13,.026,.05,FOAM),b(.0095,.058,.065,.13,.026,.05,FOAM),b(-.09,0,.06,.018,.028,.04,FOAM)]
# Subtle horizontal foam layer seams; cap and cradle use identical texture/color.
for yy in (-.0651,.0651):
 cradle_boxes.extend(b(0,yy,z,.171,.0008,.0012,(.21,.24,.25))for z in(.012,.022,.032))
CRADLE=part('cradle',(*SOURCE,TOP),mass=.12,boxes=cradle_boxes,cylinders=foam_grain(cradle_boxes))
body=[b(0,0,.008,.045,.042,.016,METAL),b(-.020,0,.071,.030,.049,.126,BODY),b(-.017,0,.127,.040,.054,.035,BODY),b(.024,0,.143,.105,.061,.027,BODY),b(.024,0,.128,.103,.061,.004,(.19,.28,.31)),b(.030,0,.044,.082,.082,.008,DARK),b(.030,0,.039,.072,.070,.003,METAL),b(.030,0,.049,.047,.025,.0018,(.55,.71,.73)),b(.031,-.021,.051,.05,.003,.002,METAL),b(.031,.021,.051,.05,.003,.002,METAL),b(-.060,-.032,.139,.018,.016,.10,DARK),b(-.060,.032,.139,.018,.016,.10,DARK),b(-.060,0,.184,.018,.08,.018,DARK)]
# Stage graduations and grippy carry handle; no decorative answer cues.
for x in np.linspace(.001,.060,10):body.append(b(float(x),-.035,.0485,.0007,.004,.0007,(.73,.77,.76)))
for yy in np.linspace(-.022,.022,8):body.append(b(-.070, float(yy),.184,.001,.0018,.016,(.18,.22,.23)))
optics=[((-.018,-.038,.085),.021,.018,DARK,'Y'),((-.018,-.049,.085),.013,.006,METAL,'Y'),((-.018,.038,.085),.021,.018,DARK,'Y'),((-.018,.049,.085),.013,.006,METAL,'Y'),((.052,0,.121),.022,.010,METAL,'Z'),((.060,0,.101),.011,.030,DARK,'Z'),((.060,0,.084),.008,.006,METAL,'Z'),((.033,-.013,.107),.007,.020,METAL,'Z'),((.033,.013,.109),.007,.016,METAL,'Z')]
for yy in (-.018,.018):
 optics += [((-.009,yy,.166),.012,.026,METAL,'Z'),((-.009,yy,.181),.015,.008,DARK,'Z'),((-.009,yy,.1855),.010,.001,(.10,.21,.26),'Z')]
# Radial flutes make the large coarse-focus knobs recognizable at scene scale.
for side in (-1.,1.):
 for k in range(16):
  ang=k*math.pi/8
  body.append(b(-.018+.0205*math.cos(ang),side*.039,.085+.0205*math.sin(ang),.002,.019,.002,(.17,.21,.22)))
INSTRUMENT=part('instrument',(*SOURCE,TOP+.04),mass=.35,friction=1.1,boxes=body,cylinders=optics)
cap_boxes=[b(.02,0,0,.14,.10,.025,FOAM),b(.02,-.048,-.027,.14,.014,.04,FOAM),b(.02,.048,-.027,.14,.014,.04,FOAM),b(.02,0,.024,.025,.027,.026,FOAM)]
for yy in (-.018,.018):
 cap_boxes.extend([b(.055,yy,-.026,.026,.012,.04,FOAM),b(-.009,yy,-.015,.024,.022,.005,FOAM)])
for yy in (-.0551,.0551):cap_boxes.append(b(.02,yy,-.01,.135,.001,.0012,(.21,.24,.25)))
CAP=part('cap',(*SOURCE,TOP+.243),mass=.04,boxes=cap_boxes,cylinders=foam_grain(cap_boxes))
stand_boxes=[b(0,0,.008,.18,.15,.016,(.18,.26,.29)),b(0,0,.020,.16,.13,.008,METAL),b(0,0,.032,.11,.09,.016,(.69,.73,.72)),b(-.05,0,.060,.015,.09,.04,METAL),b(.05,0,.060,.015,.09,.04,METAL),b(0,-.04,.057,.085,.012,.034,METAL),b(0,.04,.057,.085,.012,.034,METAL),b(0,-.061,.025,.065,.009,.001,(.70,.74,.73))]
stand_cyl=[]
for x in(-.072,.072):
 for y in(-.055,.055):
  stand_cyl += [((x,y,.003),.012,.006,DARK,'Z'),((x,y,.0248),.0035,.0015,DARK,'Z')]
STAND=part('stand',(*DEST,TOP),mass=1.,fixed_to_world=True,boxes=stand_boxes,cylinders=stand_cyl)
tray_boxes=[b(0,0,.004,.24,.30,.008,(.55,.41,.28)),b(0,0,.0085,.222,.282,.001,(.66,.53,.38)),b(-.117,0,.02,.006,.30,.032,(.44,.32,.21)),b(.117,0,.02,.006,.30,.032,(.44,.32,.21)),b(0,-.147,.02,.24,.006,.032,(.44,.32,.21)),b(0,.147,.02,.24,.006,.032,(.44,.32,.21))]
# Two recessed-looking seams describe the packing tray without labels or clutter.
for yy in (-.138,.138):tray_boxes.append(b(0,yy,.0092,.222,.001,.0005,(.45,.34,.23)))
PACKING=part('packing_tray',(*TRAY,TOP),mass=1.,fixed_to_world=True,boxes=tray_boxes)
OBJECTS=[CRADLE,INSTRUMENT,CAP,STAND,PACKING]
from score_logic import History
STATE={};HISTORY=History();LAST_HANDLE_REL={}
FOOT=np.array([[x,y,0.]for x in(-.0225,.0225)for y in(-.021,.021)])
def geometry_points(spec):
 result=[]
 for box in spec.boxes:
  centre,size=box[:2];yaw=box[3]if len(box)>3 else 0.;tilt=box[4]if len(box)>4 else 0.
  cy,sy,ct,st=math.cos(yaw),math.sin(yaw),math.cos(tilt),math.sin(tilt)
  R=np.array([[cy,-sy*ct,sy*st],[sy,cy*ct,-cy*st],[0.,st,ct]])
  for x in(-.5,.5):
   for y in(-.5,.5):
    for z in(-.5,.5):result.append(np.array(centre)+R@(np.array(size)*[x,y,z]))
 for cyl in spec.cylinders:
  centre,r,h=cyl[:3];axis=cyl[4]if len(cyl)>4 else 'Z';half=np.array([h/2 if axis==a else r for a in 'XYZ'])
  for x in(-1.,1.):
   for y in(-1.,1.):
    for z in(-1.,1.):result.append(np.array(centre)+half*[x,y,z])
 return np.array(result)
FOAM_POINTS={p.name:geometry_points(p)for p in(CAP,CRADLE)}
@on_reset
def reset(environment=None):
 global HISTORY
 HISTORY=History();LAST_HANDLE_REL.clear();STATE.clear();STATE.update(invalid=False,score=0.,tilt=0.)
 if environment is not None:
  import torch
  base=environment.unwrapped
  camera=base.scene['top_cam']
  camera.set_world_poses_from_view(torch.tensor([[.71,-.38,TOP+.60]],device=base.device),torch.tensor([[.40,.03,TOP+.105]],device=base.device))
  for _ in range(3):base.sim.render()
  camera.update(0.,force_recompute=True)
reset()
def grippers(environment):
 from yam_arena import ik,station
 from yam_arena.constants import ARM_JOINTS,FINGER_JOINTS,FINGER_OPEN
 result=[]
 for side in('left','right'):
  robot=environment.unwrapped.scene[side+'_robot'];q=getattr(robot.data.joint_pos,'torch',robot.data.joint_pos)[0]
  qs=q[[robot.joint_names.index(n)for n in ARM_JOINTS]].detach().cpu().numpy();tcp,R=ik.tcp_pose(qs,station.BASES[side])
  opening=float(q[[robot.joint_names.index(n)for n in FINGER_JOINTS]].mean().item()/FINGER_OPEN)
  result.append((side,tcp,R,opening))
 return result
@on_step
def monitor(environment):
 from yam_arena.scene import object_states
 st=object_states(environment,['instrument','cap','cradle','stand']);o=st['instrument'];gs=grippers(environment)
 tilt=math.degrees(math.acos(float(np.clip(o.rotation()[2,2],-1.,1.))))
 foot=o.pos+FOOT@o.rotation().T
 def on_deck(name,half):
  deck=st[name];local=(foot-deck.pos)@deck.rotation()
  return bool(np.all(np.abs(local[:,:2])<=np.asarray(half)+.003)and .037<=np.min(local[:,2])<=.043 and np.max(local[:,2])<=.052 and deck.rotation()[2,2]>.98)
 cradle_support=on_deck('cradle',(.09,.065));stand_support=on_deck('stand',(.055,.045))
 handle=o.to_world((-.06,0.,.184));grasp=False
 for side,tcp,R,opening in gs:
  rel=R.T@(handle-tcp);prev=LAST_HANDLE_REL.get(side)
  if opening<.38 and np.linalg.norm(rel)<.018 and prev is not None and np.linalg.norm(rel-prev)<.006:grasp=True
  LAST_HANDLE_REL[side]=rel
 stand_ok=bool(stand_support and np.linalg.norm(o.pos[:2]-DEST)<.025 and tilt<10 and np.linalg.norm(o.lin_vel)<.015 and np.linalg.norm(o.ang_vel)<.10)
 contained={};foam_stable=True
 for name in('cap','cradle'):
  part=st[name];pts=part.pos+FOAM_POINTS[name]@part.rotation().T;lo,hi=pts.min(0),pts.max(0)
  contained[name]=bool(lo[0]>=TRAY[0]-.114-0.001 and hi[0]<=TRAY[0]+.114+.001 and lo[1]>=TRAY[1]-.144-.001 and hi[1]<=TRAY[1]+.144+.001 and lo[2]>=TOP+.005 and hi[2]<=TOP+.23)
  foam_stable &= bool(np.linalg.norm(part.lin_vel)<.02 and np.linalg.norm(part.ang_vel)<.12)
 released=all(min(np.linalg.norm(tcp-handle),np.linalg.norm(tcp-st['cap'].to_world((.02,0,.026))),np.linalg.norm(tcp-st['cradle'].to_world((-.09,0,.062))))>.065 for side,tcp,R,opening in gs)
 values=HISTORY.update(z=float(o.pos[2]),tilt_deg=tilt,surface_support=cradle_support or stand_support,grasp_support=grasp,stand_ok=stand_ok,packing_ok=all(contained.values())and foam_stable,released=released,dt=float(environment.unwrapped.step_dt))
 STATE.clear();STATE.update(values,tilt=tilt,cradle_support=cradle_support,stand_support=stand_support,grasp_support=grasp,contained=contained,released=bool(released))
def score(states):return float(STATE['score'])
def success(states):return score(states)==1.
