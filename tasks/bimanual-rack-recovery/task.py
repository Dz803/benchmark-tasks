"""Physical sample carrier with independent tubes and passive grip-bar hinges."""
import pathlib,sys
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
from partkit import part
from yam_arena.task_file import TABLE_TOP
from scene_constants import SOURCE,DEST,BASE,HANDLE,SPAN,SOURCE_FRONT
SCENE='station';SCORE_OVER='end';TOP=TABLE_TOP[SCENE]
INSTRUCTION='Move the sample carrier onto the receiving stand, keeping the samples upright and contained. If it catches, stop and free it without forcing or tipping it, then complete the transfer.'
def b(p,s,c):return (tuple(map(float,p)),tuple(map(float,s)),c)
STEEL=(.55,.62,.65);BLUE=(.13,.48,.66);DARK=(.1,.18,.23);WHITE=(.88,.91,.9);ORANGE=(.95,.47,.12)
OBJECTS=[]
for name,x in [('source',SOURCE),('receiver',DEST)]:
    # Low stand, open in the transfer direction; no rail blocks recovery slide.
    OBJECTS.append(part(name,(float(x),0.,TOP),boxes=[b((0,0,.008),(.14,.27,.016),DARK)]+([b(((SOURCE_FRONT-(SOURCE+.07))/2,-.075,.025),(SOURCE_FRONT-(SOURCE-.07),.018,.018),(.86,.88,.79)),b(((SOURCE_FRONT-(SOURCE+.07))/2,.075,.025),(SOURCE_FRONT-(SOURCE-.07),.018,.018),(.86,.88,.79))] if name=='source' else [b((-.04,0,.025),(.018,.20,.018),STEEL),b((.04,0,.025),(.018,.20,.018),STEEL)]),mass=.5,friction=.10 if name=='source' else .8,fixed_to_world=True))
# Lip on positive-Y side catches ear under its horizontal cap. No motor fault.
OBJECTS.append(part('retaining_lip',(SOURCE-.048,.145,TOP),boxes=[b((0,.040,.045),(.026,.016,.090),ORANGE),b((0,.014,.060),(.026,.068,.008),ORANGE)],mass=.2,fixed_to_world=True))
body=[b((0,0,0),(.12,.24,.008),STEEL),b((-.058,0,.018),(.006,.24,.036),BLUE),b((.058,0,.018),(.006,.24,.036),BLUE)]
for s in [-1,1]:
    body += [b((0,s*.118,.018),(.12,.006,.036),BLUE),b((-.048 if s==1 else 0,s*.132,.010),(.028,.026,.008),STEEL)]
for s in [-1,1]:
    for x in [-.027,.027]:body.append(b((x,s*((.118+SPAN)/2),.018),(.007,SPAN-.118+.005,.009),STEEL))
# Independent freely rotating grip bars: revolute around X, no arm attachment.
OBJECTS.append(part('carrier',(SOURCE,0.,TOP+BASE),boxes=body,mass=.18,friction=1.))
for side,sgn in [('left',1),('right',-1)]:
    OBJECTS.append(part('handle_'+side,(SOURCE,float(sgn*SPAN),TOP+BASE+HANDLE),boxes=[],cylinders=[((0.,0.,0.),.008,.064,BLUE,'X')],mass=.015,friction=1.5,hinge_attachment=('carrier',(0.,float(sgn*SPAN),HANDLE),(0.,0.,0.),95.,'X',-95.)))
for i,(x,y) in enumerate([(x,y)for x in [-.027,.027]for y in [-.055,.055]]):
    OBJECTS.append(part('sample_'+str(i),(SOURCE+x,y,TOP+BASE+.044),boxes=[],cylinders=[((0.,0.,0.),.015,.072,WHITE,'Z'),((0.,0.,.041),.017,.010,(.28,.68,.65),'Z')],mass=.018,friction=.8))
from partkit import on_reset,on_step
import math,numpy as np
STATE={}
@on_reset
def reset(environment=None):
    STATE.clear();STATE.update(invalid=False,max_tilt_deg=0.,tilt_deg=0.,contained=True,clear=False,stable_s=0.,score=0.)
    if environment is not None:
        import torch
        base=environment.unwrapped
        base.scene['top_cam'].set_world_poses_from_view(torch.tensor([[.72,.25,TOP+.58]],device=base.device),torch.tensor([[.33,0.,TOP+.05]],device=base.device))
@on_step
def monitor(environment):
    from yam_arena.scene import object_states
    from yam_arena import ik,station
    from yam_arena.constants import ARM_JOINTS,FINGER_JOINTS,FINGER_OPEN
    st=object_states(environment,['carrier','handle_left','handle_right']+['sample_'+str(i)for i in range(4)])
    c=st['carrier'];R=c.rotation();tilt=math.degrees(math.acos(float(np.clip(R[2,2],-1,1))))
    contained=True;sample_bounds=[];sample_tilts=[]
    for i in range(4):
        tube=st['sample_'+str(i)];bottom=c.to_local(tube.to_world((0.,0.,-.036)))
        rim=np.array([c.to_local(tube.to_world((.015*math.cos(a),.015*math.sin(a),-.036)))for a in np.linspace(0,2*math.pi,16,endpoint=False)])
        ok=bool(np.all(np.abs(rim[:,0])<=.060) and np.all(np.abs(rim[:,1])<=.120) and np.all(rim[:,2]>=-.012) and np.all(rim[:,2]<=.045))
        sample_tilts.append(math.degrees(math.acos(float(np.clip(tube.rotation()[2,2],-1,1)))))
        contained=contained and bool(ok);sample_bounds.append(bottom.tolist())
    STATE['invalid']=STATE['invalid'] or tilt>12. or not contained
    STATE.update(tilt_deg=tilt,max_tilt_deg=max(STATE['max_tilt_deg'],tilt),contained=contained,sample_bottoms=sample_bounds,max_sample_tilt_deg=max(sample_tilts),samples_upright=max(sample_tilts)<=10.)
    corners=[c.to_world((x,y,z))for x in(-.06,.06)for y in(-.12,.12)for z in(-.004,.036)]
    clear=min(p[0]for p in corners)>SOURCE_FRONT
    released=True
    for side in ('left','right'):
        rob=environment.unwrapped.scene[side+'_robot'];joint=getattr(rob.data.joint_pos,'torch',rob.data.joint_pos)[0]
        q=joint[[rob.joint_names.index(n)for n in ARM_JOINTS]].detach().cpu().numpy();tcp,_=ik.tcp_pose(q,np.array(station.BASES[side]));op=float(joint[[rob.joint_names.index(n)for n in FINGER_JOINTS]].mean().item()/FINGER_OPEN)
        released=released and op>.75 and np.linalg.norm(tcp-st['handle_'+side].pos)>.03
    angle=math.degrees(math.acos(float(np.clip((np.trace(R)-1)/2,-1,1))))
    seated=np.linalg.norm(c.pos-np.array([DEST,0.,TOP+BASE]))<=.010 and angle<=5.
    stable=seated and released and STATE['samples_upright'] and np.linalg.norm(c.lin_vel)<.015 and np.linalg.norm(c.ang_vel)<.15
    STATE['stable_s']=STATE['stable_s']+float(environment.unwrapped.step_dt)if stable else 0.
    STATE.update(clear=bool(clear),seated=bool(seated),released=bool(released),carrier_pos=c.pos.tolist(),score=0. if STATE['invalid'] else 1. if STATE['stable_s']>=1. else .5 if clear and tilt<=5. and contained and STATE['samples_upright'] else 0.)
def score(_):return STATE.get('score',0.)
def success(_):return score(_)==1.
reset()
