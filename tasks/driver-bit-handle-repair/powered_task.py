"""Integrated prototype: physical chuck, grip interlock, pressure start, threaded carriage.
The thread is an ideal rack/pinion constraint (0.7 mm/rev), not resolved thread contact.
"""
from gap_assembly_task import *
import numpy as np
from dataclasses import replace

INSTRUCTION='Select and fit the matching bit, hold the drawer, tighten its loose handle, then return the stopped driver to its stand.'
# A guided mounting carriage represents the handle nut, with 1.5 mm initial slack.
# Both posts translate together; the second visible fastener is a guide, not a fixed clamp.
parts=[]
for o in OBJECTS:
    if o.name=='driver':
        extra=(Slider('pressure',(0,0,0),(),travel=.004,rate=600.,damping=2.,mass=.02,axis='Z',sign=-1),
               Slider('trigger',(.023,0,.050),(((.023,0,.050),(.006,.015,.020),ORANGE),),travel=.004,rate=100.,damping=.2,mass=.002,axis='X',sign=-1))
        o=replace(o,sliders=tuple(o.sliders)+extra)
    elif o.name=='drawer_context':
        # Preserve indices consumed by upstream geometry wrappers, remove fused handle at spawn.
        hb=(((FASTENER_X,0,.0485),(.018,.146,.012),DARK),
            ((FASTENER_X,-.06,.066),(.017,.017,.025),DARK),
            ((FASTENER_X,.06,.066),(.017,.017,.025),DARK))
        o=replace(o,sliders=(Slider('handle',(0,0,0),hb,travel=.0015,rate=200.,damping=1.,mass=.05,axis='Z',sign=1),))
    parts.append(o)
OBJECTS=parts
_prev_spawn=partkit._spawn_part

def powered_spawn(path,cfg,*args,**kwargs):
    result=_prev_spawn(path,cfg,*args,**kwargs)
    if cfg.part_name not in ('driver','drawer_context','bit'):return result
    from pxr import Gf,UsdGeom,UsdPhysics,PhysxSchema,Sdf
    from isaaclab.sim.utils.stage import get_current_stage
    stage=get_current_stage();layer=stage.GetRootLayer()
    if cfg.part_name=='bit':
        from isaaclab.sim.utils import bind_physics_material
        for i in (1,2):stage.RemovePrim(path+f'/body/box_{i}')
        for i in (0,1):
            points=[]
            for z,length,width in ((.0925,.009,.002),(.0955,.007,.0012)):
                for x,y in ((-length/2,-width/2),(length/2,-width/2),(length/2,width/2),(-length/2,width/2)):
                    points.append(Gf.Vec3f(y if i else x,x if i else y,z))
            faces=[[3,2,1,0],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]]
            if i:faces=[list(reversed(f)) for f in faces]
            m=UsdGeom.Mesh.Define(stage,path+f'/body/tapered_cross_{i}')
            m.CreatePointsAttr(points);m.CreateFaceVertexCountsAttr([4]*6);m.CreateFaceVertexIndicesAttr([v for f in faces for v in f]);m.CreateSubdivisionSchemeAttr('none');m.CreateDisplayColorAttr([Gf.Vec3f(*GREY)])
            UsdPhysics.CollisionAPI.Apply(m.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull')
            pc=PhysxSchema.PhysxCollisionAPI.Apply(m.GetPrim());pc.CreateContactOffsetAttr(.0001);pc.CreateRestOffsetAttr(0.)
            bind_physics_material(str(m.GetPath()),path+'/Looks/physics',stage=stage)
        return result

    def body(name,mass,inertia):
        bp=path+'/'+name
        g=UsdGeom.Xform.Define(stage,bp)
        UsdPhysics.RigidBodyAPI.Apply(g.GetPrim())
        ma=UsdPhysics.MassAPI.Apply(g.GetPrim());ma.CreateMassAttr(mass);ma.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        ph=PhysxSchema.PhysxRigidBodyAPI.Apply(g.GetPrim());ph.CreateAngularDampingAttr(.1)
        ph.CreateSolverPositionIterationCountAttr(128);ph.CreateSolverVelocityIterationCountAttr(4)
        return bp
    def transfer(name,dest):
        src=path+'/body/'+name
        Sdf.CopySpec(layer,src,layer,dest+'/'+name);stage.RemovePrim(src)
    def hinge(name,parent,child,anchor):
        j=UsdPhysics.RevoluteJoint.Define(stage,path+'/'+name)
        j.CreateAxisAttr('Z');j.CreateBody0Rel().SetTargets([Sdf.Path(parent)]);j.CreateBody1Rel().SetTargets([Sdf.Path(child)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*anchor));j.CreateLocalPos1Attr(Gf.Vec3f(*anchor))
        return j
    if cfg.part_name=='driver':
        spindle=body('spindle',.04,(1e-5,1e-5,2e-5))
        j=hinge('motor',path+'/pressure',spindle,(0,0,.11))
        d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular');d.CreateStiffnessAttr(0.);d.CreateDampingAttr(.02);d.CreateMaxForceAttr(.025)
        for k in range(4):transfer('box_'+str(k),spindle)
        transfer('cyl_3',spindle)
        floor=UsdGeom.Cylinder(stage.GetPrimAtPath(spindle+'/cyl_3'))
        for op in floor.GetOrderedXformOps():
            if op.GetOpType()==UsdGeom.XformOp.TypeTranslate:op.Set(Gf.Vec3d(0,0,.1007))
        # Fixed housing ends behind the 4 mm moving stroke; no parallel axial load path.
        housing=UsdGeom.Cylinder(stage.GetPrimAtPath(path+'/body/cyl_0'))
        housing.GetHeightAttr().Set(.09)
        for op in housing.GetOrderedXformOps():
            if op.GetOpType()==UsdGeom.XformOp.TypeTranslate:op.Set(Gf.Vec3d(0,0,.045))
        for k in range(6):transfer('lead_in_'+str(k),spindle)
        # 6.45 mm across-flats, 30 mm long machined hex guide; full support prevents tip orbit.
        for i,k in enumerate((1,2,4,5)):
            a=k*math.pi/3;g=UsdGeom.Cube(stage.GetPrimAtPath(spindle+f'/box_{i}'))
            for op in g.GetOrderedXformOps():
                if op.GetOpType()==UsdGeom.XformOp.TypeTranslate:op.Set(Gf.Vec3d(.004225*math.cos(a),.004225*math.sin(a),.115))
                if op.GetOpType()==UsdGeom.XformOp.TypeScale:op.Set(Gf.Vec3f(.002,.006,.030))
        # The two jaw faces have guide sections above and below their moving pad windows.
        for sign in (-1,1):
            for z,h in ((.103,.006),(.121,.018)):
                dest=spindle+f'/guide_{sign+1}_{int(z*1000)}'
                Sdf.CopySpec(layer,spindle+'/box_0',layer,dest)
                g=UsdGeom.Cube(stage.GetPrimAtPath(dest))
                for op in g.GetOrderedXformOps():
                    if op.GetOpType()==UsdGeom.XformOp.TypeTranslate:op.Set(Gf.Vec3d(sign*.004225,0,z))
                    if op.GetOpType()==UsdGeom.XformOp.TypeScale:op.Set(Gf.Vec3f(.002,.006,h))
                    if op.GetOpType()==UsdGeom.XformOp.TypeOrient:op.Set(Gf.Quatf(1,0,0,0))
        for k in range(6):
            mesh=UsdGeom.Mesh(stage.GetPrimAtPath(spindle+f'/lead_in_{k}'))
            pts=mesh.GetPointsAttr().Get();a=k*math.pi/3;n=np.array([math.cos(a),math.sin(a)])
            moved=[]
            for v in pts:
                xy=np.array([v[0],v[1]])
                if v[2]<.125 and np.dot(xy,n)<.005:xy-=n*.000275
                moved.append(Gf.Vec3f(float(xy[0]),float(xy[1]),float(v[2]+.010)))
            mesh.GetPointsAttr().Set(moved)
        for label in ('jaw_plus','jaw_minus'):
            joint=UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(path+'/'+label+'/'+label+'_slide'))
            joint.GetBody0Rel().SetTargets([Sdf.Path(spindle)])
    else:
        for k in (5,6,7):stage.RemovePrim(path+'/body/box_'+str(k))
        screw=body('screw',.012,(1e-6,1e-6,1e-6))
        j=hinge('screw_bearing',path+'/body',screw,(FASTENER_X,-.06,.099))
        d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular');d.CreateStiffnessAttr(0.);d.CreateDampingAttr(1.);d.CreateMaxForceAttr(.012)
        for sx in (0,2):
            for sy in (0,2):transfer(f'round_screw_0_{sx}_{sy}',screw)
        transfer('screw_floor_0',screw)
        j.CreateLowerLimitAttr(-1800.);j.CreateUpperLimitAttr(0.)
        follower=stage.GetPrimAtPath(path+'/handle/handle_slide')
        c=PhysxSchema.PhysxMimicJointAPI.Apply(follower,'rotZ')
        c.CreateReferenceJointRel().SetTargets([j.GetPath()])
        c.CreateGearingAttr(.0007/360.);c.CreateOffsetAttr(0.)
    return result
partkit._spawn_part=powered_spawn
_prev_object=partkit._arena_object
def powered_object(spec):
    obj=_prev_object(spec)
    from isaaclab.actuators import ImplicitActuatorCfg
    if spec.name=='driver':
        obj.object_cfg.actuators['motor']=ImplicitActuatorCfg(joint_names_expr=['motor'],stiffness=0.,damping=.02,effort_limit_sim=.025,velocity_limit_sim=10.)
    elif spec.name=='drawer_context':
        obj.object_cfg.actuators['brake']=ImplicitActuatorCfg(joint_names_expr=['screw_bearing'],stiffness=0.,damping=1.,effort_limit_sim=.012)
    return obj
partkit._arena_object=powered_object
MECHANISM={'motor_enabled':False,'stable_steps':0,'score':0.}
_STILL=[]

def mechanism_state(environment):
    base=environment.unwrapped
    driver=base.scene['driver'];drawer=base.scene['drawer_context']
    def joint(a,n):return float(partkit._np(a.data.joint_pos)[0,a.joint_names.index(n)])
    return dict(pressure_m=-joint(driver,'pressure_slide'),trigger_m=-joint(driver,'trigger_slide'),
                handle_gap_m=.0015-joint(drawer,'handle_slide'),screw_angle_rad=joint(drawer,'screw_bearing'),
                motor_angle_rad=joint(driver,'motor'),
                motor_velocity_rad_s=float(partkit._np(driver.data.joint_vel)[0,driver.joint_names.index('motor')]))
@partkit.on_reset
def reset_power(env):
    MECHANISM.clear();MECHANISM.update(motor_enabled=False,stable_steps=0,score=0.);_STILL.clear()
@partkit.on_step
def update_power(env):
    import torch
    v=mechanism_state(env);on=MECHANISM['motor_enabled']
    if v['pressure_m']>.0015 and v['trigger_m']>.001:on=True
    if v['pressure_m']<.0008 or v['trigger_m']<.0006:on=False
    a=env.unwrapped.scene['driver'];idx=a.joint_names.index('motor')
    a.set_joint_velocity_target(torch.tensor([[np.pi/2 if on else 0.]],device=a.device),joint_ids=[idx])
    MECHANISM.update(v,motor_enabled=on)
    # Final score needs measured closure and a released, stopped tool in the stand.
    st=partkit._part_state(env,next(o for o in OBJECTS if o.name=='driver'))
    home=np.array([.225,-.015,TOP+.067])
    stand=bool(np.linalg.norm(st.pos-home)<.025 and np.dot(st.rotation()[:,2],[0,1,0])>math.cos(math.radians(15)))
    velocity=partkit._np(a.data.root_vel_w)[0]
    speed=np.linalg.norm(velocity[:3]);angular_speed=np.linalg.norm(velocity[3:])
    from yam_arena import ik,station
    from yam_arena.constants import ARM_JOINTS
    hands_clear=True
    grip_center=st.pos+st.rotation()@np.array([0,0,.05])
    for arm in ('left','right'):
        robot=env.unwrapped.scene[arm+'_robot']
        q=partkit._np(robot.data.joint_pos)[0]
        tcp,_=ik.tcp_pose(q[[robot.joint_names.index(n) for n in ARM_JOINTS]],np.array(station.BASES[arm]))
        hands_clear &= np.linalg.norm(tcp-grip_center)>.06
    eligible=v['handle_gap_m']<=.0002 and not on and stand and hands_clear
    MECHANISM.update(tool_in_stand=bool(stand),hands_clear=bool(hands_clear),tool_speed=float(speed),tool_angular_speed=float(angular_speed))
    if eligible:
        _STILL.append((st.pos.copy(),np.asarray(st.quat_xyzw).copy(),v['motor_angle_rad']))
    else:_STILL.clear()
    count=round(1./env.unwrapped.step_dt)+1
    if len(_STILL)>count:del _STILL[:-count]
    stable=False
    if len(_STILL)==count:
        displacement=max(np.linalg.norm(x[0]-_STILL[0][0]) for x in _STILL)
        rotation=max(2*math.acos(min(1.,abs(float(np.dot(x[1],_STILL[0][1]))))) for x in _STILL)
        spindle=np.unwrap([x[2] for x in _STILL])
        excursion=float(np.max(spindle)-np.min(spindle))
        stable=displacement<=.002 and rotation<=math.radians(1.) and excursion<=.02
        MECHANISM.update(rest_displacement_m=float(displacement),rest_rotation_deg=math.degrees(rotation),spindle_excursion_rad=excursion)
    MECHANISM['stable_steps']=len(_STILL)
    MECHANISM['score']=float(stable)
def score(state):return MECHANISM['score']
def success(state):return score(state)==1.
