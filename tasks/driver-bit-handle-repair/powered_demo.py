"""YAM-only pick, hex insert, release and lift probe. No object pose writes."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
import imageio.v2 as imageio

def run(out,lift_only=False,preview_only=False):
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import ik,station
    from yam_arena.constants import READY, ARM_JOINTS, FINGER_JOINTS
    from yam_arena.demo import grip_rotation
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    import yam_arena.inspect_embodiment as ie
    from isaaclab.app import AppLauncher
    if ie._APP is None:
        ie._APP=AppLauncher(headless=True,device='cuda:0',enable_cameras=True).app
        __import__('atexit').register(ie._close_app_at_exit)
    from yam_arena.embodiment import BimanualYamEmbodiment
    previous_init=BimanualYamEmbodiment.__init__
    def instrumented_init(self,*args,**kwargs):
        kwargs['contact_sensors']=True
        previous_init(self,*args,**kwargs)
    BimanualYamEmbodiment.__init__=instrumented_init
    out.mkdir(parents=True,exist_ok=False)
    emb=yam_arena_embodiment(task=str(Path(__file__).with_name('powered_task.py').resolve()),object_jitter=0,control_hz=20,cam_width=640,cam_height=352,top_cam='demo')
    # Presentation/engineering run, explicitly not the official 320x176 policy input.
    from dataclasses import replace
    from scipy.spatial.transform import Rotation
    from yam_arena.rig import Camera,Lens
    eye=np.array(emb.rig.top_cam.pos); target=np.array([.23,-.015,emb.task.module.TOP+.055])
    forward=target-eye;forward/=np.linalg.norm(forward)
    left=np.cross([0.,0.,1.],forward);left/=np.linalg.norm(left)
    up=np.cross(forward,left)
    qq=Rotation.from_matrix(np.column_stack([-left,up,-forward])).as_quat()
    cam=Camera(tuple(eye),tuple(qq),'opengl',Lens(460.,460.,320.,180.,640,360))
    emb.rig=replace(emb.rig,top_cam=cam)
    obs=emb.reset(Scene(id='hex_mount_probe',instruction='Mount bit and lift driver'))
    from yam_arena.scene import object_states
    task=emb.task.module
    imageio.imwrite(out/'scene_overview.png',np.asarray(obs.images['top_cam']))
    for cam,img in obs.images.items(): imageio.imwrite(out/f'initial_{cam}.png',np.asarray(img))
    if preview_only:
        (out/'result.json').write_text(json.dumps({'scope':'scene preview only','physics_pass':None})+'\n')
        return
    q={s:np.array(READY,float) for s in ('left','right')}
    pos,rot,grip={},{},{s:1. for s in q}
    for s in q: pos[s],rot[s]=ik.tcp_pose(q[s],np.array(station.BASES[s]))
    rows=[]; frames=[]; step=0; seated=False
    from pxr import PhysxSchema, UsdPhysics, PhysicsSchemaTools
    from isaaclab.sim.utils.stage import get_current_stage
    from omni.physx import get_physx_simulation_interface
    contact_totals={}
    def report(headers,data):
        for h in headers:
            if not h.num_contact_data:continue
            a=str(PhysicsSchemaTools.intToSdfPath(h.collider0));b=str(PhysicsSchemaTools.intToSdfPath(h.collider1))
            key=a+' | '+b
            entry=contact_totals.setdefault(key,{'impulse_Ns':0.,'max_impulse_Ns':0.,'count':0})
            for ix in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data):
                mag=float(np.linalg.norm(data[ix].impulse));entry['impulse_Ns']+=mag;entry['max_impulse_Ns']=max(entry['max_impulse_Ns'],mag);entry['count']+=1
    subscription=get_physx_simulation_interface().subscribe_contact_report_events(report)
    def move(phase,side,target,opening,n,orientation=None,joint_space=False):
        nonlocal obs,step,seated
        if side=='left' and opening==0:opening=.38
        a=pos[side].copy(); r=rot[side].copy(); g=grip[side]
        rr=grip_rotation(0) if orientation is None else orientation
        qstart=q[side].copy()
        if joint_space:
            candidates=[ik.solve(target,ik.matrix_to_quat_xyzw(rr),q_init=seed,base_pos=np.array(station.BASES[side]),iters=200) for seed in ([np.array(selected['waypoints'][0]['q'])] if phase=='turn_bit' else [qstart,np.array(READY)])]
            qend,ee,re=min(candidates,key=lambda x:x[1]+x[2])
            if ee>.002 or re>np.radians(2):raise RuntimeError('Joint transit endpoint invalid')
        for i in range(1,n+1):
            u=ik.smoothstep(i/n)
            pp=a+u*(np.array(target)-a)
            rnow=ik.slerp_matrix(r,rr,u)
            if joint_space:
                q[side]=qstart+u*(qend-qstart);err=0.;rot_err=0.
                pp,rnow=ik.tcp_pose(q[side],np.array(station.BASES[side]))
            else:
                q[side],err,rot_err=ik.solve(pp,ik.matrix_to_quat_xyzw(rnow),q_init=q[side],base_pos=np.array(station.BASES[side]),iters=80)
            if err>.003 or rot_err>np.radians(3):
                (out/'abort.json').write_text(json.dumps({'phase':phase,'step':step,'position_error_m':err,'angle_error_deg':float(np.degrees(rot_err)),'mount_pass':False}))
                if frames:imageio.mimsave(out/'aborted.mp4',frames,fps=20/3,codec='libx264',output_params=['-pix_fmt','yuv420p'])
                raise RuntimeError(f'Unreachable path at {phase}: {err:.4f} m, {np.degrees(rot_err):.2f} deg')
            if step>=4800:raise RuntimeError('4800-step task limit exceeded')
            grip[side]=g+u*(opening-g)
            obs=emb.step(Action(data=pack(np.r_[q['left'],grip['left']],np.r_[q['right'],grip['right']]))).observation
            step+=1
            if step%3==0: frames.append(np.asarray(obs.images['top_cam']))
            if step%5==0 or i==n:
                st=object_states(emb._env,['bit','driver','drawer_context'])
                row={'step':step,'phase':phase,'joint_space_transit':joint_space,'ik_error':float(err),'ik_rotation_error_deg':float(np.degrees(rot_err)),
                     'commanded_tcp':{side:pp.tolist()},
                     'positions':{k:v.pos.tolist() for k,v in st.items()},
                     'quaternions':{k:v.quat_xyzw.tolist() for k,v in st.items()},
                     'bit_in_driver':st['driver'].rotation().T.dot(st['bit'].pos-st['driver'].pos).tolist()}
                row['measured_robot']={}
                for arm in ('left','right'):
                    robot=emb._env.unwrapped.scene[arm+'_robot']
                    actual=robot.data.joint_pos.torch[0].detach().cpu().numpy()
                    measured_tcp,_=ik.tcp_pose(actual[[robot.joint_names.index(j) for j in ARM_JOINTS]],np.array(station.BASES[arm]))
                    row['measured_robot'][arm]={'tcp':measured_tcp.tolist(),'finger_joints':actual[[robot.joint_names.index(j) for j in FINGER_JOINTS]].tolist(),'joint_names':robot.joint_names,'actual_q':actual.tolist(),'command_q':q[arm].tolist()}
                    for attr in ['applied_torque','computed_torque','joint_vel']:
                        try:row['measured_robot'][arm][attr]=getattr(robot.data,attr).torch[0].detach().cpu().numpy().tolist()
                        except (AttributeError,TypeError):pass
                row['contact_net']={}
                for arm in ('left','right'):
                    sensor=emb._env.unwrapped.scene[arm+'_contacts']
                    forces=sensor.data.net_forces_w
                    if hasattr(forces,'torch'):forces=forces.torch
                    row['contact_net'][arm]={'bodies':sensor.body_names,'force_N':forces[0].detach().cpu().numpy().tolist()}
                driver=emb._env.unwrapped.scene['driver']
                row['mechanism']=dict(task.MECHANISM)
                row['jaw_positions_m']=driver.data.joint_pos.torch[0].detach().cpu().numpy().tolist()
                row['contacts_since_last_sample']=dict(contact_totals)
                contact_totals.clear()
                rows.append(row)
                with (out/'trace.jsonl').open('a') as f: f.write(json.dumps(row)+'\n')
            if phase.startswith('insert_'):
                contact=object_states(emb._env,['bit','driver'])
                rel=contact['driver'].rotation().T@(contact['bit'].pos-contact['driver'].pos)
                rel[2]+=task.mechanism_state(emb._env)['pressure_m']
                if .1005<rel[2]<.1030 and np.linalg.norm(rel[:2])<.001:
                    robot=emb._env.unwrapped.scene[side+'_robot'];actual=robot.data.joint_pos.torch[0].detach().cpu().numpy()
                    target,rr=ik.tcp_pose(actual[[robot.joint_names.index(n) for n in ARM_JOINTS]],np.array(station.BASES[side]))
                    seated=True
                    break
        pos[side]=np.array(target); rot[side]=rr
        for cam,img in obs.images.items(): imageio.imwrite(out/f'{phase}_{cam}.png',np.asarray(img))
        imageio.imwrite(out/f'{phase}_overview.png',np.asarray(obs.images['top_cam']))
    import os
    resume=os.environ.get('REPAIR_CHECKPOINT')
    if resume:
        from checkpoint import restore
        cmds=restore(resume,emb._env)
        for name,dest in [('q',q),('pos',pos),('rot',rot)]:
            dest.update({arm:np.array(v) for arm,v in cmds[name].items()})
        grip.update(cmds['grip'])
    else:
        move('idle_control','left',pos['left'],1,20,rot['left'])
        # Side-grip the upright bit, turn it to the horizontal socket, then insert.
        pick=json.loads(Path(__file__).with_name('bit_side_pick_ik.json').read_text())[0]
        R=np.array([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]]);target=np.array([.107,-.13,task.TOP+.070])
        selected=json.loads(Path(__file__).with_name('selected_insertion_plan.json').read_text())['candidates'][0]
        mount=json.loads(Path(__file__).with_name('horizontal_side_insertion_ik.json').read_text())['solutions'][0]
        Rm=np.array(mount['rotation']);pm=np.array(mount['position'])+np.array([task.TOOL_SHIFT_X,0,0])
        move('approach_bit_side','right',target-R[:,2]*.08,1,40,R)
        move('grasp_position','right',target,1,35,R)
        move('close_bit','right',target,0,25,R)
        move('lift_bit','right',target+[0,0,.10],0,40,R)
        initial=object_states(emb._env,['bit','driver'])
        bit_lift=float(initial['bit'].pos[2]-(task.TOP+.067))
        if bit_lift<.04:
            result={'scope':'Side bit grasp only; stopped before insertion','bit_lift_m':bit_lift,'mount_pass':False}
        else:
            move('turn_bit','right',np.array(selected['waypoints'][0]['tcp']),0,80,np.array(selected['waypoints'][0]['R']),joint_space=True)
            move('near_socket','right',np.array(selected['waypoints'][1]['tcp'])+np.array([0,.017,0]),0,40,np.array(selected['waypoints'][1]['R']))
            seated=False
            for k,h in enumerate([.160,.152,.145,.138,.132,.128,.124,.120,.116,.112,.108,.104,.101]):
                for j in range(6):
                    st=object_states(emb._env,['bit','driver'])
                    rob=emb._env.unwrapped.scene['right_robot'];actual=rob.data.joint_pos.torch[0].detach().cpu().numpy()
                    tcp,Ractual=ik.tcp_pose(actual[[rob.joint_names.index(n) for n in ARM_JOINTS]],np.array(station.BASES['right']))
                    delta=st['driver'].rotation()@st['bit'].rotation().T
                    rv=Rotation.from_matrix(delta).as_rotvec();angle=np.linalg.norm(rv)
                    # Bound rotation feedback; hold axial position until orientation converges.
                    if angle>np.radians(2):delta=Rotation.from_rotvec(rv*np.radians(2)/angle).as_matrix()
                    rel=st['driver'].rotation().T@(st['bit'].pos-st['driver'].pos)
                    rel[2]+=task.mechanism_state(emb._env)['pressure_m']
                    if h<=.108 and .1005<rel[2]<.1030 and np.linalg.norm(rel[:2])<.001:
                        seated=True
                        # Unload elastic tracking error using a robot target at its measured pose.
                        pos['right']=tcp.copy();rot['right']=Ractual.copy()
                        break
                    depth=max(h,rel[2]) if angle>np.radians(.3) else h
                    desired_tip=st['driver'].pos+st['driver'].rotation()@np.array([0,0,depth-task.mechanism_state(emb._env)['pressure_m']])
                    offset=Ractual.T@(st['bit'].pos-tcp)
                    desired_actual_R=delta@Ractual
                    desired_R=delta@rot['right']
                    desired_p=pos['right']+desired_tip-desired_actual_R@offset-tcp
                    d=desired_p-pos['right'];length=np.linalg.norm(d)
                    if length>.004:desired_p=pos['right']+d*.004/length
                    move(f'insert_{k}_{j}','right',desired_p,0,10,desired_R)
                if seated:break
            if not seated:
                (out/'abort.json').write_text(json.dumps({'phase':'insertion_not_seated','step':step,'mount_pass':False}))
                imageio.mimsave(out/'aborted.mp4',frames,fps=20/3,codec='libx264',output_params=['-pix_fmt','yuv420p'])
                raise RuntimeError('Insertion did not seat; stop before release')
            move('release_bit','right',pos['right'],1,25,rot['right'])
            move('withdraw_axial','right',pos['right']+np.array([0,.06,0]),1,35,rot['right'])
            move('hold_after_release','right',pos['right'],1,40,rot['right'])
            st=object_states(emb._env,['bit','driver']);rel=st['driver'].rotation().T@(st['bit'].pos-st['driver'].pos)
            result={'scope':'Actual robot pick, insertion, release, withdrawal and 2 s hold; no tool lift or tightening','bit_lift_m':bit_lift,'bit_in_driver_m':rel.tolist(),'mount_pass':bool(np.linalg.norm(rel[:2])<.002 and .097<rel[2]<.106)}
        (out/'insertion_result.json').write_text(json.dumps(result,indent=2)+'\n')
        if not result['mount_pass']:raise RuntimeError('Insertion gate failed')
        move('withdraw_transit','right',pos['right']+np.array([0,0,.10]),1,60,rot['right'])
        move('return_pick_clear','right',target+[0,0,.10],1,70,R,joint_space=True)
        ready_p,ready_R=ik.tcp_pose(np.array(READY),np.array(station.BASES['right']))
        move('park_right','right',ready_p,1,60,ready_R,joint_space=True)
        plan=json.loads(Path(__file__).with_name('horizontal_plan.json').read_text())['candidates'][1]
        start=np.array(plan['start'])+np.array([task.TOOL_SHIFT_X,0,0]);Ri=np.array(plan['initial_R']);goal=np.array(plan['goal']);Rf=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
        pickup_initial_z=object_states(emb._env,['driver'])['driver'].pos[2]
        for name,target,opening,n,R in [('approach_tool',start+[0,0,.12],1,45,Ri),('lower_tool',start,1,45,Ri),('grasp_tool',start,0,30,Ri),('lift_tool',start+[0,0,.10],0,50,Ri),('turn_down',goal+[0,0,.10],0,80,Rf)]:
            move(name,'left',target,opening,n,R)
            if name=='lift_tool':
                lift=float(object_states(emb._env,['driver'])['driver'].pos[2]-pickup_initial_z)
                if lift<.06:
                    (out/'abort.json').write_text(json.dumps({'phase':'pickup_gate','tool_lift_m':lift,'passed':False}))
                    imageio.mimsave(out/'aborted.mp4',frames,fps=20/3,codec='libx264',output_params=['-pix_fmt','yuv420p'])
                    raise RuntimeError('Actual tool pickup failed')
        side_plan=json.loads(Path(__file__).with_name('oblique_approach_plan.json').read_text())['candidates'][0]
        for i,w in enumerate(side_plan['waypoints']):
            move(f'side_arc_{i}','left',np.array(w['p']),0,4,np.array(w['R']))
    if resume:move('grip_settle','left',pos['left'],.38,80,rot['left'])
    current_bit_R=object_states(emb._env,['bit'])['bit'].rotation()
    Rdesired=current_bit_R.copy()  # retain measured near-down tilt; verify engagement separately
    errors=[]
    for height in ([.015] if resume else [.035,.015]):
        converged=False
        for k in range(40):
            st=object_states(emb._env,['bit','driver','drawer_context']);bit=st['bit'];drawer=st['drawer_context']
            target_tip=drawer.pos+drawer.rotation()@np.array([task.FASTENER_X,-.06,.099+height])
            actual_tip=bit.pos+bit.rotation()@np.array([0.,0.,.094])
            dp=target_tip-actual_tip;delta=Rdesired@bit.rotation().T;rv=Rotation.from_matrix(delta).as_rotvec();angle=np.linalg.norm(rv)
            errors.append({'height_m':height,'iteration':k,'tip_error_m':float(np.linalg.norm(dp)),'angle_error_deg':float(np.degrees(angle))})
            if np.linalg.norm(dp)<.0015 and angle<np.radians(2):converged=True;break
            rob=emb._env.unwrapped.scene['left_robot'];aq=rob.data.joint_pos.torch[0].detach().cpu().numpy()
            tcp,Ractual=ik.tcp_pose(aq[[rob.joint_names.index(j) for j in ARM_JOINTS]],station.BASES['left'])
            if angle>np.radians(2):delta=Rotation.from_rotvec(rv*np.radians(2)/angle).as_matrix()
            offset=Ractual.T@(actual_tip-tcp)
            corrected_actual_R=delta@Ractual
            desired_p=pos['left']+target_tip-corrected_actual_R@offset-tcp
            d=.35*(desired_p-pos['left']);desired_p=pos['left']+d;length=np.linalg.norm(d)
            if length>.003:desired_p=pos['left']+d*.003/length
            move(f'align_{int(height*1000)}_{k}','left',desired_p,0,12,delta@rot['left'])
        if not converged:break
    # Two seconds of measured-tip feedback, ordinary joint targets only.
    hold_errors=[]
    previous_error=None
    for k in range(10):
        st=object_states(emb._env,['bit','drawer_context'])
        target_tip=st['drawer_context'].pos+st['drawer_context'].rotation()@np.array([task.FASTENER_X,-.06,.099+height])
        actual_tip=st['bit'].pos+st['bit'].rotation()@np.array([0.,0.,.094])
        error=target_tip-actual_tip
        velocity_error=np.zeros(3) if previous_error is None else error-previous_error
        correction=.20*error
        previous_error=error.copy()
        size=np.linalg.norm(correction)
        if size>.0005:correction*=.0005/size
        move(f'hold_feedback_{k}','left',pos['left']+correction,0,20,rot['left'])
        bs=object_states(emb._env,['bit'])['bit']
        hold_errors.append(float(np.linalg.norm(bs.pos+bs.rotation()@np.array([0.,0.,.094])-target_tip)))
        if k>=9 and max(hold_errors[-3:])<.001:break

    from checkpoint import save
    save(out/'hover_checkpoint.json',emb._env,{'q':{k:v.tolist() for k,v in q.items()},'pos':{k:v.tolist() for k,v in pos.items()},'rot':{k:v.tolist() for k,v in rot.items()},'grip':grip})
    st=object_states(emb._env,['bit','driver','drawer_context']);tip=st['bit'].pos+st['bit'].rotation()@np.array([0.,0.,.094]);rel=st['driver'].rotation().T@(st['bit'].pos-st['driver'].pos)
    result={'scope':'Actual robot bit installation, tool pickup and oblique hover; no engagement or tightening','converged':converged,'tip_error_m':float(np.linalg.norm(tip-target_tip)),'final_target_height_m':height,'bit_in_driver_m':rel.tolist(),'steps':step,'tool_tilt_deg':float(np.degrees(np.arccos(-st['bit'].rotation()[2,2]))),'errors':errors,'hold_mode':'Ten quasistatic updates at 1 Hz; approach gate: 3 mm position error and 2 mm lateral error at 15 mm nominal clearance','hold_errors_m':hold_errors}
    result['verification_max_error_m']=max(hold_errors[-3:])
    result['approach_clearance_pass']=bool(height==.015 and result['tip_error_m']<.003 and result['verification_max_error_m']<.003 and np.linalg.norm((tip-target_tip)[:2])<.002 and np.linalg.norm(rel[:2])<.002 and .097<rel[2]<.106)
    (out/'hover_result.json').write_text(json.dumps(result,indent=2)+'\n')
    imageio.mimsave(out/'alignment.mp4',frames,fps=20/3,codec='libx264',output_params=['-pix_fmt','yuv420p'])
    print(json.dumps({k:v for k,v in result.items() if k!='errors'}),flush=True)
    # Only robot joint/gripper actions below; physical switches own motor activation.
    if not result['approach_clearance_pass']:
        raise RuntimeError('Hover gate failed; no tightening attempted')
    rr=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
    for label,p,n in [('brace_approach',[.295,-.23,task.TOP+.20],45),('brace_lower',[.325,-.23,task.TOP+.17],35),('brace_side',[.325,-.122,task.TOP+.17],40),('brace_contact',[.395,-.122,task.TOP+.17],40)]:
        move(label,'right',np.array(p),1,n,rr)
    move('brace_close','right',pos['right'],0,25,rr)
    # Match only the insertion axis. The physical motor turns the cross into the recess.
    for k in range(20):
        st=object_states(emb._env,['bit','drawer_context'])
        axis=st['bit'].rotation()[:,2];want=-st['drawer_context'].rotation()[:,2]
        rv=np.cross(axis,want);ang=np.arctan2(np.linalg.norm(rv),np.dot(axis,want))
        if ang<np.radians(.7):break
        delta=Rotation.from_rotvec(rv/max(np.linalg.norm(rv),1e-9)*min(ang,np.radians(1))).as_matrix()
        tip=st['bit'].pos+st['bit'].rotation()@np.array([0,0,.094])
        # Rotate about the measured working tip, keeping its position stable.
        pnew=tip+delta@(pos['left']-tip)
        move(f'axis_{k}','left',pnew,0,10,delta@rot['left'])
    for h in [.010,.006,.003,.001]:
        for k in range(20):
            st=object_states(emb._env,['bit','drawer_context'])
            target=st['drawer_context'].pos+st['drawer_context'].rotation()@np.array([task.FASTENER_X,-.06,.099+h])
            tip=st['bit'].pos+st['bit'].rotation()@np.array([0,0,.094]);err=target-tip
            if np.linalg.norm(err)<.0006:break
            correction=.25*err
            if np.linalg.norm(correction)>.001:correction*=.001/np.linalg.norm(correction)
            move(f'engage_{h}_{k}','left',pos['left']+correction,0,4,rot['left'])
    press_start=pos['left'].copy();powered=[]
    for k in range(140):
        st=object_states(emb._env,['bit','drawer_context'])
        target=st['drawer_context'].pos+st['drawer_context'].rotation()@np.array([task.FASTENER_X,-.06,.099])
        tip=st['bit'].pos+st['bit'].rotation()@np.array([0,0,.094]);m=task.mechanism_state(emb._env)
        d=.12*(target-tip);d[2]=-np.clip(.8*(.0022-m['pressure_m']),-.001,.001)
        d[:2]=np.clip(d[:2],-.0002,.0002)
        nxt=pos['left']+d;nxt[2]=max(nxt[2],press_start[2]-.045)
        move(f'powered_{k}','left',nxt,0,4,rot['left'])
        powered.append(dict(task.MECHANISM))
        if k>15 and max(x['handle_gap_m'] for x in powered[-10:])<.0002:break
    (out/'powered_trace.json').write_text(json.dumps(powered,indent=2))
    save(out/'powered_checkpoint.json',emb._env,{'q':{k:v.tolist() for k,v in q.items()},'pos':{k:v.tolist() for k,v in pos.items()},'rot':{k:v.tolist() for k,v in rot.items()},'grip':grip})
    if task.mechanism_state(emb._env)['handle_gap_m']>.0002:
        imageio.mimsave(out/'tightening_failed.mp4',frames,fps=20/3,codec='libx264',output_params=['-pix_fmt','yuv420p'])
        raise RuntimeError('Handle did not close; no full-score claim')
    move('disengage','left',pos['left']+[0,0,.040],0,50,rot['left'])
    move('brace_release','right',pos['right'],1,20,rr)
    move('brace_withdraw','right',pos['right']+[-.08,0,0],1,40,rr)
    ready_p,ready_R=ik.tcp_pose(np.array(READY),np.array(station.BASES['right']))
    move('right_clear','right',ready_p,1,60,ready_R,joint_space=True)
    plan=json.loads(Path(__file__).with_name('horizontal_plan.json').read_text())['candidates'][1]
    start=np.array(plan['start'])+np.array([task.TOOL_SHIFT_X,0,0]);Ri=np.array(plan['initial_R'])
    goal=np.array(plan['goal']);Rf=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
    move('tool_clear','left',goal+[0,0,.10],0,80,Rf)
    move('tool_horizontal','left',start+[0,0,.10],0,80,Ri)
    move('tool_stand','left',start,0,70,Ri)
    move('tool_release','left',start,1,30,Ri)
    move('left_clear','left',start+[0,0,.12],1,50,Ri)
    move('final_hold','left',pos['left'],1,30,Ri)
    result={'scope':'Full robot sequence' if not resume else 'Diagnostic checkpoint continuation; not full-demo evidence',
            'score':task.score(None),'mechanism':dict(task.MECHANISM),'steps':step,'full_demo':not bool(resume)}
    (out/'result.json').write_text(json.dumps(result,indent=2))
    imageio.mimsave(out/'demo.mp4',frames,fps=20/3,codec='libx264',output_params=['-pix_fmt','yuv420p'])
    print(json.dumps(result),flush=True)

if __name__=='__main__':run(Path(__import__('sys').argv[1]))
