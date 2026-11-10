"""Physical robot-actions-only trial. No object/joint state writes or attachments."""
from pathlib import Path
import argparse,json,math
import numpy as np

def run(out,mode):
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import ik,station
    from yam_arena.constants import READY,ARM_JOINTS,FINGER_JOINTS,FINGER_OPEN
    from yam_arena.demo import grip_rotation
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    import imageio.v2 as imageio
    out.mkdir(parents=True,exist_ok=False)
    emb=yam_arena_embodiment(task=str(Path(__file__).with_name('task.py').resolve()),object_jitter=0,control_hz=20,cam_width=640,cam_height=360,top_cam='demo')
    task=emb.task.module;obs=emb.reset(Scene(id='rack_'+mode,instruction=task.INSTRUCTION))
    from yam_arena.scene import object_states
    import torch
    base=emb._env.unwrapped
    from clearance import carrier_lip_gap,carried_obstacle_gap
    def clearance(carrier):return carrier_lip_gap(task,carrier.pos,carrier.rotation())
    contact_events=[];contact_report_error=None;contact_subscription=None
    try:
        from pxr import PhysxSchema,PhysicsSchemaTools
        from isaaclab.sim.utils.stage import get_current_stage
        import omni.physx
        for prim in get_current_stage().Traverse():
            if str(prim.GetPath()).endswith('/carrier/body') or str(prim.GetPath()).endswith('/retaining_lip/body'):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0)
        def contact_callback(headers,data):
            for h in headers:
                a=str(PhysicsSchemaTools.intToSdfPath(h.actor0));b=str(PhysicsSchemaTools.intToSdfPath(h.actor1))
                if 'retaining_lip' in a+b or 'carrier' in a+b:
                    impulses=[float(np.linalg.norm(data[k].impulse))for k in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data)]
                    contact_events.append(dict(actor0=a.split('/')[-2],actor1=b.split('/')[-2],impulse_norm_sum_Ns=sum(impulses)))
        contact_subscription=omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(contact_callback)
    except Exception as exc:contact_report_error=str(exc)
    q={s:np.array(READY,float)for s in('left','right')};tcp={};rot={};grip={s:1. for s in q};down=grip_rotation();worst=0.;trace=[];images=[]
    for s in q:tcp[s],rot[s]=ik.tcp_pose(q[s],np.array(station.BASES[s]))
    def step(label):
        nonlocal obs
        obs=emb.step(Action(data=pack(np.r_[q['left'],grip['left']],np.r_[q['right'],grip['right']]))).observation
        st=object_states(emb._env,['carrier','handle_left','handle_right']+['sample_'+str(i)for i in range(4)])
        actual_grippers={}
        for side in q:
            rob=base.scene[side+'_robot'];jv=getattr(rob.data.joint_pos,'torch',rob.data.joint_pos)[0]
            ja=jv[[rob.joint_names.index(n)for n in ARM_JOINTS]].detach().cpu().numpy();tip,_=ik.tcp_pose(ja,np.array(station.BASES[side]))
            actual_grippers[side]=dict(command_tcp=tcp[side].tolist(),command_q=q[side].tolist(),actual_q=ja.tolist(),tracking_error_m=float(np.linalg.norm(tip-tcp[side])),tcp=tip.tolist(),opening=float(jv[[rob.joint_names.index(n)for n in FINGER_JOINTS]].mean().item()/FINGER_OPEN),bar_distance_m=float(np.linalg.norm(tip-st['handle_'+side].pos)))
        trace.append(dict(sample_max_angular_speed_rad_s=max(float(np.linalg.norm(st['sample_'+str(i)].ang_vel))for i in range(4)),receiver_clearance_m=carried_obstacle_gap(task,st,'receiver'),carrier_quat=st['carrier'].quat_xyzw.tolist(),lip_clearance_m=clearance(st['carrier']),contacts=list(contact_events),grippers=actual_grippers,step=len(trace),phase=label,score=task.score(None),state=dict(task.STATE),carrier=st['carrier'].pos.tolist(),handle_left=st['handle_left'].pos.tolist(),handle_right=st['handle_right'].pos.tolist()))
        contact_events.clear()
        if len(trace)%2==0:images.append(np.asarray(obs.images['top_cam']))
        if task.STATE['invalid'] or task.STATE['max_sample_tilt_deg']>25.:
            (out/'trace.jsonl').write_text(''.join(json.dumps(r)+'\n'for r in trace))
            (out/'result.json').write_text(json.dumps(dict(mode=mode,physical_score=task.score(None),success=False,stopped_early=True,reason='carrier safety violation or sample toppled',steps=len(trace),state=task.STATE),indent=2))
            imageio.imwrite(out/'final.png',obs.images['top_cam']);imageio.mimsave(out/'demo.mp4',images,fps=10,codec='libx264',output_params=['-pix_fmt','yuv420p'])
            raise RuntimeError('Stopped after observed carrier violation or sample toppling')
    def hold(n,label):
        for _ in range(min(n,10) if mode=='release' and label!='final' else n):step(label)
    def move(label,targets,opening,n):
        nonlocal worst
        starts={s:tcp[s].copy()for s in q};r0={s:rot[s].copy()for s in q};g0=grip.copy()
        for i in range(1,n+1):
            v=i/n;u=ik.smoothstep(v) if label in ('approach','lower_to_bars','grasp_both') else v*v*v*(10.+v*(-15.+6.*v))
            for s in q:
                pt=starts[s]+u*(np.array(targets.get(s,starts[s]))-starts[s]);rt=ik.slerp_matrix(r0[s],down,u)
                q[s],e,_=ik.solve(pt,ik.matrix_to_quat_xyzw(rt),q_init=q[s],base_pos=np.array(station.BASES[s]),iters=100)
                worst=max(worst,e);tcp[s]=pt;rot[s]=rt;grip[s]=g0[s]+u*(opening.get(s,g0[s])-g0[s])
            step(label)
        print(json.dumps(dict(phase=label,score=task.score(None),state=task.STATE)),flush=True)
    def shot(name):imageio.imwrite(out/(name+'.png'),obs.images['top_cam'])
    hold(10,'settle');shot('setup')
    if mode!='idle':
        st=object_states(emb._env,['handle_left','handle_right']);targets={s:st['handle_'+s].pos+np.array([0,0,.012])for s in q}
        move('approach',{s:p+[0,0,.065]for s,p in targets.items()},{s:1. for s in q},40)
        move('lower_to_bars',targets,{s:1. for s in q},35)
        move('grasp_both',targets,{s:0. for s in q},30);hold(10,'grasp_settle');shot('grasp')
        if mode=='camera':
            previews={
                'far_positive':([.72,.25,task.TOP+.58],[.33,0.,task.TOP+.05]),
                'far_center':([.72,0.,task.TOP+.62],[.33,0.,task.TOP+.05]),
                'far_negative':([.72,-.25,task.TOP+.58],[.33,0.,task.TOP+.05])}
            for label,(eye,look) in previews.items():
                base.scene['top_cam'].set_world_poses_from_view(torch.tensor([eye],device=base.device),torch.tensor([look],device=base.device))
                hold(3,'camera_'+label);shot(label)
            (out/'result.json').write_text(json.dumps({'mode':'camera_preview','state':task.STATE,'physical_score':task.score(None)},indent=2))
            return
        if mode=='unsafe':
            move('continue_right_lift',{'right':tcp['right']+[0,0,.09]},{},90);hold(25,'unsafe_hold')
        else:
            # Clear the visible lip horizontally before lifting; allowed safe path.
            move('slide_clear',{s:tcp[s]+[.080,0,0]for s in q},{},140)
            st=object_states(emb._env,['carrier'])['carrier']
            if clearance(st)<.030 or st.pos[0]-task.SOURCE<.070 or trace[-1]['receiver_clearance_m']<.005:
                (out/'trace.jsonl').write_text(''.join(json.dumps(r)+'\n'for r in trace))
                raise RuntimeError('Measured clearance <30mm or horizontal movement <70mm before lift')
            move('lift_level',{s:tcp[s]+[0,0,.045]for s in q},{},85);hold(15,'lift_hold');shot('lifted')
            if mode in ('demo','release'):
                move('transfer',{s:tcp[s]+[task.DEST-task.SOURCE-.080,0,0]for s in q},{},110)
                actual=object_states(emb._env,['carrier'])['carrier'].pos
                correction=np.array([task.DEST-actual[0],-actual[1],0.])
                if np.linalg.norm(correction)>.004:move('align_receiver',{s:tcp[s]+correction for s in q},{},40)
                actual=object_states(emb._env,['carrier'])['carrier'].pos
                move('seat',{s:tcp[s]+[0,0,task.TOP+task.BASE-actual[2]]for s in q},{},80)
                hold(15,'supported_settle')
                actual=object_states(emb._env,['carrier'])['carrier'].pos
                correction=np.array([task.DEST-actual[0],-actual[1],0.])
                if np.linalg.norm(correction)>.004:move('center_on_receiver',{s:tcp[s]+correction for s in q},{},40)
                hold(20,'centered_settle')
                grip.update({s:trace[-1]['grippers'][s]['opening']for s in q})
                move('release_in_place',tcp.copy(),{s:1. for s in q},35)
                move('clear_hands',{s:tcp[s]+[0,0,.045]for s in q},{},40)
    hold(35,'final');shot('final')
    result=dict(contact_reporting_verified=any(r['contacts']for r in trace),contact_report_error=contact_report_error,mode=mode,physical_score=task.score(None),success=task.success(None),steps=len(trace),worst_ik_error_m=worst,state=task.STATE,carrier_start=trace[0]['carrier'],carrier_final=trace[-1]['carrier'],max_carrier_z=max(r['carrier'][2]for r in trace))
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');(out/'trace.jsonl').write_text(''.join(json.dumps(r)+'\n'for r in trace));imageio.mimsave(out/'demo.mp4',images,fps=10,codec='libx264',output_params=['-pix_fmt','yuv420p','-movflags','+faststart']);print(json.dumps(result),flush=True)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--mode',choices=['probe','demo','idle','unsafe','camera','release'],default='probe');a=p.parse_args();run(a.output,a.mode)
