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
    q={s:np.array(READY,float)for s in('left','right')};tcp={};rot={};grip={s:1. for s in q};down=grip_rotation();worst=0.;trace=[];images=[]
    for s in q:tcp[s],rot[s]=ik.tcp_pose(q[s],np.array(station.BASES[s]))
    def step(label):
        nonlocal obs
        obs=emb.step(Action(data=pack(np.r_[q['left'],grip['left']],np.r_[q['right'],grip['right']]))).observation
        st=object_states(emb._env,['carrier','handle_left','handle_right'])
        actual_grippers={}
        for side in q:
            rob=base.scene[side+'_robot'];jv=getattr(rob.data.joint_pos,'torch',rob.data.joint_pos)[0]
            ja=jv[[rob.joint_names.index(n)for n in ARM_JOINTS]].detach().cpu().numpy();tip,_=ik.tcp_pose(ja,np.array(station.BASES[side]))
            actual_grippers[side]=dict(tcp=tip.tolist(),opening=float(jv[[rob.joint_names.index(n)for n in FINGER_JOINTS]].mean().item()/FINGER_OPEN),bar_distance_m=float(np.linalg.norm(tip-st['handle_'+side].pos)))
        trace.append(dict(grippers=actual_grippers,step=len(trace),phase=label,score=task.score(None),state=dict(task.STATE),carrier=st['carrier'].pos.tolist(),handle_left=st['handle_left'].pos.tolist(),handle_right=st['handle_right'].pos.tolist()))
        if len(trace)%2==0:images.append(np.asarray(obs.images['top_cam']))
    def hold(n,label):
        for _ in range(n):step(label)
    def move(label,targets,opening,n):
        nonlocal worst
        starts={s:tcp[s].copy()for s in q};r0={s:rot[s].copy()for s in q};g0=grip.copy()
        for i in range(1,n+1):
            u=ik.smoothstep(i/n)
            for s in q:
                pt=starts[s]+u*(np.array(targets.get(s,starts[s]))-starts[s]);rt=ik.slerp_matrix(r0[s],down,u)
                q[s],e,_=ik.solve(pt,ik.matrix_to_quat_xyzw(rt),q_init=q[s],base_pos=np.array(station.BASES[s]),iters=100)
                worst=max(worst,e);tcp[s]=pt;rot[s]=rt;grip[s]=g0[s]+u*(opening.get(s,g0[s])-g0[s])
            step(label)
        print(json.dumps(dict(phase=label,score=task.score(None),state=task.STATE)),flush=True)
    def shot(name):imageio.imwrite(out/(name+'.png'),obs.images['top_cam'])
    hold(30,'settle');shot('setup')
    if mode!='idle':
        st=object_states(emb._env,['handle_left','handle_right']);targets={s:st['handle_'+s].pos+np.array([0,0,.012])for s in q}
        move('approach',{s:p+[0,0,.065]for s,p in targets.items()},{s:1. for s in q},40)
        move('lower_to_bars',targets,{s:1. for s in q},35)
        move('grasp_both',targets,{s:0. for s in q},30);hold(10,'grasp_settle');shot('grasp')
        if mode=='unsafe':
            move('continue_right_lift',{'right':tcp['right']+[0,0,.09]},{},90);hold(25,'unsafe_hold')
        else:
            # Clear the visible lip horizontally before lifting; allowed safe path.
            move('slide_clear',{s:tcp[s]+[.055,0,0]for s in q},{},50)
            move('lift_level',{s:tcp[s]+[0,0,.055]for s in q},{},45);hold(15,'lift_hold');shot('lifted')
            if mode=='demo':
                move('transfer',{s:tcp[s]+[task.DEST-task.SOURCE-.055,0,0]for s in q},{},60)
                actual=object_states(emb._env,['carrier'])['carrier'].pos
                correction=np.array([task.DEST-actual[0],-actual[1],0.])
                move('align_receiver',{s:tcp[s]+correction for s in q},{},30)
                actual=object_states(emb._env,['carrier'])['carrier'].pos
                move('seat',{s:tcp[s]+[0,0,task.TOP+task.BASE-actual[2]]for s in q},{},45)
                actual=object_states(emb._env,['carrier'])['carrier'].pos
                move('finish_seating',{s:tcp[s]+[0,0,task.TOP+task.BASE-actual[2]-.001]for s in q},{},30)
                hold(15,'supported_settle')
                actual=object_states(emb._env,['carrier'])['carrier'].pos
                correction=np.array([task.DEST-actual[0],-.003-actual[1],0.])
                move('center_on_receiver',{s:tcp[s]+correction for s in q},{},30)
                hold(20,'centered_settle')
                move('release',tcp.copy(),{s:1. for s in q},45)
                move('clear_hands',{s:tcp[s]+[-.015,.055 if s=='left'else-.055,.055]for s in q},{},35)
    hold(35,'final');shot('final')
    result=dict(mode=mode,physical_score=task.score(None),success=task.success(None),steps=len(trace),worst_ik_error_m=worst,state=task.STATE,carrier_start=trace[0]['carrier'],carrier_final=trace[-1]['carrier'],max_carrier_z=max(r['carrier'][2]for r in trace))
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');(out/'trace.jsonl').write_text(''.join(json.dumps(r)+'\n'for r in trace));imageio.mimsave(out/'demo.mp4',images,fps=10,codec='libx264',output_params=['-pix_fmt','yuv420p','-movflags','+faststart']);print(json.dumps(result),flush=True)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--mode',choices=['probe','demo','idle','unsafe'],default='probe');a=p.parse_args();run(a.output,a.mode)
