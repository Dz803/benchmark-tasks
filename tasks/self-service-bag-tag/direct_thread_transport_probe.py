"""Continuous printer pickup and measured mid-strip regrasp, no object resets."""
import json
import os
import sys
import time
from pathlib import Path
import numpy as np
from yam_arena import station, ik
from yam_arena.constants import FINGER_OPEN, FINGER_CLOSED, READY
from bagtag_probe import Plan, Run, W, frame, hold_ready
from exit_guide_pickup_probe import pickup_waypoints, panel_frame


def main(out):
    run = Run(out, 'direct_thread_transport')
    from isaaclab.sim.utils.stage import get_current_stage
    material = {}
    for prim in get_current_stage().Traverse():
        if 'checked_suitcase' in str(prim.GetPath()) and prim.GetName() == 'physics':
            material = {a.GetName(): str(a.Get()) for a in prim.GetAttributes() if 'friction' in a.GetName().lower()}
    (out/'case_material.json').write_text(json.dumps(material,indent=2)+'\n')
    reports = []
    def current(side):
        return np.asarray(run.state()['robots'][side]['joints'])
    def hold(side):
        q = current(side)
        # Maintain motor closure, not the measured obstruction position: using
        # the latter removes squeeze force and lets the paper slide away.
        a = np.r_[q[:6], 1.0 if np.mean(q[6:]) < -.02 else 0.0]
        return lambda t: (a, 'hold current')
    def stage(side, waypoints, name):
        q = current(side)
        check = Plan(waypoints, station.BASES[side], q_start=q[:6], finger_start=float(np.mean(q[6:])))
        for t in np.arange(.05, check.duration+.001, .05):
            check.action7(t)
        if check.worst_ik_mm > float(os.environ.get('BAGTAG_IK_MM','5')) or check.worst_ik_deg > float(os.environ.get('BAGTAG_IK_DEG','8')):
            raise RuntimeError(f'Preflight IK rejected {name}: {check.worst_ik_mm:.2f} mm / {check.worst_ik_deg:.2f} deg')
        plan = Plan(waypoints, station.BASES[side], q_start=q[:6], finger_start=float(np.mean(q[6:])))
        count = round(plan.duration * 20)
        start = run.step_i
        funcs = {'left': hold('left'), 'right': hold('right')}
        funcs[side] = plan.action7
        # Release/withdrawal audit: robot-link/panel contacts and panel motion while a hand lets go.
        was_closed = float(np.mean(q[6:])) > -.004
        opening = was_closed and (any(w.finger < float(np.mean(q[6:])) - .002 for w in waypoints) or any(
            k in (getattr(w, 'label', '') or '').lower() for w in waypoints for k in ('release', 'withdraw', 'let go', 'back out')))
        audit = None
        if opening:
            import robot_contacts
            from scipy.spatial.transform import Rotation as _R
            st0 = run.state(); x0 = np.asarray(st0['panel_xyz'])
            audit = {'contacts': {}, 'max_panel_motion_m': 0., 'worst_panel': None}
            env = run.emb._env; sim_dt = float(env.unwrapped.cfg.sim.dt)
            def hook(r, k):
                for sd, link, pnl, f in robot_contacts.read(env, sim_dt):
                    key = f'{sd}:{link}->panel{pnl:02d}'
                    c = audit['contacts'].setdefault(key, {'steps': 0, 'max_force_n': 0.})
                    c['steps'] += 1; c['max_force_n'] = max(c['max_force_n'], f)
                if k % 4 == 0 or k == count - 1:
                    xs = np.asarray(r.state()['panel_xyz']); d = np.linalg.norm(xs - x0, axis=1)
                    if d.max() > audit['max_panel_motion_m']:
                        audit['max_panel_motion_m'] = float(d.max()); audit['worst_panel'] = int(d.argmax())
        else:
            hook = None
        run.go(count, funcs['left'], funcs['right'], snaps=(start+count,), video_every=4, hook=hook)
        entry = {'phase': name, 'step': run.step_i, 'ik_mm': plan.worst_ik_mm, 'ik_deg': plan.worst_ik_deg}
        if audit is not None:
            audit['panel_moved_over_5mm'] = audit['max_panel_motion_m'] > .005
            entry['release_audit'] = audit
        reports.append(entry)
        run.save({'stage': 'continuous_partial_transport_not_scored', 'phases': reports})
        if plan.worst_ik_mm > float(os.environ.get('BAGTAG_IK_MM','5')) or plan.worst_ik_deg > float(os.environ.get('BAGTAG_IK_DEG','8')):
            raise RuntimeError('IK gate failed in ' + name)
    def move_joint_target(side, pos, R, finger, name, q_seed=None):
        from handle_route_cpu import best_ik, obstacle_points
        from exit_guide_pickup_cpu import inside_any
        import yam_hulls
        from scipy.spatial import cKDTree
        q0 = current(side)[:6]
        if q_seed is not None:
            q1, pe, re = ik.solve(np.asarray(pos), ik.matrix_to_quat_xyzw(R), np.asarray(q_seed, float), base_pos=np.asarray(station.BASES[side]), iters=300)
        else:
            q1, pe, re = best_ik(side, np.asarray(pos), R)
        if pe > .001 or np.degrees(re) > 1:
            raise RuntimeError('No accurate pregrasp IK')
        obs = obstacle_points(.008)
        other = 'right' if side == 'left' else 'left'
        other_h = [h for hs in yam_hulls.hulls(current(other),other).values() for h in hs]
        other_v = np.concatenate([h.vertices for h in other_h])
        other_tree = cKDTree(other_v)
        for f in np.linspace(0,1,25):
            q = q0+(q1-q0)*ik.smoothstep(f)
            hs = [h for pieces in yam_hulls.hulls(np.r_[q,finger,finger],side).values() for h in pieces]
            if any(inside_any(points,hs).any() for points in obs.values()):
                raise RuntimeError('Pregrasp joint route obstacle collision')
            if other_tree.query(np.concatenate([h.vertices for h in hs]))[0].min() < float(os.environ.get('BAGTAG_ARM_CLEARANCE', '.002')):
                raise RuntimeError('Pregrasp joint route arm clearance')
        def command(t):
            return np.r_[q0+(q1-q0)*ik.smoothstep(min(t/3,1)),finger/FINGER_OPEN],name
        funcs={'left':hold('left'),'right':hold('right')};funcs[side]=command
        start=run.step_i
        run.go(80,funcs['left'],funcs['right'],snaps=(start+80,),video_every=4)
        reports.append({'phase':name,'step':run.step_i,'ik_mm':float(pe*1000),'ik_deg':float(np.degrees(re))})
        run.save({'stage':'continuous_partial_transport_not_scored','phases':reports})
    def roll_in_place(side, delta, finger, name, seconds=3.):
        """Turn only the wrist roll joint by `delta` (rad): the held panel turns over about the gripper
        axis with almost no translation, so the strip is not dragged. Collision-checked like
        move_joint_target; the roll stays inside the joint limit."""
        from handle_route_cpu import obstacle_points
        from exit_guide_pickup_cpu import inside_any
        import yam_hulls
        from scipy.spatial import cKDTree
        q0=current(side)[:6]; q1=q0.copy(); q1[5]=q0[5]+delta
        if abs(q1[5])>2.09:
            raise RuntimeError(f'Wrist roll {q1[5]:.2f} rad outside the joint limit')
        obs=obstacle_points(.008); other='right' if side=='left' else 'left'
        other_tree=cKDTree(np.concatenate([h.vertices for hs in yam_hulls.hulls(current(other),other).values() for h in hs]))
        for f in np.linspace(0,1,25):
            q=q0+(q1-q0)*f
            hs=[h for pieces in yam_hulls.hulls(np.r_[q,finger,finger],side).values() for h in pieces]
            if any(inside_any(points,hs).any() for points in obs.values()):
                raise RuntimeError('Wrist roll route obstacle collision')
            if other_tree.query(np.concatenate([h.vertices for h in hs]))[0].min()<.005:
                raise RuntimeError('Wrist roll route arm clearance')
        n=round(seconds*20)
        def command(t):
            return np.r_[q0+(q1-q0)*ik.smoothstep(min(t/seconds,1)),finger/FINGER_OPEN],name
        funcs={'left':hold('left'),'right':hold('right')};funcs[side]=command
        start=run.step_i
        run.go(n+10,funcs['left'],funcs['right'],snaps=(start+n+10,),video_every=4)
        reports.append({'phase':name,'step':run.step_i,'roll_rad':float(delta)})
        run.save({'stage':'continuous_partial_transport_not_scored','phases':reports})
    try:
        staged = os.environ.get('BAGTAG_STAGE_STATE')
        if staged:
            from restore_contact_diagnostic import restore
            restore(run, staged, int(os.environ.get('BAGTAG_STAGE_STEP','1838')))
            run.snap('staged_initial_not_demo')
            run.save({'stage':'staged_contact_diagnostic_not_demo','phases':reports})
        else:
            run.snap('reset')
            stage('right', pickup_waypoints(.8), 'proven pickup replay')
            if os.environ.get('BAGTAG_NEGATIVE') == 'drop_after_extraction':
                q=current('right')[:6].copy()
                run.go(120,hold_ready(),lambda t:(np.r_[q,1.],'open grip and let strip fall'),snaps=(run.step_i+120,),video_every=4)
                reports.append({'phase':'physical negative: drop without attachment','step':run.step_i})
                assert run.state()['closure']['score']==0., 'Dropped tag must score zero'
                run.save({'stage':'physical_negative_drop_passed','phases':reports})
                return
            Rx = frame((1,0,0),(0,0,1))
            PX = float(os.environ.get('BAGTAG_PRESENT_X', '.22'))   # strip hangs clear of the left hand's route
            PZ = float(os.environ.get('BAGTAG_PRESENT_Z', '1.285'))  # the real-stiffness strip hangs straight down
            stage('right', [W(2.0,(PX,-.18,PZ),Rx,FINGER_CLOSED,'present tag beside case'),
                            W(1.0,(PX,-.18,PZ),Rx,FINGER_CLOSED,'settle tag')], 'present')
            # Wait (hands holding still) until the hanging strip has stopped swinging before the left
            # hand measures it: at most 6 s, checked every 0.5 s on the panel positions.
            x_prev = np.asarray(run.state()['panel_xyz'])
            for _ in range(12):
                run.go(10, hold('left'), hold('right'), snaps=(run.step_i+10,), video_every=4)
                x_now = np.asarray(run.state()['panel_xyz'])
                swing = float(np.max(np.linalg.norm(x_now - x_prev, axis=1)))
                x_prev = x_now
                if swing < .003:
                    break
            reports.append({'phase':'wait for hanging strip to settle','step':run.step_i,'last_half_second_motion_m':swing})
            s = run.state()
            # Mid-strip grasp panel: 3.5 panels (187 mm) of leader remain toward panel00,
            # matching the leader length the threading motions were tuned for.
            # The grasp point is the same strip position (240 mm from the leading end) for any panel
            # count: panel 4 of 10, or halfway between panels 8 and 9 of 20.
            from tag_dims import SEGMENTS as _SEG
            MIDF = float(os.environ.get('BAGTAG_MID_PANEL', '8.17'))   # about 330 mm from the leading end (14 panels)
            MID = int(MIDF)
            c, P = panel_frame(s['panel_xyz'][MID], s['panel_quat_xyzw'][MID])
            if MIDF > MID:
                c = (1 - (MIDF - MID)) * np.asarray(c) + (MIDF - MID) * np.asarray(s['panel_xyz'][MID + 1])
            mid_point = lambda st_: (1 - (MIDF - MID)) * np.asarray(st_['panel_xyz'][MID]) + (MIDF - MID) * np.asarray(st_['panel_xyz'][min(MID + 1, _SEG - 1)])
            approach = np.array([0.,-1.,0.])
            # The opposite pad normal avoids the left wrist roll limit. With this
            # grasp, rolling to Rx presents the panel00 end toward -Y for threading.
            R = frame(approach, -P[:,2])
            gp = c - .013*approach
            # Pass above the suitcase handle first (joint route, collision-checked), then descend
            # vertically to the pregrasp point; the real-stiffness strip hangs too low for a direct swing.
            pre = gp-.080*approach
            move_joint_target('left',pre+np.array([0.,0.,.10]),R,FINGER_OPEN,f'above the handle before panel{MID:02d}')
            stage('left',[W(1.5,pre,R,FINGER_OPEN,f'descend beside panel{MID:02d}')],f'orient beside panel{MID:02d}')
            stage('left', [W(1.5,gp,R,FINGER_OPEN,f'straddle panel{MID:02d} faces'),
                           W(1.,gp,R,FINGER_CLOSED,f'pinch panel{MID:02d}'),
                           W(.8,gp,R,FINGER_CLOSED,'hold both pinches')], 'left mid-strip grasp')
            before = run.state()
            rc, rr = ik.tcp_pose(current('right')[:6],station.BASES['right'])
            stage('right',[W(.7,rc,rr,FINGER_OPEN,'release end A'),
                           W(1.5,rc+[0,-.06,0],rr,FINGER_OPEN,'clear released end')], 'right release')
            s = run.state()
            lp, lr = ik.tcp_pose(current('left')[:6],station.BASES['left'])
            offset = lr.T @ (mid_point(s)-lp)
            reports.append({'phase':'left grip verification','mid_panel':MID,'mid_panel_in_tcp_m':offset.tolist(),
                            'left_fingers_m':current('left')[6:].tolist(),
                            'mid_panel_motion_after_right_release_m':float(np.linalg.norm(mid_point(s)-mid_point(before)))})
            if abs(offset[0]) > .006 or np.linalg.norm(offset) > .045:
                run.save({'stage':'midstrip_regrasp_failed_no_feed','phases':reports})
                return
            stage('right',[W(2.,(.20,-.30,1.30),Rx,FINGER_OPEN,'clear route for left hand')], 'right clear')
            stage('left',[W(2.5,(.28,-.08,1.25),Rx,FINGER_CLOSED,'roll strip above case'),
                           W(2.5,(.337,.24,1.25),Rx,FINGER_CLOSED,'roll free end toward handle'),
                           W(1.5,(.337,.24,1.25),Rx,FINGER_CLOSED,'settle before feed')], 'pre-feed roll')
            run.save({'stage':'midstrip_transport_finished_not_threaded','phases':reports})
        # Replay the validated robot commands in one continuous fresh rollout.
        # These are motion targets, never saved object states or physics resets.
        replay = os.environ.get('BAGTAG_REPLAY_COMMANDS')
        if replay:
            replay_limit = int(os.environ.get('BAGTAG_REPLAY_LIMIT', '999'))
            for path in sorted(Path(replay).glob('*.json')):
                cmd = json.loads(path.read_text())
                if int(cmd['id']) > replay_limit:
                    continue
                # Read actual paper position for the receiving grasp. This changes
                # robot targets only; it never moves the paper by assignment.
                if cmd.get('measure_panel') is not None:
                    centre=np.asarray(run.state()['panel_xyz'][int(cmd['measure_panel'])])
                    delta=centre-np.array([.013,0,0])-np.asarray(cmd['waypoints'][-1]['pos'])
                    for w in cmd['waypoints']:w['pos']=(np.asarray(w['pos'])+delta).tolist()
                # Grasp a hanging panel across its measured faces: approach along +x
                # projected into the panel plane, pads closing along the measured normal.
                if cmd.get('measure_panel_frame') is not None:
                    from scipy.spatial.transform import Rotation
                    state=run.state(); k=int(cmd['measure_panel_frame'])
                    c=np.asarray(state['panel_xyz'][k]); Rp=Rotation.from_quat(state['panel_quat_xyzw'][k]).as_matrix()
                    n=Rp[:,2]; a=np.array([1.,0,0])-n*n[0]; a/=np.linalg.norm(a)
                    axis_name=cmd.get('approach_panel_axis')
                    if axis_name: a=(-1. if axis_name[0]=='-' else 1.)*Rp[:,'xyz'.index(axis_name[-1])]
                    if cmd.get('approach_panel_deg') is not None:
                        th=np.radians(float(cmd['approach_panel_deg'])); a=np.cos(th)*Rp[:,0]+np.sin(th)*Rp[:,1]
                    side=cmd['side']; q0=current(side)[:6]; best=None
                    final=c-a*float(cmd.get('tcp_back_m',.003))+Rp[:,0]*float(cmd.get('along_panel_m',0.))
                    sign=int(cmd.get('closing_sign',0))
                    for cl in ((n,-n) if sign==0 else ((n,) if sign>0 else (-n,))):
                        Rg=frame(a,cl)
                        _,pe,re=ik.solve(final,ik.matrix_to_quat_xyzw(Rg),q_init=q0,base_pos=station.BASES[side],iters=400)
                        if best is None or pe+.1*re<best[0]:best=(pe+.1*re,Rg)
                    Rg=best[1]
                    cmd['waypoints']=[dict(duration=t,pos=p.tolist(),approach=Rg[:,2].tolist(),closing=Rg[:,0].tolist(),finger=f,label=label)
                        for t,p,f,label in [(float(cmd.get('approach_s',1.5)),final-.07*a,float(cmd.get('open_finger',-.012)),'line up with the measured panel'),
                                            (1.5,final,float(cmd.get('open_finger',-.012)),'straddle the measured panel'),
                                            (1.,final,0.,'pinch across both faces'),(.8,final,0.,'hold the panel')]]
                if cmd.get('rotate_held_endpoint'):
                    p0,R0=ik.tcp_pose(current('right')[:6],station.BASES['right'])
                    turn_R=frame([.5,.8660254,0],[0,0,1])
                    cmd['waypoints']=[dict(duration=t,pos=p0.tolist(),approach=R[:,2].tolist(),closing=R[:,0].tolist(),finger=f,label=label)
                        for t,R,f,label in [(.4,R0,-.004,'loosen before changing approach'),(2.,turn_R,-.004,'turn open jaws around supported endpoint'),(.5,turn_R,0.,'regrasp without twisting strip'),(.4,turn_R,0.,'settle')]]
                if cmd.get('press_endpoint_pair'):
                    from scipy.spatial.transform import Rotation
                    state=run.state();xyz=np.array(state['panel_xyz'])
                    normal=Rotation.from_quat(state['panel_quat_xyzw'][0]).as_matrix()[:,2]
                    p0,_=ik.tcp_pose(current('right')[:6],station.BASES['right'])
                    target=p0+xyz[-1]+.0012*normal-xyz[0];pre=target-np.array([0,.045,0])
                    press_R=frame([.5,.8660254,0],[0,0,1])
                    stage('right',[W(2.,pre,press_R,0.,'approach second layer'),W(.3,pre,press_R,-.004,'open for two layers')],'open receiving jaws')
                    state=run.state();xyz=np.array(state['panel_xyz'])
                    normal=Rotation.from_quat(state['panel_quat_xyzw'][0]).as_matrix()[:,2]
                    p0,_=ik.tcp_pose(current('right')[:6],station.BASES['right'])
                    target=p0+xyz[-1]+.0012*normal-xyz[0]
                    stage('right',[W(1.5,target,press_R,-.004,'align two layers using measured poses')],'align inside receiving jaws')
                    g=run.state()['closure']
                    if g['overlap_m2']<.0006 or g['normal_dot']<.985:
                        raise RuntimeError('Layers not aligned inside jaws; do not close')
                    cmd['waypoints']=[dict(duration=t,pos=target.tolist(),approach=[.5,.8660254,0],closing=[0,0,1],finger=0.,label=label) for t,label in
                        [(.8,'deliberately pinch aligned patches'),(3.,'sustain pinch at sealed overlap before release')]]
                # Lay one held panel's adhesive back onto another panel, closed loop on measured
                # poses (robot targets only), then press it on by commanded overtravel.
                if cmd.get('place_held_panel'):
                    from scipy.spatial.transform import Rotation
                    side=cmd['side']; held=int(cmd['held_panel']); tgt=int(cmd['target_panel'])
                    cmd_p,cmd_R=ik.tcp_pose(current(side)[:6],station.BASES[side])
                    for k in range(int(cmd.get('passes',3))):
                        st=run.state(); xs=np.asarray(st['panel_xyz'])
                        zt=Rotation.from_quat(st['panel_quat_xyzw'][tgt]).as_matrix()[:,2]
                        desired=xs[tgt]-float(cmd.get('gap_m',.0015))*zt
                        delta=desired-xs[held]
                        reports.append({'phase':f'place pass {k}','held_minus_desired_m':(-delta).tolist()})
                        cmd_p=cmd_p+delta
                        stage(side,[W(1.2,cmd_p,cmd_R,0.,f'lay panel{held:02d} on panel{tgt:02d} (pass {k})'),
                                    W(.5,cmd_p,cmd_R,0.,'settle')],f'place panel{held:02d} pass {k}')
                    st=run.state(); zt=Rotation.from_quat(st['panel_quat_xyzw'][tgt]).as_matrix()[:,2]
                    cmd['waypoints']=[dict(duration=t,pos=(cmd_p+zt*float(cmd.get('press_m',.004))).tolist(),approach=cmd_R[:,2].tolist(),
                                           closing=cmd_R[:,0].tolist(),finger=0.,label=label)
                                      for t,label in [(.8,'press the adhesive back on'),(2.,'hold the press')]]
                # Press the stacked strip down with the closed fingers' flat underside at a
                # measured stack height (robot targets only; the lid or held strip resists).
                if cmd.get('press_down_at') is not None:
                    side=cmd['side']; st=run.state(); xs=np.asarray(st['panel_xyz'])
                    if cmd.get('press_over_panel') is not None:
                        xy=xs[int(cmd['press_over_panel'])][:2]+np.asarray(cmd.get('press_offset_xy',[0.,0.]),float)
                    else:
                        xy=np.asarray(cmd['press_down_at'],float)
                    a=np.asarray(cmd.get('approach',[1,0,0]),float); a/=np.linalg.norm(a)
                    cl=np.asarray(cmd.get('closing',[0,0,1]),float)
                    tip=float(cmd.get('tip_offset_m',0.)); half=float(cmd.get('finger_half_m',.004)); ov=float(cmd.get('overtravel_m',.004))
                    if cmd.get('reference_panel') is not None:
                        # Press down to the supported lower layer plus the layers above it.
                        z_top=float(xs[int(cmd['reference_panel'])][2])+.0005+.001*float(cmd.get('layers',1))
                        near=[]
                    else:
                        near=[x for x in xs if np.linalg.norm(x[:2]-xy)<.035]
                        z_top=(max(x[2] for x in near) if near else float(cmd.get('fallback_z',1.08)))+.0005
                    tcp_xy=xy-tip*a[:2]
                    zc=z_top+half-tip*a[2]
                    reports.append({'phase':'press stack','press_xy':xy.tolist(),'stack_top_m':z_top,'panels_near':len(near),'approach':a.tolist()})
                    cmd['waypoints']=[dict(duration=t,pos=[float(tcp_xy[0])-float(back)*a[0],float(tcp_xy[1])-float(back)*a[1],float(z)],approach=a.tolist(),closing=cl.tolist(),finger=0.,label=label)
                                      for t,back,z,label in [(2.,.05,zc+.05,'come in from the side above the seal'),(1.2,0.,zc+.01,'above the stacked layers'),
                                                             (1.,0.,zc+.001,'touch the stack'),(.8,0.,zc-ov,'press the adhesive layers together'),
                                                             (2.,0.,zc-ov,'hold the press'),(1.,.04,zc+.05,'lift off and back out sideways')]]
                # Slide a held panel into the other hand's open jaws, parallel to the panel
                # already there and gap_m above its front: measured poses, robot targets only.
                if cmd.get('insert_panel_into_jaws'):
                    from scipy.spatial.transform import Rotation
                    side=cmd['side']; held=int(cmd['held_panel']); tgt=int(cmd['target_panel'])
                    cmd_p,cmd_R=ik.tcp_pose(current(side)[:6],station.BASES[side])
                    def plan(offset):
                        st=run.state(); xs=np.asarray(st['panel_xyz'])
                        Rt=Rotation.from_quat(st['panel_quat_xyzw'][tgt]).as_matrix()
                        Rh=Rotation.from_quat(st['panel_quat_xyzw'][held]).as_matrix()
                        tp,tR=ik.tcp_pose(current(side)[:6],station.BASES[side])
                        Rc=Rt@Rh.T                                   # turn the held panel parallel to the target
                        r=tR.T@(xs[held]-tp)                         # held panel centre in the hand frame
                        newR=Rc@tR
                        goal=xs[tgt]+float(cmd.get('gap_m',.0012))*Rt[:,2]+np.asarray(offset,float)
                        new_tcp=goal-newR@r
                        err=float(np.linalg.norm(goal-xs[held]))
                        ang=float(np.degrees(np.arccos(np.clip((np.trace(Rc)-1)/2,-1,1))))
                        return new_tcp-tp,newR,err,ang
                    for label,offset,n in (('line up beside the jaws',cmd.get('pre_offset',[0,.04,0]),2),('slide into the jaws',[0,0,0],int(cmd.get('passes',2)))):
                        for k in range(n):
                            d,newR,err,ang=plan(offset)
                            reports.append({'phase':f'insert {label} pass {k}','position_error_m':err,'angle_error_deg':ang})
                            cmd_p=cmd_p+d; cmd_R=newR
                            stage(side,[W(1.5,cmd_p,cmd_R,0.,f'{label} (pass {k})'),W(.5,cmd_p,cmd_R,0.,'settle')],f'insert panel{held:02d}: {label} {k}')
                    d,newR,err,ang=plan([0,0,0])
                    reports.append({'phase':'insert final check','position_error_m':err,'angle_error_deg':ang})
                    cmd['waypoints']=[dict(duration=.5,pos=cmd_p.tolist(),approach=cmd_R[:,2].tolist(),closing=cmd_R[:,0].tolist(),finger=0.,label='hold the layer inside the jaws')]
                # Crossed back-to-back seal held by both hands: lay the left's panel (back down) across the
                # free half of the right's panel (back up) and press it on. Measured poses, robot targets only.
                if cmd.get('cross_press'):
                    from scipy.spatial.transform import Rotation
                    side=cmd['side']; held=int(cmd['held_panel']); tgt=int(cmd['target_panel'])
                    HLc=__import__('tag_dims').SEGMENT_LENGTH/2
                    def Tm(p,R):
                        M=np.eye(4); M[:3,:3]=R; M[:3,3]=p; return M
                    def desired():
                        st=run.state(); xs=np.asarray(st['panel_xyz'])
                        P0=Tm(xs[tgt],Rotation.from_quat(st['panel_quat_xyzw'][tgt]).as_matrix())
                        free_c=P0[:3,3]-float(cmd['tip_sign'])*HLc/2*P0[:3,0]
                        x8=Rotation.from_rotvec(np.radians(float(cmd['cross_deg']))*P0[:3,2]).apply(P0[:3,0]); n8=-P0[:3,2]
                        gh=float(cmd['grip_half'])
                        c8=free_c+gh*HLc/2*x8+float(cmd.get('shift',0.))*np.cross(P0[:3,2],x8)-float(cmd.get('gap_m',.0011))*P0[:3,2]
                        Ph=Tm(xs[held],Rotation.from_quat(st['panel_quat_xyzw'][held]).as_matrix())
                        tp,tR=ik.tcp_pose(current(side)[:6],station.BASES[side])
                        rel=np.linalg.inv(Tm(tp,tR))@Ph                  # measured panel-in-hand
                        P8=Tm(c8,np.column_stack((x8,np.cross(n8,x8),n8)))
                        goal=P8@np.linalg.inv(rel)
                        return goal, tp, P0[:3,2], float(np.linalg.norm(c8-xs[held]))
                    cmd_p,cmd_R=ik.tcp_pose(current(side)[:6],station.BASES[side])
                    if cmd.get('via_z') is not None:
                        goal,tp,up,err=desired()
                        via=np.array([goal[0,3],goal[1,3],float(cmd['via_z'])])
                        stage(side,[W(2.5,np.array([tp[0],tp[1],float(cmd['via_z'])]),goal[:3,:3],0.,'rise above the handle'),
                                    W(2.5,via,goal[:3,:3],0.,'carry the tail over the seal')],f'cross panel{held:02d}: carry over')
                        cmd_p,cmd_R=via,goal[:3,:3]
                    for label,lift,n in (('line up above the seal',.03,2),('lay across the free half',0.,2)):
                        for k in range(n):
                            goal,tp,up,err=desired()
                            reports.append({'phase':f'cross {label} pass {k}','held_to_goal_m':err})
                            cmd_p=cmd_p+(goal[:3,3]+lift*up-tp); cmd_R=goal[:3,:3]
                            stage(side,[W(1.5,cmd_p,cmd_R,0.,f'{label} (pass {k})'),W(.5,cmd_p,cmd_R,0.,'settle')],f'cross panel{held:02d}: {label} {k}')
                    goal,tp,up,err=desired()
                    press=cmd_p-up*float(cmd.get('press_m',.004))
                    cmd['waypoints']=[dict(duration=t_,pos=p_.tolist(),approach=cmd_R[:,2].tolist(),closing=cmd_R[:,0].tolist(),finger=0.,label=lab)
                                      for t_,p_,lab in [(.8,press,'press the crossed adhesive backs together'),(2.,press,'hold the press')]]
                # Move relative to the current measured hand pose, keeping its orientation.
                if cmd.get('relative_move') is not None:
                    side=cmd['side']; p0_,R0_=ik.tcp_pose(current(side)[:6],station.BASES[side])
                    pts=[p0_+np.asarray(d,float) for d in cmd['relative_move']]
                    cmd['waypoints']=[dict(duration=float(cmd.get('seconds',2.)),pos=p.tolist(),approach=R0_[:,2].tolist(),closing=R0_[:,0].tolist(),
                                           finger=float(cmd.get('finger',0.)),label=cmd.get('name','relative move')) for p in pts]
                # Loosen, slide along the held panel to the crossed overlap, and pinch both layers.
                if cmd.get('slide_pinch'):
                    from scipy.spatial.transform import Rotation
                    side=cmd['side']; st=run.state(); k=int(cmd['panel'])
                    xk=Rotation.from_quat(st['panel_quat_xyzw'][k]).as_matrix()[:,0]
                    p0_,R0_=ik.tcp_pose(current(side)[:6],station.BASES[side])
                    p1_=p0_+float(cmd['slide_m'])*xk
                    W_=lambda t_,p,f,lab: dict(duration=t_,pos=p.tolist(),approach=R0_[:,2].tolist(),closing=R0_[:,0].tolist(),finger=f,label=lab)
                    cmd['waypoints']=[W_(.6,p0_,float(cmd.get('open_finger',-.005)),'loosen to take a second layer'),
                                      W_(1.5,p1_,float(cmd.get('open_finger',-.005)),'slide along the panel to the overlap'),
                                      W_(.8,p1_,0.,'pinch both adhesive layers'),W_(2.5,p1_,0.,'hold the pinch')]
                # Lay the right's end panel (back down) across the back of a panel of the other strand that
                # no hand holds; measured poses, closed loop on the commanded TCP.
                if cmd.get('end_on_panel'):
                    from scipy.spatial.transform import Rotation
                    side=cmd['side']; held=int(cmd['held_panel']); HLc=__import__('tag_dims').SEGMENT_LENGTH/2
                    def Tm(p,R):
                        M=np.eye(4); M[:3,:3]=R; M[:3,3]=p; return M
                    st=run.state(); xs=np.asarray(st['panel_xyz'])
                    lp,_=ik.tcp_pose(current('left' if side=='right' else 'right')[:6],station.BASES['left' if side=='right' else 'right'])
                    from handle_route_cpu import best_ik as _best_ik, obstacle_points as _op
                    from exit_guide_pickup_cpu import inside_any as _inside
                    import yam_hulls as _yh
                    from scipy.spatial import cKDTree as _KD
                    import tag_closure as _tc
                    _obs=_op(.008); _o='left' if side=='right' else 'right'
                    _scene=_KD(np.vstack([v_ for kk_,v_ in _obs.items() if kk_ in ('case','handle','kiosk')]))
                    def select(tag):
                        """Every candidate panel and crossing angle, checked from the measured state: exact IK at
                        2 cm above and on the panel, no scene penetration, clearance to the other arm, overlap.
                        Returns the candidate with the largest worst-case clearance."""
                        st=run.state(); xs=np.asarray(st['panel_xyz']); Rs=np.array([Rotation.from_quat(q_).as_matrix() for q_ in st['panel_quat_xyzw']])
                        other=_KD(np.vstack([h.vertices for pcs in _yh.hulls(current(_o),_o).values() for h in pcs]))
                        tpn,tRn=ik.tcp_pose(current(side)[:6],station.BASES[side])
                        rel0=np.linalg.inv(Tm(tpn,tRn))@Tm(xs[held],Rs[held]); tip0=float(np.sign(np.linalg.inv(rel0)[0,3]) or 1.)
                        options=[]
                        for k_ in cmd['candidates']:
                            back=-Rs[k_][:,2]
                            for cr in range(0,360,30):
                                x0=Rotation.from_rotvec(np.radians(cr)*Rs[k_][:,2]).apply(Rs[k_][:,0])
                                c0=xs[k_]+.0012*back+tip0*HLc/2*x0
                                P0=Tm(c0,np.column_stack((x0,np.cross(back,x0),back)))
                                xx=xs.copy(); RR=Rs.copy(); xx[held]=c0; RR[held]=P0[:3,:3]
                                ov=_tc.pair_geometry(xx,RR,held,k_)['overlap_m2']
                                if ov<_tc.MIN_OVERLAP_M2: continue
                                goal=P0@np.linalg.inv(rel0); margin=1.
                                for lift in (.02,0.):
                                    qq,pe,re=_best_ik(side,goal[:3,3]+lift*back,goal[:3,:3])
                                    if pe>.002 or re>.03: margin=-1; break
                                    vv=np.vstack([h.vertices for pcs in _yh.hulls(np.r_[qq,0.,0.],side).values() for h in pcs])
                                    hs_=[h for pcs in _yh.hulls(np.r_[qq,0.,0.],side).values() for h in pcs]
                                    if any(_inside(v_,hs_).any() for kk_,v_ in _obs.items() if kk_ in ('case','handle','kiosk')): margin=-1; break
                                    margin=min(margin,float(other.query(vv)[0].min()),float(_scene.query(vv)[0].min()))
                                if margin>=float(cmd.get('min_margin_m',.006)):
                                    travel=float(np.linalg.norm(c0-xs[held]))
                                    options.append((-travel if cmd.get('prefer_short') else margin,int(k_),cr,round(ov*1e6)))
                        reports.append({'phase':f'end on panel options ({tag})','options':[[o[1],o[2],round(o[0]*1e3,1),o[3]] for o in sorted(options,reverse=True)[:12]]})
                        if not options:
                            raise RuntimeError(f'No reachable collision-free panel/crossing ({tag})')
                        m,k_,cr,ov=max(options)
                        reports.append({'phase':f'end on panel target ({tag})','panel':k_,'cross_deg':cr,'margin_mm':round(m*1e3,1),'overlap_mm2':ov})
                        return k_,cr
                    tgt,cmd['cross_deg']=select('before turn-over')
                    cmd['chosen_panel']=int(tgt)
                    def desired():
                        st=run.state(); xs=np.asarray(st['panel_xyz'])
                        Pk=Tm(xs[tgt],Rotation.from_quat(st['panel_quat_xyzw'][tgt]).as_matrix()); back=-Pk[:3,2]
                        Ph=Tm(xs[held],Rotation.from_quat(st['panel_quat_xyzw'][held]).as_matrix())
                        tp,tR=ik.tcp_pose(current(side)[:6],station.BASES[side])
                        rel=np.linalg.inv(Tm(tp,tR))@Ph
                        tip=float(np.sign((np.linalg.inv(rel))[0,3]) or 1.)
                        x0=Rotation.from_rotvec(np.radians(float(cmd.get('cross_deg',180)))*Pk[:3,2]).apply(Pk[:3,0])
                        c0=Pk[:3,3]+float(cmd.get('gap_m',.0012))*back+tip*HLc/2*x0
                        P0=Tm(c0,np.column_stack((x0,np.cross(back,x0),back)))
                        return P0@np.linalg.inv(rel),tp,back,float(np.linalg.norm(c0-xs[held]))
                    cmd_p,cmd_R=ik.tcp_pose(current(side)[:6],station.BASES[side])
                    # The end-over turn (~180 deg) goes through joint space, collision-checked; a straight
                    # Cartesian slerp would pass through unreachable orientations.
                    q6=float(current(side)[5]); delta=np.pi if q6+np.pi<=2.09 else -np.pi
                    reports.append({'phase':'end on panel: turn over by wrist roll','q6_from':q6,'delta':float(delta)})
                    roll_in_place(side,delta,0.,'turn the end over in place (wrist roll)')
                    # The strip moves while the end is turned over: plan again from the measured state.
                    tgt,cmd['cross_deg']=select('after turn-over')
                    cmd['chosen_panel']=int(tgt)
                    cmd_p,cmd_R=ik.tcp_pose(current(side)[:6],station.BASES[side])
                    for label,lift,n in (('line up above the panel',.02,1),('lay the end on the panel',0.,2)):
                        for k in range(n):
                            goal,tp,back,err=desired()
                            reports.append({'phase':f'end on panel: {label} {k}','held_to_goal_m':err})
                            cmd_p=cmd_p+(goal[:3,3]+lift*back-tp); cmd_R=goal[:3,:3]
                            try:
                                stage(side,[W(1.5,cmd_p,cmd_R,0.,f'{label} ({k})'),W(.5,cmd_p,cmd_R,0.,'settle')],f'end on panel{tgt:02d}: {label} {k}')
                            except RuntimeError as exc:
                                if 'Preflight' not in str(exc): raise
                                # Straight-line orientation path unreachable: same measured target through joint space.
                                reports.append({'phase':f'end on panel: {label} {k} via joint space','reason':str(exc)[:120]})
                                move_joint_target(side,goal[:3,3]+lift*back,goal[:3,:3],0.,f'{label} ({k}, joint space)')
                                cmd_p,cmd_R=ik.tcp_pose(current(side)[:6],station.BASES[side])
                    goal,tp,back,err=desired()
                    cmd['waypoints']=[dict(duration=.8,pos=(cmd_p-back*float(cmd.get('press_m',.002))).tolist(),approach=cmd_R[:,2].tolist(),closing=cmd_R[:,0].tolist(),finger=0.,label='light press')]
                # Let go of the held panel and pinch the two-layer overlap of a bonded-to-be pair.
                if cmd.get('pinch_overlap'):
                    from scipy.spatial.transform import Rotation
                    import tag_closure as _tc
                    side=cmd['side']; a_k=int(cmd['panel_a']); b_k=cmd.get('panel_b')
                    if b_k=='chosen':
                        prev=[r for r in reports if r.get('phase')=='end on panel target']; b_k=prev[-1]['panel']
                    b_k=int(b_k)
                    st=run.state(); xs=np.asarray(st['panel_xyz']); Rs=np.array([Rotation.from_quat(q_).as_matrix() for q_ in st['panel_quat_xyzw']])
                    g=_tc.pair_geometry(xs,Rs,a_k,b_k)
                    # overlap centre: midpoint of the two panel centres projected (good enough for a 2 cm pad)
                    centre=(xs[a_k]+xs[b_k])/2; n=Rs[b_k][:,2]
                    base=np.asarray(station.BASES[side]); best=None
                    for ang in range(0,360,20):
                        a=np.cos(np.radians(ang))*Rs[b_k][:,0]+np.sin(np.radians(ang))*Rs[b_k][:,1]
                        for sgn in (1,-1):
                            Rg=frame(a,sgn*n); p=centre-.004*a
                            qq,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(Rg),q_init=current(side)[:6],base_pos=base,iters=300)
                            sc=pe+.1*re-.002*float(np.dot(a,(centre-base)/np.linalg.norm(centre-base)))
                            if best is None or sc<best[0]: best=(sc,a,Rg,p,pe,re)
                    _,a,Rg,p,pe,re=best
                    reports.append({'phase':'pinch overlap plan','pair':[a_k,b_k],'overlap_m2':g['overlap_m2'],'ik_mm':pe*1e3})
                    p_now,R_now=ik.tcp_pose(current(side)[:6],station.BASES[side])
                    Wd=lambda t_,pp,RR,f,lab: dict(duration=t_,pos=np.asarray(pp).tolist(),approach=RR[:,2].tolist(),closing=RR[:,0].tolist(),finger=f,label=lab)
                    cmd['waypoints']=[Wd(.8,p_now,R_now,-.03,'let go of the tail'),Wd(1.5,p_now-.06*R_now[:,2],R_now,-.03,'back away'),
                                      Wd(2.5,p-.06*a,Rg,-.02,'line up with the overlap'),Wd(1.5,p,Rg,-.02,'straddle both layers'),
                                      Wd(.8,p,Rg,0.,'pinch both adhesive layers'),Wd(2.5,p,Rg,0.,'hold the pinch')]
                # Closed-loop tip feed: keep the measured leading tip on the lid and advance it in small steps,
                # so it can only pass through the gap under the handle; stop if it rises to go over the bar.
                if cmd.get('tip_feed'):
                    from scipy.spatial.transform import Rotation
                    side=cmd['side']; HLc=__import__('tag_dims').SEGMENT_LENGTH/2
                    lid_z=float(cmd.get('lid_z',1.068)); edge=float(cmd.get('case_edge_y',-.086))
                    def tip():
                        st=run.state(); R0_=Rotation.from_quat(st['panel_quat_xyzw'][0]).as_matrix()
                        return np.asarray(st['panel_xyz'][0])+HLc*R0_[:,0]
                    tp_,R_=ik.tcp_pose(current(side)[:6],station.BASES[side])
                    targets=[float(cmd.get('start_y',.05))]*int(cmd.get('settle_steps',4))+list(np.arange(float(cmd.get('start_y',.05)),float(cmd.get('end_y',-.12))-1e-6,-float(cmd.get('step_m',.02))))
                    stuck=0; prev=tip()
                    for k_,ty in enumerate(targets):
                        t_=tip(); settling=k_<int(cmd.get('settle_steps',4))
                        if settling and abs(t_[1]-ty)<.01 and abs(t_[2]-lid_z)<.004:
                            continue                           # tip already resting on the lid at the start
                        if not settling and abs(t_[1])<.03 and t_[2]>float(cmd.get('over_bar_z',1.088)):
                            raise RuntimeError(f'Tip rising to pass over the handle (tip z {t_[2]:.3f} at y {t_[1]:.3f})')
                        tz=lid_z if t_[1]>edge else t_[2]
                        if settling and t_[1]<ty-.01:
                            d_=np.array([0.,ty-t_[1],0.])      # still beyond the start: draw the tip back first, above the bar
                        else:
                            d_=np.array([0.,ty-t_[1],(tz-t_[2]) if t_[1]>edge else 0.])
                        d_=np.clip(d_,-.03,.03)
                        if not settling and cmd.get('feed_tcp_z') is not None:
                            # Keep the hand low so the strip behind the tip lies along the lid, not in a buckling arch.
                            d_[2]=float(np.clip(float(cmd['feed_tcp_z'])-tp_[2],-.03,.03))
                        tp_=tp_+d_
                        if not settling and tp_[1]<float(cmd.get('min_tcp_y',.07)):
                            raise RuntimeError(f'Hand reached the feed limit with the tip at {t_.round(3).tolist()}')
                        stage(side,[W(float(cmd.get('seconds',.8)),tp_,R_,0.,f'advance tip to y {ty:.2f}')],f'tip feed {k_}')
                        now=tip(); reports.append({'phase':f'tip feed {k_}','target_y':float(ty),'tip':np.round(now,4).tolist(),'tcp':np.round(tp_,4).tolist()})
                        stuck = stuck+1 if (not settling and np.linalg.norm(now-prev)<.003) else 0; prev=now
                        if stuck>=3: raise RuntimeError(f'Tip not advancing at {now.round(3).tolist()}')
                        if not settling and now[1]<=float(cmd.get('stop_tip_y',-.105)):
                            break                              # enough tip beyond the case edge for the receiving hand
                    cmd['waypoints']=[dict(duration=.8,pos=tp_.tolist(),approach=R_[:,2].tolist(),closing=R_[:,0].tolist(),finger=0.,label='hold the fed tip for the receiving hand')]
                # Pinch both overhanging layers in one jaw: open around the held upper end, centre the jaws between
                # it and the lower layer's measured end (both past the case edge), close and hold.
                if cmd.get('pinch_layers'):
                    side=cmd['side']; st=run.state(); xs=np.asarray(st['panel_xyz'])
                    up=xs[int(cmd.get('upper_panel',0))]
                    cand=[k_ for k_ in range(int(cmd.get('lower_from',len(xs)//2)),len(xs)) if xs[k_][2]<up[2]-.002]
                    if not cand: raise RuntimeError('No lower layer below the held end')
                    lo_k=min(cand,key=lambda k_:abs(xs[k_][1]-up[1])+abs(xs[k_][2]-up[2])); lo=xs[lo_k]
                    p_,R_=ik.tcp_pose(current(side)[:6],station.BASES[side])
                    centre=np.array([p_[0], lo[1]*.5+up[1]*.5, (up[2]+lo[2])/2])
                    if cmd.get('centre_above_lower_m') is not None:   # bias the jaw so the lower finger passes under the lower layer
                        centre=np.array([p_[0], lo[1], lo[2]+float(cmd['centre_above_lower_m'])])
                    reports.append({'phase':'pinch layers plan','upper':np.round(up,4).tolist(),'lower':np.round(lo,4).tolist(),'jaw_centre':np.round(centre,4).tolist(),
                                    'layer_gap_m':float(up[2]-lo[2])})
                    if abs(up[2]-lo[2])>float(cmd.get('max_gap_m',.06)):
                        raise RuntimeError(f'Layers {up[2]-lo[2]:.3f} m apart: too far for one jaw')
                    op=float(cmd.get('open_finger',-.035))
                    cmd['waypoints']=[dict(duration=t_,pos=pp.tolist(),approach=R_[:,2].tolist(),closing=R_[:,0].tolist(),finger=f_,label=l_)
                                      for t_,pp,f_,l_ in ([(.8,p_,op,'open around both layers'),(1.2,centre,op,'centre the jaws on both layers'),
                                                          (1.,centre,0.,'pinch both adhesive layers together'),(2.,centre,0.,'hold the pinch')] if not cmd.get('side_entry') else
                                                         [(.8,p_,op,'open the jaws'),(1.2,p_-.08*R_[:,2],op,'back out across the width'),
                                                          (1.2,centre-.08*R_[:,2],op,'line up beside both layers'),(1.5,centre,op,'slide the open jaws over both layers'),
                                                          (1.,centre,0.,'pinch both adhesive layers together'),(2.,centre,0.,'hold the pinch')])]
                # Pinch the two layers overhanging the case edge: open jaws enter along the strip from outside
                # the edge (approach given), the lower finger passing under the measured bottom layer.
                if cmd.get('overhang_pinch'):
                    side=cmd['side']; st=run.state(); xs=np.asarray(st['panel_xyz'])
                    y0,y1=cmd.get('y_range',[.088,.14])
                    over=[x for x in xs if y0<x[1]<y1 and x[2]>.98]
                    if len(over)<2:
                        raise RuntimeError(f'Fewer than two panels overhang the edge in y {y0}-{y1}: {len(over)}')
                    top=max(over,key=lambda x:x[2]); bot=min(over,key=lambda x:x[2])
                    a=np.asarray(cmd['approach'],float); a/=np.linalg.norm(a); Rp=frame(a,cmd.get('closing',[0,0,1]))
                    centre=np.array([float(np.mean([x[0] for x in over])),(top[1]+bot[1])/2,(top[2]+bot[2])/2+float(cmd.get('z_bias_m',0.))])
                    P=centre-float(cmd.get('tip_depth_m',.0))*a
                    reports.append({'phase':'overhang pinch plan','top':np.round(top,4).tolist(),'bottom':np.round(bot,4).tolist(),'centre':np.round(centre,4).tolist(),'n_over':len(over)})
                    op=float(cmd.get('open_finger',-.04))
                    for j_,v_ in enumerate(cmd.get('via_tcp',[])):
                        move_joint_target(side,np.asarray(v_,float),Rp,op,f'approach from above ({j_})')
                    move_joint_target(side,P-.09*a,Rp,op,'line up outside the case edge')
                    cmd['waypoints']=[dict(duration=t_,pos=pp.tolist(),approach=a.tolist(),closing=Rp[:,0].tolist(),finger=f_,label=l_)
                                      for t_,pp,f_,l_ in [(1.8,P,op,'slide the open jaws over both layers'),(1.,P,0.,'pinch both adhesive layers together'),
                                                          (2.,P,0.,'hold the pinch'),(.8,P,op,'open'),(1.5,P-.10*a,op,'withdraw straight out')]]
                # Go to the ready pose through a collision-checked joint route (no Cartesian path needed).
                if cmd.get('joint_ready'):
                    side=cmd['side']; pr,Rr_=ik.tcp_pose(READY,station.BASES[side])
                    p_,R_=ik.tcp_pose(current(side)[:6],station.BASES[side])
                    for j_,v_ in enumerate([list(p_+np.asarray(d_,float)) for d_ in cmd.get('via_rel',[])]+list(cmd.get('via_tcp',[]))):
                        try:
                            move_joint_target(side,np.asarray(v_,float),R_,-.047,f'clear the tag ({j_})')
                        except RuntimeError:
                            move_joint_target(side,np.asarray(v_,float),Rr_,-.047,f'clear the tag ({j_}, ready orientation)')
                    move_joint_target(side,pr,Rr_,-.047,'return ready (joint route)')
                    cmd['waypoints']=[dict(duration=.5,pos=pr.tolist(),approach=Rr_[:,2].tolist(),closing=Rr_[:,0].tolist(),finger=-.047,label='hold ready')]
                # Turn only the wrist roll joint (in place), e.g. to turn a held end over.
                if cmd.get('wrist_roll') is not None:
                    roll_in_place(cmd['side'],float(cmd['wrist_roll']),float(cmd.get('finger',0.)),cmd.get('name','wrist roll'))
                    p_,R_=ik.tcp_pose(current(cmd['side'])[:6],station.BASES[cmd['side']])
                    cmd['waypoints']=[dict(duration=.5,pos=p_.tolist(),approach=R_[:,2].tolist(),closing=R_[:,0].tolist(),finger=float(cmd.get('finger',0.)),label='settle after roll')]
                # Move a held panel to a target pose (centre + axes) through collision-checked joint routes,
                # using the measured grasp (panel in hand) at command time.
                if cmd.get('held_panel_to') is not None:
                    from scipy.spatial.transform import Rotation
                    side=cmd['side']; k_=int(cmd['held_panel'])
                    def Tm(p,R):
                        M=np.eye(4); M[:3,:3]=R; M[:3,3]=p; return M
                    st=run.state(); tp_,tR_=ik.tcp_pose(current(side)[:6],station.BASES[side])
                    rel=np.linalg.inv(Tm(tp_,tR_))@Tm(np.asarray(st['panel_xyz'][k_]),Rotation.from_quat(st['panel_quat_xyzw'][k_]).as_matrix())
                    Rt=np.column_stack([np.asarray(v,float) for v in cmd['axes']])
                    for j_,c_ in enumerate(cmd['held_panel_to']):
                        goal=Tm(np.asarray(c_,float),Rt)@np.linalg.inv(rel)
                        move_joint_target(side,goal[:3,3],goal[:3,:3],0.,f"{cmd.get('name','move held panel')} ({j_})")
                    p_,R_=ik.tcp_pose(current(side)[:6],station.BASES[side])
                    cmd['waypoints']=[dict(duration=.8,pos=p_.tolist(),approach=R_[:,2].tolist(),closing=R_[:,0].tolist(),finger=0.,label='hold the end in place')]
                # Press the stacked layers onto the lid with the closed fingertips (approach given), measured stack top.
                if cmd.get('fingertip_press') is not None and cmd.get('repeat_last') and getattr(run,'_last_press',None) is not None:
                    side=cmd['side']; P,Rp,a_tilt=run._last_press; dn=np.array([0.,0.,-1.]); ov=float(cmd.get('overtravel_m',.01))
                    reports.append({'phase':'repeat the press at the same spot','P':np.round(P,4).tolist()})
                    cmd['waypoints']=[dict(duration=t_,pos=p_.tolist(),approach=Rp[:,2].tolist(),closing=Rp[:,0].tolist(),finger=0.,label=l_)
                                      for t_,p_,l_ in [(1.5,P,'come back down onto the tacked layers'),(.8,P+ov*dn,'press firmly'),
                                                      (float(cmd.get('hold_s',2.5)),P+ov*dn,'hold the firm press'),(1.,P-float(cmd.get('lift_back_m',.05))*a_tilt,'lift the fingertips off')]]
                elif cmd.get('fingertip_press') is not None:
                    side=cmd['side']
                    run.go(int(cmd.get('settle_steps',10)),hold('left'),hold('right'),snaps=(run.step_i+10,),video_every=4)   # let the strip come to rest
                    st=run.state(); xs=np.asarray(st['panel_xyz'])
                    xy=np.asarray(cmd['fingertip_press'],float)
                    near=[x for x in xs if abs(x[1]-xy[1])<float(cmd.get('search_r',.04)) and abs(x[0]-xy[0])<.09 and x[2]<float(cmd.get('max_layer_z',1.25))]
                    lid=[x for x in near if x[2]<1.075]
                    up=[x for x in near if x[2]>=1.075]
                    if cmd.get('fixed_xy') and near and (not lid or not up):   # layers already stacked: press on the lowest
                        srt=sorted(near,key=lambda x:x[2]); lid=[srt[0]]; up=[srt[-1]]
                    if not lid or not up:
                        raise RuntimeError(f'Need a layer on the lid and one above it near the press point: {len(lid)} / {len(near)}')
                    # Press at the measured overlap: midway between the nearest lower and upper panel centres.
                    lo_=min(lid,key=lambda x:np.linalg.norm(x[:2]-xy)); hi_=min(up,key=lambda x:np.linalg.norm(x[:2]-xy))
                    xy=np.array([(lo_[0]+hi_[0])/2, xy[1]]) if cmd.get('fixed_xy') else (lo_[:2]+hi_[:2])/2; near=[lo_,hi_]   # x follows the measured strip
                    # Press the upper layer down onto the lower one: contact = lower layer + one sheet.
                    z_top=min(x[2] for x in lid)+.0015
                    a=np.asarray(cmd['approach'],float); a/=np.linalg.norm(a); Rp=frame(a,cmd.get('closing',[1,0,0]))
                    contact=np.array([xy[0],xy[1],z_top]); P=contact-.013*a
                    rise=max(x[2] for x in near)-z_top+.03
                    reports.append({'phase':'fingertip press plan','press_xy':xy.tolist(),'stack_top_m':z_top,'layers_near':len(near),'hover_up_m':rise})
                    # Hover straight above the press point and come straight down: a slanted descent drags the paper sideways.
                    # Leave the paper first: lift straight up from the backed-out pose so the joint route
                    # to the hover pose passes above the strip instead of sweeping through it.
                    p_up,R_up=ik.tcp_pose(current(side)[:6],station.BASES[side])
                    for lift_ in (() if cmd.get('no_lift') else (.10,.07,.05,.03)):
                        try:
                            stage(side,[W(1.2,p_up+np.array([0.,0.,lift_]),R_up,0.,'lift clear above the paper')],'lift clear above the paper'); break
                        except RuntimeError as exc:
                            if 'Preflight' not in str(exc): raise
                    rise=max(rise,.08)
                    for j_,v_ in enumerate(cmd.get('via_tcp',[])):
                        move_joint_target(side,np.asarray(v_,float),Rp,0.,f'approach from above ({j_})')
                    hovers=[] if cmd.get('slant_only') else [P+np.array([0.,0.,r_]) for r_ in [rise]+[v for v in (.06,.045,.03,.02) if v<rise]]
                    if cmd.get('slant_ok') or cmd.get('slant_only'):
                        hovers+= [P-d_*a for d_ in cmd.get('slant_backs',(.08,.06,.04,.03))]   # back off along the (tilted) approach instead
                    for hv in hovers:
                        try:
                            move_joint_target(side,hv,Rp,0.,'hover above the upper layer',q_seed=cmd.get('q_seed')); break
                        except RuntimeError as exc:
                            reports.append({'phase':'press hover rejected','hover':np.round(hv,3).tolist(),'reason':str(exc)[:80]})
                    else:
                        run.save({'stage':'continuous_partial_transport_not_scored','phases':reports})
                        raise RuntimeError('No reachable hover above the press point')
                    st2=run.state(); xs2=np.asarray(st2['panel_xyz'])
                    import tag_closure as _tc
                    from scipy.spatial.transform import Rotation as _Rot
                    Rs2=np.array([_Rot.from_quat(q_).as_matrix() for q_ in st2['panel_quat_xyzw']])
                    # Aim at the centre of the best back-to-back overlapping pair near the press point, so the
                    # fingertip pushes the layers flat instead of tilting them about an edge.
                    cands=[]
                    for a_ in range(len(xs2)):
                        for b_ in range(a_+2,len(xs2)):
                            mid=(xs2[a_]+xs2[b_])/2
                            if abs(mid[1]-xy[1])>.05 or abs(mid[0]-xy[0])>.09: continue
                            g_=_tc.pair_geometry(xs2,Rs2,a_,b_)
                            if g_['overlap_m2']>=_tc.MIN_OVERLAP_M2 and g_['normal_dot']<-.95 and abs(g_['face_gap_m'])<.02:
                                cands.append((g_['overlap_m2']-abs(g_['face_gap_m'])*.05,a_,b_,mid))
                    if cands and os.environ.get('BAGTAG_AIM_OVERLAP','0')=='1':
                        _,a_,b_,mid=max(cands)
                        shift=np.array([mid[0]-xy[0],mid[1]-xy[1],0.])
                        P=P+shift; reports.append({'phase':'press re-aim on the measured overlap','pair':[int(a_),int(b_)],'shift_m':np.round(shift,4).tolist()})
                    else:
                        near2=[x for x in xs2 if abs(x[1]-xy[1])<.03 and abs(x[0]-xy[0])<.09 and x[2]<1.2]
                        if near2:
                            dx=float(np.mean([x[0] for x in near2]))-float(xy[0])
                            P=P+np.array([dx,0.,0.]); reports.append({'phase':'press re-aim on the measured strip','dx_m':dx})
                    a_tilt=a.copy(); a=np.array([0.,0.,-1.])   # press along the vertical; the hand keeps its tilted orientation
                    run._last_press=(P.copy(),Rp.copy(),a_tilt.copy())
                    ov=float(cmd.get('overtravel_m',.003))
                    cmd['waypoints']=[dict(duration=t_,pos=p_.tolist(),approach=Rp[:,2].tolist(),closing=Rp[:,0].tolist(),finger=0.,label=l_)
                                      for t_,p_,l_ in [(2.,P,'push the upper layer down onto the lower one'),(.8,P+ov*a,'press the two adhesive backs together'),(float(cmd.get('hold_s',1.5)),P+ov*a,'hold the press'),(1.,P-float(cmd.get('lift_back_m',.03))*a_tilt,'lift the fingertips off')]]
                # Let go of the held panel and back the open fingers straight out across the strip width.
                if cmd.get('let_go_back_out'):
                    side=cmd['side']; p_,R_=ik.tcp_pose(current(side)[:6],station.BASES[side]); f_=float(cmd.get('open_finger',-.012))
                    away=np.asarray(cmd['lift_vec'],float) if cmd.get('lift_vec') else (np.array([0.,0.,float(cmd['lift_m'])]) if cmd.get('lift_m') else -float(cmd.get('back_m',.07))*R_[:,2])
                    cmd['waypoints']=[dict(duration=.8,pos=p_.tolist(),approach=R_[:,2].tolist(),closing=R_[:,0].tolist(),finger=f_,label='let go of the end'),
                                      dict(duration=1.5,pos=(p_+away).tolist(),approach=R_[:,2].tolist(),closing=R_[:,0].tolist(),finger=f_,label='lift straight off the paper' if cmd.get('lift_m') else 'back out across the width')]
                # Pinch both strands of the loop together, as a person zips a self-adhesive tag shut: measured
                # strand positions at height z, open jaws straddle both strands from the front edge, then close.
                if cmd.get('pinch_legs'):
                    from scipy.spatial.transform import Rotation
                    from handle_route_cpu import best_ik as _bik, obstacle_points as _op
                    from exit_guide_pickup_cpu import inside_any as _inside
                    import yam_hulls as _yh
                    from scipy.spatial import cKDTree as _KD
                    side=cmd['side']; _o='left' if side=='right' else 'right'; HLc=__import__('tag_dims').SEGMENT_LENGTH/2; HWc=.0508/2
                    st=run.state(); xs=np.asarray(st['panel_xyz']); Rs=np.array([Rotation.from_quat(q_).as_matrix() for q_ in st['panel_quat_xyzw']])
                    z=float(cmd.get('z',1.14))
                    def crossings(idx):
                        ys=[]
                        for k_ in idx:
                            a_=xs[k_]-HLc*Rs[k_][:,0]; b_=xs[k_]+HLc*Rs[k_][:,0]
                            if (a_[2]-z)*(b_[2]-z)<=0 and abs(b_[2]-a_[2])>1e-6:
                                ys.append(float(a_[1]+(z-a_[2])/(b_[2]-a_[2])*(b_[1]-a_[1])))
                        return ys
                    ya=crossings(cmd['leg_a']); yb=crossings(cmd['leg_b'])
                    if not ya or not yb:
                        raise RuntimeError(f'Strands do not both cross z={z:.3f}: {ya} {yb}')
                    ya_=max(ya); yb_=min(yb); gap=yb_-ya_
                    reports.append({'phase':'pinch legs: measured strands','z':z,'strand_a_y':ya,'strand_b_y':yb,'gap_m':gap})
                    if not 0.<gap<float(cmd.get('max_gap_m',.07)):
                        raise RuntimeError(f'Strand gap {gap*1e3:.0f} mm at z={z:.3f} is outside the jaw range')
                    Rg=frame([1,0,0],[0,-1,0])
                    P=np.array([float(np.mean(xs[:,0])),(ya_+yb_)/2,z]); pre=P-float(cmd.get('line_up_m',.07))*Rg[:,2]
                    _obs=_op(.008); other=_KD(np.vstack([h.vertices for pcs in _yh.hulls(current(_o),_o).values() for h in pcs]))
                    g_=np.array([(x_,y_,z_) for x_ in np.linspace(-HLc,HLc,9) for y_ in np.linspace(-HWc,HWc,9) for z_ in (-.0005,.0005)])
                    for lab,pp in (('line up',pre),('straddle',P)):
                        qq,pe,re=_bik(side,pp,Rg)
                        if pe>.001 or np.degrees(re)>1: raise RuntimeError(f'Pinch {lab}: no accurate IK')
                        pcs=_yh.hulls(np.r_[qq,-.047,-.047],side); hs_=[h for v_ in pcs.values() for h in v_]
                        if any(_inside(v_,hs_).any() for kk_,v_ in _obs.items() if kk_ in ('case','handle','kiosk')):
                            raise RuntimeError(f'Pinch {lab}: scene collision')
                        clr=float(other.query(np.vstack([h.vertices for h in hs_]))[0].min())
                        fingers=[h for n_ in ('tip_left','tip_right') for h in pcs[n_]]
                        cut=[k_ for k_ in range(len(xs)) if _inside(xs[k_]+g_@Rs[k_].T,fingers).any()]
                        reports.append({'phase':f'pinch legs: {lab} check','other_arm_m':clr,'open_fingers_cut_panels':cut})
                        if clr<float(cmd.get('min_margin_m',.008)): raise RuntimeError(f'Pinch {lab}: other arm clearance {clr*1e3:.0f} mm')
                        if cut: raise RuntimeError(f'Pinch {lab}: open fingers would cut panels {cut}')
                    move_joint_target(side,pre,Rg,-.047,'line up in front of both strands')
                    cmd['waypoints']=[dict(duration=t_,pos=p_.tolist(),approach=Rg[:,2].tolist(),closing=Rg[:,0].tolist(),finger=f_,label=l_)
                                      for t_,p_,f_,l_ in [(1.5,P,-.047,'straddle both strands'),(1.2,P,0.,'pinch both strands together'),(2.5,P,0.,'hold the pinch')]]
                if cmd.get('release_ready'):
                    side=cmd['side'];p0,R0=ik.tcp_pose(current(side)[:6],station.BASES[side])
                    pr,Rr=ik.tcp_pose(READY,station.BASES[side]);sg=1 if side=='left' else -1
                    opening=-.025 if side=='right' else -.006
                    release_p=p0+(float(cmd.get('release_shift_m',.025))*R0[:,0] if side=='right' else np.zeros(3))
                    back=release_p+(np.array([-.06,-.12,.045]) if side=='right' else np.array([-.08,0.,.025]))
                    if cmd.get('withdraw_along_approach_m'):
                        back=release_p-float(cmd['withdraw_along_approach_m'])*R0[:,2]
                    cmd['waypoints']=[dict(duration=t,pos=p.tolist(),approach=R[:,2].tolist(),closing=R[:,0].tolist(),finger=f,label='release then withdraw and return ready')
                        for t,p,R,f in [(1.,release_p,R0,opening),(2. if side=='right' else 1.5,back,R0,opening),(2.,back,frame([1,0,0],[0,0,-sg]),-.047),(1.5,np.array([.24,sg*.26,1.2]),frame([1,0,0],[0,0,-sg]),-.047),(2.,pr,Rr,-.047),(3.,pr,Rr,-.047)]]
                (out/'commands').mkdir(exist_ok=True)
                (out/'commands'/path.name).write_text(json.dumps(cmd,indent=2))
                waypoints = [W(w['duration'], w['pos'], frame(w['approach'], w['closing']),
                               w['finger'], w['label']) for w in cmd['waypoints']]
                stage(cmd['side'], waypoints, cmd.get('name', path.stem))
                if cmd.get('ensure_adhesive') and not run.state()['closure'].get('bonded'):
                    raise RuntimeError('Physical pressing failed; keep evidence and do not release')
            # Record a substantial live terminal interval so sampled review
            # frames show released attachment, rather than the last manipulation.
            _hold_n=int(os.environ.get('BAGTAG_FINAL_HOLD','320')); run.go(_hold_n,hold('left'),hold('right'),snaps=(run.step_i+_hold_n,),video_every=4)
            last=run.state()['closure']
            valid=last.get('score')==1. and last.get('hands_clear') and last.get('hands_open')
            run.save({'stage': ('staged_press_diagnostic_complete' if staged else
                               'continuous_full_demo_complete') if valid else 'demo_incomplete',
                      'terminal_success':bool(valid),'phases':reports})
            if not valid:raise RuntimeError('Final released stable loop not verified')
        # Keep this same physical rollout for bounded, state-informed feed
        # commands rather than repeating printer extraction for each probe.
        commands = os.environ.get('BAGTAG_COMMAND_FILE')
        if commands:
            seen = None
            deadline = time.monotonic()+900
            (out/'command_status.json').write_text(json.dumps({'state':'awaiting_command','step':run.step_i}))
            while time.monotonic() < deadline:
                try:
                    cmd = json.loads(Path(commands).read_text())
                except (FileNotFoundError,json.JSONDecodeError):
                    time.sleep(.2); continue
                if cmd.get('id') == seen:
                    time.sleep(.2); continue
                seen=cmd.get('id')
                if cmd.get('stop'): break
                try:
                    waypoints=[W(w['duration'],w['pos'],frame(w['approach'],w['closing']),w['finger'],w['label']) for w in cmd['waypoints']]
                    stage(cmd['side'],waypoints,cmd.get('name',str(seen)))
                    state={'state':'completed','id':seen,'step':run.step_i}
                except RuntimeError as e:
                    state={'state':'rejected','id':seen,'step':run.step_i,'reason':str(e)}
                (out/'command_status.json').write_text(json.dumps(state,indent=2)+'\n')
                deadline=time.monotonic()+900
    finally:
        run.close()


if __name__ == '__main__':
    main(Path(sys.argv[1]))
