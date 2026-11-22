"""Contact-gated adhesive approximation and outcome scoring; no pose writes.

The whole back (-z face) of the tag carries a linerless, self-only (cohesive) adhesive: it bonds
only to the same adhesive. Two non-neighbouring panels bond only back to back, after aligned,
physically pressed back-on-back contact for 0.4 s; a back pressed on a printed front, the case,
the handle or a gripper never bonds. Chemistry and peel mechanics
are simplified as a breakable fixed joint, created at the *existing* relative transform.
"""
import numpy as np
from scipy.spatial.transform import Rotation

STATE = {}
JOINT_PATH = '/World/baggage_tag_adhesive'   # one joint per bonded pair: f'{JOINT_PATH}_{a:02d}_{b:02d}'
from tag_dims import SEGMENTS, SEGMENT_LENGTH, BARCODE_PANELS
HALF_L = SEGMENT_LENGTH / 2   # panel half-length (533 mm stock, see tag_dims)
HALF_W = .0508 / 2          # panel half-width (2 in stock)
MIN_OVERLAP_M2 = .00060
_EVENT_SUB = None
MIN_PRESS_N = float(__import__("os").environ.get("BAGTAG_FIRM_N", "0.5"))   # seal: a deliberate press (the fingertip press here peaks at about 1-2 N but does not hold 1 N for 0.4 s)
TACK_PRESS_N = 0.25    # tack: light contact makes a weak bond that a firm press completes
TACK_BREAK = (2.0, .05)   # N, N m
FIRM_BREAK = (8.0, .15)
PRESS_SECONDS = .4
RELEASE_SECONDS = .25
SUCCESS_SECONDS = 2.


def reset(environment=None):
    global _EVENT_SUB
    _EVENT_SUB = None
    import press_contacts
    press_contacts.reset()
    STATE.clear()
    STATE.update(contact_s=0., stable_s=0., bonded=False, broken=False,
                 extracted=False, threaded=False, score=0., diagnostics={},
                 dwell={}, bonds={})
    if environment is not None:
        from isaaclab.sim.utils.stage import get_current_stage
        stage=get_current_stage()
        for prim in list(stage.Traverse()):
            if str(prim.GetPath()).startswith(JOINT_PATH): stage.RemovePrim(prim.GetPath())
        import carb
        carb.settings.get_settings().set_bool('/physics/disableContactProcessing', False)


def _clip(subject, clip):
    """Sutherland-Hodgman: intersection of a polygon with a convex clip polygon (2D, CCW)."""
    out = list(subject)
    for i in range(len(clip)):
        a, b = clip[i], clip[(i + 1) % len(clip)]
        inp, out = out, []
        if not inp:
            break
        side = lambda p: (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        for j in range(len(inp)):
            p, q = inp[j], inp[(j + 1) % len(inp)]
            sp, sq = side(p), side(q)
            if sp >= 0:
                out.append(p)
            if (sp >= 0) != (sq >= 0):
                t = sp / (sp - sq)
                out.append((p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])))
    return out


def _area_centroid(poly):
    if len(poly) < 3:
        return 0., (0., 0.)
    x = np.array([p[0] for p in poly]); y = np.array([p[1] for p in poly])
    cr = x * np.roll(y, -1) - np.roll(x, -1) * y
    area = cr.sum() / 2
    if abs(area) < 1e-12:
        return 0., (0., 0.)
    return abs(area), (float(((x + np.roll(x, -1)) * cr).sum() / (6 * area)), float(((y + np.roll(y, -1)) * cr).sum() / (6 * area)))


def pair_geometry(xyz, rotations, a, b):
    """Panel b against panel a's adhesive back, back to back, at any in-plane angle.

    The whole back is adhesive, so the ends need not line up: the overlap is the true polygon
    intersection of the two panels in a's plane. Back to back means opposed normals (within 15
    degrees) and, at the overlap centroid, b's mid-plane on a's back side within the 1 mm
    collision-sheet contact band (and vice versa)."""
    R = np.asarray(rotations); xyz = np.asarray(xyz)
    na, nb = R[a, :, 2], R[b, :, 2]
    align = float(np.dot(na, nb))
    rect = [(-HALF_L, -HALF_W), (HALF_L, -HALF_W), (HALF_L, HALF_W), (-HALF_L, HALF_W)]
    corners_b = [xyz[b] + R[b] @ np.array([x, y, 0.]) for x, y in rect]
    proj = [tuple((R[a].T @ (p - xyz[a]))[:2]) for p in corners_b]
    if _area_centroid(proj)[0] > 0 and np.cross(np.subtract(proj[1], proj[0]), np.subtract(proj[2], proj[1])) < 0:
        proj = proj[::-1]
    area, (cx, cy) = _area_centroid(_clip(proj, rect))
    pt = xyz[a] + R[a] @ np.array([cx, cy, 0.])
    den = float(np.dot(nb, na))
    gap_a = -float(np.dot(nb, pt - xyz[b])) / den if abs(den) > .5 else 1.
    contact = (align < -.966 and area >= MIN_OVERLAP_M2 and -.0018 <= gap_a <= -.00055)
    return dict(face_gap_m=float(gap_a), overlap_m2=float(area), normal_dot=align, contact=bool(contact))


def back_press_force(xyz, rotations, a, contacts, dt):
    """Compressive force on panel a's adhesive back from another tag panel, during the sampled substep.

    Points must lie within panel a's footprint on its back (-z) side, with a surface-normal contact.
    Normal impulses divided by elapsed simulated time are forces, not counts.
    """
    if not np.isfinite(dt) or dt <= 0:
        return 0.
    total = 0.
    for point, normal, impulse in contacts:
        if not all(np.all(np.isfinite(v)) for v in (point, normal, impulse)):
            continue
        if abs(normal @ rotations[a, :, 2]) < .98:
            continue
        p = rotations[a].T @ (point - xyz[a])
        if abs(p[0]) <= HALF_L and abs(p[1]) <= HALF_W and -.002 <= p[2] <= 0.:
            total += abs(float(impulse @ normal))
    return total / dt


def press_dwell(previous, geometry_contact, speed, force, dt, threshold=None):
    threshold = MIN_PRESS_N if threshold is None else threshold
    valid = (np.all(np.isfinite([previous, speed, force, dt])) and
             previous >= 0. and dt > 0. and 0. <= speed < .04)
    if not (valid and geometry_contact):
        return 0.
    if force >= threshold:
        return previous + dt
    # Contact chatter: a brief dip to within 20 % below the threshold pauses the dwell instead of resetting it.
    return previous if force >= .8 * threshold else 0.


def geometry(xyz, rotations, bar_x=.350, bar_z=1.0965, pair=(0, SEGMENTS - 1)):
    """Loop geometry for the closing pair (a, b): the strip from panel min(a,b) to max(a,b), with
    panel a's adhesive back on panel b. The default (0, last) is the end-to-end loop."""
    xyz=np.asarray(xyz); R=np.asarray(rotations)
    pa, pb = pair
    pg = pair_geometry(xyz, R, pa, pb)
    # Every hinge must still join its two actual anchors.
    anchors=[np.linalg.norm(xyz[i-1]-HALF_L*R[i-1,:,0]-(xyz[i]+HALF_L*R[i,:,0])) for i in range(1,SEGMENTS)]
    lo, hi = min(pair), max(pair)
    # Follow panel edges through each physical hinge, rather than cutting
    # diagonally between centres across a sharp fold.
    curve=np.concatenate([np.array([p+HALF_L*r[:,0],p,p-HALF_L*r[:,0]]) for p,r in zip(xyz[lo:hi+1],R[lo:hi+1])])
    yz=curve[:,1:3]-[0.,bar_z]
    nxt=np.roll(yz,-1,axis=0)
    winding=float(np.sum(np.arctan2(yz[:,0]*nxt[:,1]-yz[:,1]*nxt[:,0],np.sum(yz*nxt,axis=1)))/(2*np.pi))
    under=False
    for a,b in zip(curve[:-1],curve[1:]):
        if a[1]*b[1]<0:
            t=-a[1]/(b[1]-a[1]);p=a+t*(b-a)
            if abs(p[0]-bar_x)<.065 and 1.0668<p[2]<bar_z-.003:under=True
    linked=abs(winding)>.8 and under and np.all(np.abs(xyz[lo:hi+1,0]-bar_x)<.075)
    # Only the front is printed. Require a complete sampled barcode face to have
    # a clear outward line of sight, not merely an outward panel-centre normal.
    barcode_outward = barcode_exposed(xyz, R, bar_z)
    return dict(barcode_outward=bool(barcode_outward),face_gap_m=pg['face_gap_m'],overlap_m2=pg['overlap_m2'],
                normal_dot=pg['normal_dot'],pair=[int(pa),int(pb)],
                end_distance_m=float(np.linalg.norm(xyz[pb]-xyz[pa])),
                max_hinge_gap_m=float(max(anchors)),contact=pg['contact'],
                under=bool(under),linked=bool(linked),winding=winding)


def panel_corners(xyz, rotations):
    local = np.array([[x,y,0.] for x in (-HALF_L,HALF_L) for y in (-HALF_W,HALF_W)])
    return np.asarray(xyz)[:,None,:] + np.einsum('nij,kj->nki',rotations,local)


def barcode_exposed(xyz, R, bar_z):
    """Nine distributed surface samples, obstructed by any other paper panel.

    This is a finite geometric exposure test, not OCR or a visibility proof.
    Only upward exterior views are accepted, keeping rays clear of the shell.
    """
    for i in BARCODE_PANELS:
        radial = xyz[i] - np.array([xyz[i,0],0.,bar_z])
        for sign in (1.,):          # the thermal print is on the front (+z) only
            n = sign*R[i,:,2]
            if n @ radial <= .01 or n[2] < .1 or xyz[i,2] < 1.070:
                continue
            visible = True
            for x in (-.014,0.,.014):
                for y in (-.0165,0.,.0165):
                    origin = xyz[i]+x*R[i,:,0]+y*R[i,:,1]+.0008*n
                    for j in range(SEGMENTS):
                        if j == i: continue
                        o=R[j].T@(origin-xyz[j]);v=R[j].T@n
                        # Ray / oriented thin box, including grazing cases.
                        lo,hi=0.,.30
                        for k,h in enumerate((HALF_L,HALF_W,.0007)):
                            if abs(v[k])<1e-10:
                                if abs(o[k])>h: hi=-1.; break
                            else:
                                a,b=sorted(((-h-o[k])/v[k],(h-o[k])/v[k]))
                                lo=max(lo,a);hi=min(hi,b)
                        if hi>=lo and hi>=0.: visible=False; break
                    if not visible: break
                if not visible: break
            if visible: return True
    return False


def motion_speed(history, dt):
    """Largest consecutive corner displacement: detects rotation and oscillation."""
    if len(history)<2: return float('inf')
    return float(np.max(np.linalg.norm(np.diff(np.asarray(history),axis=0),axis=-1))/dt)


def final_valid(extracted, bonded, broken, linked, barcode, opened, clear, speed):
    return bool(extracted and bonded and not broken and linked and barcode
                and opened and clear)


def on_joint_break(path):
    # Printer retention is intentionally breakable; only a seal is fatal here.
    if str(path).startswith(JOINT_PATH):
        if str(path).endswith('_tack'):
            key=tuple(int(v) for v in str(path)[len(JOINT_PATH)+1:-5].split('_'))
            if STATE.get('bonds',{}).get(key,{}).get('firm'):
                return            # the firm bond took over; the tack giving way is harmless
            STATE.setdefault('bonds',{}).pop(key, None)   # a lost tack is simply no bond yet
            return
        STATE['broken'] = True
        STATE['bond_break_event'] = True


def monitor_bond_breaks():
    global _EVENT_SUB
    from omni.physx import get_physx_interface
    from omni.physx.bindings._physx import SimulationEvent
    from pxr import PhysicsSchemaTools
    def on_event(event):
        if event.type == int(SimulationEvent.JOINT_BREAK):
            pair=event.payload['jointPath']
            on_joint_break(PhysicsSchemaTools.decodeSdfPath(pair[0],pair[1]))
    _EVENT_SUB = get_physx_interface().get_simulation_event_stream_v2().create_subscription_to_pop(on_event)


def create_bond(environment, xyz, rotations, a, b, firm=True):
    dwell = STATE['dwell'] if firm else STATE.setdefault('tack_dwell', {})
    if dwell.get((a, b), 0.) + 1e-9 < PRESS_SECONDS:
        raise RuntimeError('Bond requires verified physical pressing dwell')
    from pxr import Gf,Sdf,UsdPhysics
    from isaaclab.sim.utils.stage import get_current_stage
    stage=get_current_stage();paths={}
    names=(f'tag_panel_{a:02d}', f'tag_panel_{b:02d}')
    for prim in stage.Traverse():
        if prim.GetName() in names:
            body=str(prim.GetPath())+'/body'
            if stage.GetPrimAtPath(body):paths[prim.GetName()]=body
    if len(paths)!=2:raise RuntimeError('Cannot locate both tag bodies for contact bond')
    if _EVENT_SUB is None: monitor_bond_breaks()
    path=f'{JOINT_PATH}_{min(a,b):02d}_{max(a,b):02d}'+('' if firm else '_tack')
    joint=UsdPhysics.FixedJoint.Define(stage,path)
    anchor=(xyz[a]+xyz[b])/2
    for slot,i in enumerate((a,b)):
        getattr(joint,f'CreateBody{slot}Rel')().SetTargets([Sdf.Path(paths[f'tag_panel_{i:02d}'])])
        local=rotations[i].T@(anchor-xyz[i]);q=Rotation.from_matrix(rotations[i].T).as_quat()
        getattr(joint,f'CreateLocalPos{slot}Attr')(Gf.Vec3f(*local))
        getattr(joint,f'CreateLocalRot{slot}Attr')(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3])))
    brk=FIRM_BREAK if firm else TACK_BREAK
    joint.CreateBreakForceAttr(brk[0]);joint.CreateBreakTorqueAttr(brk[1])
    # A tack keeps the layers in contact so that a later firm press is still measured.
    joint.CreateCollisionEnabledAttr(not firm)


def linking_bond(xyz, R, bar_z):
    """Loop geometry of the bonded pair whose closed strip goes round the handle (widest pair first)."""
    best = None
    for key, info in sorted([kv for kv in STATE['bonds'].items() if kv[1].get('firm', True)] or list(STATE['bonds'].items()), key=lambda kv: -abs(kv[0][0] - kv[0][1])):
        g = geometry(xyz, R, bar_z=bar_z, pair=(info['adhesive_panel'], info['other_panel']))
        if best is None:
            best = g
        if g['linked']:
            return g
    return best


def step(environment):
    from yam_arena.scene import object_states
    from yam_arena import ik,station
    st=object_states(environment,[f'tag_panel_{i:02d}' for i in range(SEGMENTS)]+['checked_suitcase'])
    panels=[st[f'tag_panel_{i:02d}'] for i in range(SEGMENTS)]
    xyz=np.array([p.pos for p in panels]);R=np.array([Rotation.from_quat(p.quat_xyzw).as_matrix() for p in panels])
    bar_z=1.0965+st['checked_suitcase'].sliders['carry_handle']
    raw_speed=max(float(np.linalg.norm(p.lin_vel)) for p in panels)
    dt=float(getattr(environment.unwrapped, 'step_dt', .05))
    sim_dt=float(environment.unwrapped.cfg.sim.dt)
    # Use consecutive corner motions over the last 0.25 s. Raw root velocity
    # can retain solver impulses even when constrained panel poses are static.
    history=STATE.setdefault('position_history', [])
    history.append(panel_corners(xyz,R))
    if len(history)>6: history.pop(0)
    speed=motion_speed(history,dt)
    # Per-panel speed: the press dwell only needs the two pressed panels to be still, not the whole strip.
    if len(history)>=2:
        hist=np.asarray(history); panel_speed=np.max(np.linalg.norm(np.diff(hist,axis=0),axis=-1),axis=(0,2))/dt
    else:
        panel_speed=np.full(len(xyz),np.inf)
    # Headless GPU simulation does not dispatch contact subscription events.
    # Poll the latest physics substep explicitly, using its dt (not control dt).
    import press_contacts
    pair_contacts = press_contacts.read()
    # Extraction uses the actual outlet plane; no policy flags or phase IDs.
    STATE['extracted']=bool(np.max(panel_corners(xyz,R)[:,:,0])<.465)
    g_full=geometry(xyz,R,bar_z=bar_z)
    if g_full['under'] and np.min(xyz[:,1])<-.025 and np.max(xyz[:,1])>.025:
        STATE['threaded']=True
    # Adhesive back of panel a pressed on panel b: any non-neighbouring pair, either order.
    # Bonded pairs have collisions disabled, so their cached contacts are skipped.
    best_press = (0., None)
    for a in range(SEGMENTS):
        for b in range(a + 2, SEGMENTS):   # back to back is symmetric: test each pair once
            key = (min(a, b), max(a, b))
            if key in STATE['bonds'] and STATE['bonds'][key].get('firm'):
                continue
            pg = pair_geometry(xyz, R, a, b)
            # Both backs must carry the press (cohesive adhesive on each mating face).
            force = min(back_press_force(xyz, R, a, pair_contacts.get(key, []), sim_dt),
                        back_press_force(xyz, R, b, pair_contacts.get(key, []), sim_dt))
            if force > best_press[0]:
                best_press = (force, (a, b))
            # Bond formation needs the two mating panels still relative to each other (sliding smears the adhesive).
            if len(history)>=2:
                rel=hist[:,a]-hist[:,b]
                pair_speed=float(np.max(np.linalg.norm(np.diff(rel,axis=0),axis=-1))/dt)
            else:
                pair_speed=float('inf')
            # Only the last physics substep of each control step is sampled, and paper-on-paper contact
            # impulses arrive intermittently between substeps: use the peak of the last 3 samples (0.15 s).
            fh=STATE.setdefault('force_hist',{}).setdefault((a,b),[]); fh.append(force); del fh[:-3]
            force=max(fh)
            STATE['dwell'][(a, b)] = press_dwell(STATE['dwell'].get((a, b), 0.), pg['contact'], pair_speed, force, dt)
            td = STATE.setdefault('tack_dwell', {})
            td[(a, b)] = press_dwell(td.get((a, b), 0.), pg['contact'], pair_speed, force, dt, TACK_PRESS_N)
            if STATE['dwell'][(a, b)] + 1e-9 >= PRESS_SECONDS:
                create_bond(environment, xyz, R, a, b, firm=True)
                STATE['bonds'][key] = dict(adhesive_panel=a, other_panel=b, press_n=force, firm=True)
                STATE['bonded'] = True
                STATE.setdefault('bond_press_n', force)
            elif key not in STATE['bonds'] and td[(a, b)] + 1e-9 >= PRESS_SECONDS:
                create_bond(environment, xyz, R, a, b, firm=False)
                STATE['bonds'][key] = dict(adhesive_panel=a, other_panel=b, press_n=force, firm=False)
    STATE['contact_s'] = max(STATE['dwell'].values(), default=0.)
    if g_full['max_hinge_gap_m']>.008:
        STATE['broken']=True
    for key, info in STATE['bonds'].items():
        pg = pair_geometry(xyz, R, info['adhesive_panel'], info['other_panel'])
        # The fixed joint holds the bonded panels' relative pose; allow numerical jitter around the
        # formation thresholds so a bond made at exactly 600 mm2 is not flagged on a 1 % flicker.
        if (pg['overlap_m2']<.8*MIN_OVERLAP_M2 or pg['normal_dot']>-.94 or
                not -.004<=pg['face_gap_m']<=.002):
            STATE['broken']=True
    g = linking_bond(xyz, R, bar_z) if STATE['bonds'] else g_full
    base=environment.unwrapped;clear=True;opened=True
    for side in ('left','right'):
        robot=base.scene[side+'_robot'];q=robot.data.joint_pos.torch[0].detach().cpu().numpy()
        opened &= bool(np.mean(q[-2:])<-.025)
        tcp,_=ik.tcp_pose(q[:6],station.BASES[side])
        clear &= bool(np.min(np.linalg.norm(xyz-tcp,axis=1))>.12)
        poses=robot.data.body_pose_w.torch[0].detach().cpu().numpy()
        for name in ('tip_left','tip_right','gripper'):
            p=poses[robot.body_names.index(name),:3]
            clear &= bool(np.min(np.linalg.norm(xyz-p,axis=1))>.075)
    final=final_valid(STATE['extracted'],STATE['bonded'],STATE['broken'],
                      g['linked'],g['barcode_outward'],opened,clear,speed)
    corners=panel_corners(xyz,R)
    if not final:
        STATE['hold_anchor']=None
        STATE['stable_s']=0.
    elif STATE.get('hold_anchor') is None:
        STATE['hold_anchor']=corners.copy()
        STATE['stable_s']=0.
    elif np.max(np.linalg.norm(corners-STATE['hold_anchor'],axis=-1)) > .005:
        STATE['hold_anchor']=corners.copy()
        STATE['stable_s']=0.
    else:
        STATE['stable_s']+=dt
    released_stable=final and STATE['stable_s']>=RELEASE_SECONDS
    value=outcome_score(STATE['extracted'], STATE['bonded'], STATE['broken'],
                        g['linked'], released_stable, STATE['stable_s'], g['barcode_outward'])
    STATE['score']=value
    n_contacts=sum(len(v) for v in pair_contacts.values())
    STATE['diagnostics']={**g,'speed_m_s':speed,'raw_solver_speed_m_s':raw_speed,'hands_clear':bool(clear),'hands_open':bool(opened),
                         'bonded':STATE['bonded'],
                         'bonds':[[k[0],k[1],v['adhesive_panel'],float(v['press_n']),bool(v.get('firm',True))] for k,v in STATE['bonds'].items()],
                         'broken':STATE['broken'],
                         'bond_break_event':STATE.get('bond_break_event',False),'contact_s':STATE['contact_s'],
                         'patch_press_n':best_press[0],'press_pair':best_press[1],'patch_contact_points':n_contacts,
                         'bond_press_n':STATE.get('bond_press_n',0.),
                         'stable_s':STATE['stable_s'],'score':value,
                         'tack_s':max(STATE.get('tack_dwell',{}).values(),default=0.)}


def outcome_score(extracted, bonded, broken, linked, released_stable, stable_s, barcode_outward=True):
    """Current physical outcome only; extraction/threading alone earn nothing."""
    if broken or not (extracted and bonded and linked and barcode_outward):
        return 0.
    if not released_stable or not np.isfinite(stable_s) or stable_s < RELEASE_SECONDS:
        return .75
    return 1. if stable_s >= SUCCESS_SECONDS else .85


def score(_objects):return float(STATE.get('score',0.))
def diagnostics():return dict(STATE.get('diagnostics',{}))
reset()
