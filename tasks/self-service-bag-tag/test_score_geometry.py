"""Adversarial CPU predicates; no claim of physical rollout validation."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import tag_closure as s
root=Path(__file__).parent
row=json.loads((root/('score_fixture_14.json' if (root/'score_fixture_14.json').exists() else 'score_fixture.json')).read_text())
(root/'runs').mkdir(exist_ok=True)
x=np.array(row['panel_xyz']);R=Rotation.from_quat(row['panel_quat_xyzw']).as_matrix()
if len(x)!=s.SEGMENTS:   # fixture recorded with 10 panels: resample along the same piecewise-rigid strip
 L0=.533/len(x);L=2*s.HALF_L;ks=[min(len(x)-1,int(((j+.5)*L)//L0)) for j in range(s.SEGMENTS)]
 x=np.array([x[k]+((k+.5)*L0-(j+.5)*L)*R[k][:,0] for j,k in enumerate(ks)]);R=R[ks]
assert s.barcode_exposed(x,R,1.0965)
# Put a separate paper face above each printed panel: normal alone must not pass.
y=x.copy();Q=R.copy()
for src,dst in ((s.BARCODE_PANELS[0],s.BARCODE_PANELS[0]+4),(s.BARCODE_PANELS[1],s.BARCODE_PANELS[1]+4)):
 n=R[src,:,2]; n=n if n[2]>0 else -n
 y[dst]=x[src]+.003*n;Q[dst]=R[src]
assert not s.barcode_exposed(y,Q,1.0965)
# Rigid-centre rotation and returning oscillation must fail the speed gate.
c=s.panel_corners(x,R)
assert s.motion_speed([c,c+np.array([0,0,.01]),c],.05)>.015
Q=R.copy();Q[1]=R[1]@Rotation.from_euler('x',20,degrees=True).as_matrix()
assert s.motion_speed([c,s.panel_corners(x,Q)],.05)>.015
assert s.motion_speed([c,c],.05)==0
# An invalid barcode/extraction must reset a continuous hold, not merely gate output.
for invalid_index in (0,4):
 args=[True,True,False,True,True,True,True,0.]
 stable=1.95
 args[invalid_index]=False
 stable=stable+.05 if s.final_valid(*args) else 0.
 assert stable==0
 args[invalid_index]=True
 stable=stable+.05 if s.final_valid(*args) else 0.
 assert s.outcome_score(True,True,False,True,True,stable,True)==.75
s.reset();s.STATE['bonded']=True
s.on_joint_break('/World/printer/frangible_joint');assert not s.STATE['broken']
s.on_joint_break(s.JOINT_PATH);assert s.STATE['broken']
assert s.outcome_score(True,True,s.STATE['broken'],True,True,3.)==0
out={'passed':True,'kind':'CPU synthetic predicates only','checks':['final known geometry exposed','paper occludes printed panels','rotating corners','returning oscillation','barcode resets hold','extraction resets hold','printer break ignored','adhesive break latches zero']}
(root/'runs/score-geometry-regression.json').write_text(json.dumps(out,indent=2))
print(out)
