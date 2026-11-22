"""Synthetic contact filtering and dwell regressions, not physics evidence."""
import numpy as np
import tag_closure as c

# Panel 0's adhesive back is its -z face; panel 9 lies against it back to back (flipped about x).
xyz=np.zeros((10,3)); xyz[9,2]=-.001
R=np.tile(np.eye(3),(10,1,1)); R[9]=np.diag([1.,-1.,-1.]); dt=.05
normal=np.array([0.,0.,1.])
contact=(np.array([0.,0.,-.0005]),normal,.075*normal)
assert np.isclose(c.back_press_force(xyz,R,0,[contact],dt),1.5)
assert c.back_press_force(xyz,R,0,[],dt)==0.
# Outside the panel footprint, or on its printed front (+z), the adhesive is not pressed.
for p in ([c.HALF_L+.003,0,-.0005],[0,c.HALF_W+.003,-.0005],[0,0,.0005],[0,0,.01]):
 assert c.back_press_force(xyz,R,0,[(np.array(p),normal,.075*normal)],dt)==0.
assert c.back_press_force(xyz,R,0,[(contact[0],np.array([1.,0,0]),np.array([.075,0,0]))],dt)==0.
# Pair geometry: back to back is a candidate; back on a printed front (same orientation) is not,
# and neither is panel 9 on panel 0's front.
assert c.pair_geometry(xyz,R,0,9)['contact']
same=R.copy();same[9]=np.eye(3)
assert not c.pair_geometry(xyz,same,0,9)['contact'], 'back on front must not bond (cohesive adhesive)'
# The whole back is adhesive: crossed strips bond if enough area overlaps, at any in-plane angle.
from scipy.spatial.transform import Rotation as _Rot
for ang in (30,60,90):
 cr=R.copy(); cr[9]=_Rot.from_euler('z',ang,degrees=True).as_matrix()@np.diag([1.,-1.,-1.])
 g=c.pair_geometry(xyz,cr,0,9); assert g['contact'] and g['overlap_m2']>=c.MIN_OVERLAP_M2,(ang,g)
off=xyz.copy(); off[9,0]=.045
assert not c.pair_geometry(off,R,0,9)['contact'], 'under 600 mm2 of overlap must not bond'
front=xyz.copy();front[9,2]=+.001
assert not c.pair_geometry(front,R,0,9)['contact']
for force in (0.,.1962,.49):   # below the firm-press threshold: no firm bond
 dwell=0.
 for _ in range(40):dwell=c.press_dwell(dwell,True,0.,force,dt)
 assert dwell==0.
for force in (0.,.1962,.24):   # below the tack threshold: not even a tack
 dwell=0.
 for _ in range(40):dwell=c.press_dwell(dwell,True,0.,force,dt,c.TACK_PRESS_N)
 assert dwell==0.
dwell=0.
for _ in range(8):dwell=c.press_dwell(dwell,True,0.,.3,dt,c.TACK_PRESS_N)
assert dwell+1e-9>=c.PRESS_SECONDS   # a light 0.3 N contact held 0.4 s tacks
dwell=0.
for _ in range(8):dwell=c.press_dwell(dwell,True,0.,1.5,dt)
assert dwell+1e-9>=c.PRESS_SECONDS
assert c.press_dwell(dwell,False,0.,1.5,dt)==0.
assert c.press_dwell(dwell,True,.1,1.5,dt)==0.
assert c.press_dwell(dwell,True,0.,0.,dt)==0.
c.reset()
try:c.create_bond(None,xyz,R,0,9)
except RuntimeError:pass
else:raise AssertionError('No contact must not create bond')
for bad in (float("nan"), float("inf")):
 assert c.press_dwell(0.,True,0.,bad,.05)==0.
 assert c.back_press_force(xyz,R,0,[contact],bad)==0.
assert c.back_press_force(xyz,R,0,[contact],0.)==0.
print('Press gate CPU checks passed: no-force, weak-force, off-panel, printed-front, tangential, back-to-back only (back on front never bonds), dwell reset, guarded bond, non-finite inputs')
