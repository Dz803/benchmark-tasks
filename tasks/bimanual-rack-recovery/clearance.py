"""Conservative AABB distances, not collision-force or mesh-contact claims."""
import numpy as np
SIGNS=np.array([[x,y,z]for x in[-1,1]for y in[-1,1]for z in[-1,1]])
def bounds(center,size,R=np.eye(3),p=np.zeros(3)):
    v=(SIGNS*np.array(size)/2+center)@R.T+p
    return v.min(0),v.max(0)
def gap(a,b):return float(np.linalg.norm(np.maximum(np.maximum(a[0]-b[1],b[0]-a[1]),0.)))
def carrier_lip_gap(task,p,R):
    carrier=next(o for o in task.OBJECTS if o.name=='carrier');lip=next(o for o in task.OBJECTS if o.name=='retaining_lip')
    return min(gap(bounds(np.array(a[0]),a[1],R,p),bounds(np.array(b[0]),b[1],p=np.array(lip.pos)))for a in carrier.boxes for b in lip.boxes)
def carried_obstacle_gap(task,states,obstacle):
    obs=next(o for o in task.OBJECTS if o.name==obstacle);ob=[bounds(np.array(b[0]),b[1],p=np.array(obs.pos))for b in obs.boxes];result=float('inf')
    for obj in task.OBJECTS:
        if obj.name not in states:continue
        st=states[obj.name];R=st.rotation()
        for b in obj.boxes:result=min(result,min(gap(bounds(np.array(b[0]),b[1],R,st.pos),v)for v in ob))
        for c in obj.cylinders:
            size=np.array([2*c[1]]*3);size['XYZ'.index(c[4])]=c[2]
            result=min(result,min(gap(bounds(np.array(c[0]),size,R,st.pos),v)for v in ob))
    return result
