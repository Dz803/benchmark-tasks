"""Diagnostic PhysX contacts between robot links and tag panels (release/withdrawal audits only).

Reads native contact impulses; never changes any state. One rigid contact view per robot link,
filtered to the ten tag panel bodies.
"""
import numpy as np

LINKS = ('tip_left', 'tip_right', 'gripper', 'link5', 'link4')
from tag_dims import SEGMENTS
_VIEWS = None


def _array(v):
    if hasattr(v, 'detach'):
        return v.detach().cpu().numpy()
    return np.asarray(v.numpy() if hasattr(v, 'numpy') else v)


def _setup(env):
    global _VIEWS
    from isaaclab.sim.utils.stage import get_current_stage
    from isaaclab_physx.physics.physx_manager import PhysxManager
    from pxr import UsdPhysics
    panels = {}
    for p in get_current_stage().Traverse():
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            for i in range(SEGMENTS):
                if f'/tag_panel_{i:02d}/body' in str(p.GetPath()):
                    panels[i] = str(p.GetPath())
    bodies = [panels[i] for i in range(SEGMENTS)]
    sim_view = PhysxManager.get_physics_sim_view()
    scene = env.unwrapped.scene
    _VIEWS = []
    for side in ('left', 'right'):
        for path in scene[f'{side}_robot'].root_physx_view.link_paths[0]:
            name = path.rsplit('/', 1)[-1]
            if name in LINKS:
                _VIEWS.append((side, name, sim_view.create_rigid_contact_view(
                    [path], filter_patterns=[bodies], max_contact_data_count=256)))


def read(env, dt):
    """[(side, link, panel, force_N)] for every robot-link/panel pair in contact now."""
    if _VIEWS is None:
        _setup(env)
    out = []
    for side, link, view in _VIEWS:
        force, _p, _n, _s, count, start = map(_array, view.get_contact_data(dt=1.))
        count = count.reshape(-1); start = start.reshape(-1)
        for k in range(SEGMENTS):
            f = sum(abs(float(np.asarray(force[i]).reshape(-1)[0])) for i in range(int(start[k]), int(start[k] + count[k])))
            if f > 0:
                out.append((side, link, k, f / dt))
    return out
